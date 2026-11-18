#!/usr/bin/env python3
"""Every-frame, unsmoothed candidate video review; never trains or deploys.

The source manifest is the hash-verified download manifest. Outputs are an
H.264 MP4, a JSONL prediction trace, and a uniform storyboard for every clip.
Output-presence statistics are NOT accuracy, recall, or verified misses.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def output_state(predictions: list[dict]) -> str:
    """Describe output, not scene truth. Multiple lights remain explicit."""
    signals = sorted({d.get('signal') or 'unknown' for d in predictions
                      if d['class'] == 'traffic_light'})
    return '|'.join(signals) if signals else 'no_detection'


def state_runs(states: list[str], fps: float) -> list[dict]:
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('fps must be finite and positive')
    runs = []
    for index, state in enumerate(states):
        if not runs or runs[-1]['state'] != state:
            runs.append({'state': state, 'startFrame': index, 'endFrameExclusive': index + 1})
        else:
            runs[-1]['endFrameExclusive'] = index + 1
    for run in runs:
        run['startS'] = round(run['startFrame'] / fps, 6)
        run['endS'] = round(run['endFrameExclusive'] / fps, 6)
        run['frames'] = run['endFrameExclusive'] - run['startFrame']
    return runs


def run(args: argparse.Namespace) -> dict:
    import cv2
    import imageio_ffmpeg
    from PIL import Image, ImageDraw, ImageFont, ImageOps
    import torch
    import ultralytics
    from robot_control.vision.detector import YoloDetector

    profile = json.loads(args.profile.read_text())
    if profile['backend'] != 'yolo+fusion':
        raise ValueError('video review requires the saved yolo+fusion profile')
    model_sha = sha256(args.weights)
    if model_sha != profile['sha256']:
        raise ValueError('checkpoint differs from saved candidate evaluation')
    if args.weights.resolve() == (ROOT / 'models/vision/yolo-stop.pt').resolve():
        raise ValueError('Use the candidate checkpoint, not production')
    source = json.loads(args.manifest.read_text())
    samples = json.loads((ROOT / 'datasets/yolo-stop/samples.json').read_text())
    splits: dict[str, set] = {}
    for sample in samples.values():
        if sample['kind'] == 'video':
            splits.setdefault(sample['source'], set()).add(sample['split'])
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    detector = YoloDetector(str(args.weights), confidence=profile['confidence'],
                            imgsz=profile['imgsz'], prefer_red=profile['preferRed'], device='cpu',
                            traffic_mode=profile.get('trafficMode', 'legacy'))
    args.output.mkdir(parents=True, exist_ok=True)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 18)
    small_font = ImageFont.truetype(font_path, 15)
    bold = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 20)
    colors = {'stop_sign': '#ff6387', 'speed_bump': '#ffe073', 'cone': '#ffa965',
              'pedestrian_crossing': '#78c7ff', 'red': '#ff6387', 'yellow': '#ffe073',
              'green': '#62e6a7', 'unknown': '#c6afff'}
    labels = {'stop_sign': 'STOP', 'speed_bump': 'НЕРОВНОСТЬ', 'cone': 'КОНУС',
              'pedestrian_crossing': 'ПЕШЕХОДНЫЙ', 'red': 'КРАСНЫЙ', 'yellow': 'ЖЁЛТЫЙ',
              'green': 'ЗЕЛЁНЫЙ', 'unknown': 'ЦВЕТ НЕИЗВЕСТЕН'}
    clips = []
    for item in source['videos']:
        path = args.source / item['name']
        if sha256(path) != item['sha256']:
            raise ValueError(f'video hash mismatch: {path}')
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise ValueError(f'cannot open {path}')
        fps = cap.get(cv2.CAP_PROP_FPS)
        declared = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not math.isfinite(fps) or fps <= 0 or declared <= 0:
            cap.release()
            raise ValueError(f'invalid stream metadata: {path}')
        stem = path.stem
        video = args.output / f'{stem}-detections.mp4'
        trace = args.output / f'{stem}-predictions.jsonl'
        chosen = set(int(round(n * (declared - 1) / 11)) for n in range(12))
        tiles = []
        states, latency = [], []
        frames_by_class, detection_counts = Counter(), Counter()
        frame_no = 0
        source_resolution = None
        encoder = None
        errors = tempfile.TemporaryFile()
        try:
            with trace.open('w', encoding='utf-8') as log:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        break
                    pts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
                    # First call initializes/fuses the model and is not a latency sample.
                    if frame_no == 0:
                        source_resolution = [frame.shape[1], frame.shape[0]]
                        detector.detect(frame)
                    started = time.perf_counter()
                    detections = detector.detect(frame)
                    elapsed = (time.perf_counter() - started) * 1000
                    latency.append(elapsed)
                    predictions = [d.to_dict() for d in detections]
                    state = output_state(predictions)
                    states.append(state)
                    detection_counts.update(d['class'] for d in predictions)
                    frames_by_class.update({d['class'] for d in predictions})
                    log.write(json.dumps({'frame': frame_no, 'playbackTimeS': round(frame_no / fps, 6),
                                          'sourcePtsS': round(pts, 6) if math.isfinite(pts) else None,
                                          'trafficOutput': state, 'inferenceMs': round(elapsed, 3),
                                          'predictions': predictions}, ensure_ascii=False,
                                         separators=(',', ':')) + '\n')
                    # Resize ONLY for display, after detection on the original 1280x720 image.
                    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    image = ImageOps.contain(image, (960, 540))
                    w, h = image.size
                    canvas = Image.new('RGB', (w + w % 2, h + h % 2 + 84), '#0b1422')
                    canvas.paste(image, (0, 40))
                    draw = ImageDraw.Draw(canvas)
                    draw.text((12, 10), 'КАНДИДАТ YOLO · ВСЕ КАДРЫ · БЕЗ СГЛАЖИВАНИЯ', font=font, fill='white')
                    for det in detections:
                        x, y, bw, bh = det.box
                        box = (int(x*w), int(y*h)+40, min(w-1, int((x+bw)*w)),
                               min(h+39, int((y+bh)*h)+40))
                        key = (det.color or 'unknown') if det.cls == 'traffic_light' else det.cls
                        color = colors[key]
                        draw.rectangle(box, outline=color, width=2)
                        tag = ('СВЕТОФОР / ' if det.cls == 'traffic_light' else '') + labels[key] + f' {det.confidence:.2f}'
                        text_width = int(draw.textlength(tag, font=small_font)) + 8
                        tx = max(0, min(box[0], w-text_width))
                        ty = max(40, box[1]-22)
                        draw.rectangle((tx, ty, tx+text_width, ty+21), fill='#0b1422', outline=color)
                        draw.text((tx+4, ty+1), tag, font=small_font, fill=color)
                    status = 'НЕТ РАМКИ СВЕТОФОРА' if state == 'no_detection' else ' / '.join(labels[s] for s in state.split('|'))
                    draw.text((12, h+45), f'{stem[-3:]}  |  {frame_no/fps:05.2f} с  |  кадр {frame_no+1}/{declared}', font=small_font, fill='#dae6f5')
                    draw.text((w//2, h+45), status, font=small_font, fill='#c6afff')
                    draw.text((12, h+64), f'YOLO + HSV   |   {profile["imgsz"]} px   |   порог {profile["confidence"]}   |   score ≠ точность', font=small_font, fill='#aebcd0')
                    if encoder is None:
                        rate = str(Fraction(fps).limit_denominator(1000000))
                        encoder = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error',
                            '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{canvas.width}x{canvas.height}',
                            '-r', rate, '-i', '-', '-an', '-c:v', 'libx264', '-threads', '1', '-preset', 'veryfast',
                            '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)],
                            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors)
                    encoder.stdin.write(canvas.tobytes())
                    if frame_no in chosen:
                        tile = canvas.resize((480, round(canvas.height * 480 / canvas.width)))
                        tiles.append(tile)
                    if frame_no == declared // 2:
                        canvas.save(args.output / f'{stem}-poster.jpg', quality=90)
                    frame_no += 1
                    if frame_no % 150 == 0:
                        print(f'{stem}: {frame_no}/{declared}', flush=True)
            if encoder is None:
                raise ValueError(f'no decoded frames: {path}')
            encoder.stdin.close()
            if encoder.wait(timeout=90) != 0:
                errors.seek(0)
                raise RuntimeError(errors.read().decode(errors='replace'))
        finally:
            cap.release()
            if encoder is not None and encoder.poll() is None:
                encoder.kill()
                encoder.wait()
            errors.close()
        if frame_no != declared or frame_no != item['frames']:
            raise ValueError(f'incomplete decode: {path}: {frame_no} != {declared}/{item["frames"]}')
        sheet = Image.new('RGB', (1440, 64 + 4*tiles[0].height), '#0b1422')
        draw = ImageDraw.Draw(sheet)
        draw.text((12, 10), f'{stem} — равномерные кадры от начала до конца', font=bold, fill='white')
        draw.text((12, 38), 'Рамки настоящие; «нет рамки» не всегда означает пропуск (объект может быть вне кадра).', font=font, fill='#b8c8dd')
        for index, tile in enumerate(tiles):
            sheet.paste(tile, (index % 3 * tile.width, 64 + index // 3 * tile.height))
        sheet.save(args.output / f'{stem}-storyboard.jpg', quality=92)
        # Count output-only runs without calling them ground-truth misses or correct colors.
        spans = state_runs(states, fps)
        summary = {'source': item['name'], 'sha256': item['sha256'], 'split': sorted(splits.get(item['name'], set())),
                   'frames': frame_no, 'declaredFrames': declared, 'allFramesProcessed': True,
                   'fps': fps, 'durationS': frame_no/fps, 'sourceResolution': source_resolution,
                   'outputResolution': [canvas.width, canvas.height], 'video': video.name,
                   'videoSha256': sha256(video), 'trace': trace.name, 'traceSha256': sha256(trace),
                   'poster': f'{stem}-poster.jpg', 'storyboard': f'{stem}-storyboard.jpg',
                   'framesWithClass': dict(frames_by_class), 'detectionCounts': dict(detection_counts),
                   'trafficOutputFrameCounts': dict(Counter(states)), 'trafficOutputRuns': spans,
                   'trafficOutputTransitions': max(0, len(spans)-1),
                   'inferenceMedianMs': round(statistics.median(latency), 3),
                   'inferenceP95Ms': round(sorted(latency)[int((len(latency)-1)*.95)], 3)}
        clips.append(summary)
        print(json.dumps({k: summary[k] for k in ('source','frames','durationS','split','framesWithClass','trafficOutputFrameCounts')}, ensure_ascii=False), flush=True)
    report = {'createdAt': datetime.now(timezone.utc).isoformat(), 'weights': str(args.weights.relative_to(ROOT)) if args.weights.is_relative_to(ROOT) else args.weights.name,
              'modelSha256': model_sha, 'sourceRevision': source['revision'], 'archiveSha256': source['archiveSha256'],
              'profile': {**{k:profile[k] for k in ('confidence','imgsz','preferRed','backend')},
                          'trafficMode': profile.get('trafficMode','legacy')},
              'versions': {'torch': torch.__version__, 'ultralytics': ultralytics.__version__, 'opencv': cv2.__version__},
              'frames': sum(c['frames'] for c in clips), 'durationS': sum(c['durationS'] for c in clips),
              'processing': 'Every decoded source frame; no tracker, smoothing, frame skipping or heuristic ensemble. Detection on original pixels. Output resized to 960x540 plus labels.',
              'timing': 'Output CFR at source average FPS, approximately original speed; source PTS in trace. Audio omitted.',
              'warning': 'Not independent new data: sparse frames from these same source videos belonged to train/val/test as marked. No frame-level ground truth. Output counts are NOT accuracy, recall, verified misses, or a safety guarantee.',
              'clips': clips}
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(f'FINISHED: {len(clips)} complete clips, {report["frames"]} frames, {report["durationS"]:.3f} seconds.', flush=True)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=ROOT/'models/vision/yolo-stop-candidate.pt')
    parser.add_argument('--profile', type=Path, default=ROOT/'models/vision/yolo-stop-candidate-evaluation.json')
    args = parser.parse_args()
    args.weights = args.weights.resolve()
    run(args)


if __name__ == '__main__':
    main()
