#!/usr/bin/env python3
"""Choose a robot candidate's fixed bench profile on VAL, never on TEST.

A bounded, declared confidence/mode grid, with unchanged controls and the
already-reviewed traffic-free VAL shoe clip. Nothing is deployed by this tool.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.compare_vision import summarize

CONFIDENCES = (.15,.25,.35,.45)
MODES = ('legacy','body')


def eligible(summary: dict, negative: dict, *, frame_limit: int, colour_limit: int) -> bool:
    return (summary['falseGreen'] == 0 and
            negative['framesWithTrafficFalsePositive'] <= frame_limit and
            negative['specificColourFalsePositiveBoxes'] <= colour_limit)


def select_new(points: list[dict]) -> tuple[dict,bool]:
    """A failing candidate still has a lab profile, never an implied promotion."""
    new=[p for p in points if p['variant']=='robot']
    allowed=[p for p in new if p['eligible']]
    pool=allowed or [p for p in new if p['falseGreen']==0] or new
    if not pool:
        raise ValueError('no candidate points')
    return max(pool,key=lambda p:(p['score'],p['macroF1'],p['confidence'],p['trafficMode']=='body')),bool(allowed)


def video_negative(weights: Path, video: Path, review: dict, *, confidence: float, mode: str) -> dict:
    import cv2
    from robot_control.vision.detector import YoloDetector
    detector=YoloDetector(str(weights),confidence=confidence,imgsz=512,device='cpu',prefer_red=False,traffic_mode=mode)
    cap=cv2.VideoCapture(str(video));rows=[]
    try:
        while True:
            ok,frame=cap.read()
            if not ok:break
            traffic=[d.to_dict() for d in detector.detect(frame) if d.cls=='traffic_light']
            rows.append({'frame':len(rows),'trafficFalsePositives':traffic})
    finally:
        cap.release()
    if len(rows)!=review['frames']:
        raise ValueError('reviewed negative frame count differs')
    signals=Counter(d['signal'] or 'unknown' for row in rows for d in row['trafficFalsePositives'])
    return {'frames':len(rows),'framesWithTrafficFalsePositive':sum(bool(row['trafficFalsePositives']) for row in rows),
            'trafficFalsePositiveBoxes':sum(signals.values()),'specificColourFalsePositiveBoxes':sum(signals.get(c,0) for c in ('red','yellow','green')),
            'falseTrafficSignals':dict(signals),'trace':rows}


def select(*, weights: Path, data: Path, video: Path, negative_review: Path, folder: Path, profile_file: Path) -> dict:
    import cv2
    import torch
    from scripts.evaluate_vision import evaluate
    from scripts.train_vision import inspect_dataset
    torch.set_num_threads(2);cv2.setNumThreads(1)
    review=json.loads(negative_review.read_text())
    video_sha=hashlib.sha256(video.read_bytes()).hexdigest()
    if (review.get('sourceSha256')!=video_sha or review.get('originalSplit')!='val' or
        review.get('expectedTrafficLightsPerFrame')!=0 or review.get('reviewedTarget')!='traffic_light' or
        review.get('reviewedFrameRangeInclusive')!=[0,review['frames']-1]):
        raise ValueError('a complete traffic-free VAL video review is required')
    _,audit=inspect_dataset(data.resolve())
    samples=json.loads((data.parent/'samples.json').read_text())
    families={(s['split'],s['group']) for s in samples.values() if s['source']==video.name}
    if len(families)!=1 or next(iter(families))[0]!='val':
        raise ValueError('negative video must remain in its canonical VAL family')
    folder.mkdir(parents=True,exist_ok=True)
    # Controls at their historically declared .25 confidence; not a test sweep.
    specifications=[('baseline',ROOT/'models/vision/yolo-stop-candidate.pt',.25,'legacy'),
                    ('baseline',ROOT/'models/vision/yolo-stop-candidate.pt',.25,'body'),
                    ('previous',ROOT/'models/vision/yolo-stop-improved.pt',.25,'body')]
    specifications += [('robot',weights,confidence,mode) for confidence in CONFIDENCES for mode in MODES]
    points=[];frame_limit=colour_limit=None
    for variant,model,confidence,mode in specifications:
        name=f'{variant}-{mode}-{confidence:.2f}'
        report=evaluate(weights=model,data=data,output=folder/f'val-{name}.json',split='val',confidence=confidence,
                        imgsz=512,device='cpu',prefer_red=False,traffic_mode=mode)
        summary=summarize(report)
        negative=video_negative(model,video,review,confidence=confidence,mode=mode)
        (folder/f'negative-{name}.json').write_text(json.dumps({'source':video.name,'sourceSha256':video_sha,
                    'weightsSha256':report['sha256'],'confidence':confidence,'trafficMode':mode,**negative},ensure_ascii=False,indent=2)+'\n')
        compact={k:v for k,v in negative.items() if k!='trace'}
        if variant=='baseline' and mode=='legacy':
            frame_limit=negative['framesWithTrafficFalsePositive'];colour_limit=negative['specificColourFalsePositiveBoxes']
            # Protocol is fixed before training, so a changed reference is an error.
            if (frame_limit,colour_limit)!=(21,1):
                raise RuntimeError('baseline regression limits changed; do not silently redefine the protocol')
        point={'name':name,'variant':variant,'weights':str(model.relative_to(ROOT) if model.is_absolute() else model),
               'sha256':report['sha256'],'confidence':confidence,'trafficMode':mode,
               'macroF1':summary['macroF1'],'signalBalancedRecall':summary['signalBalancedRecall'],
               'falseGreen':summary['falseGreen'],'score':summary['validationObjective'],
               'summary':summary,'negativeClip':compact,
               'eligible':eligible(summary,compact,frame_limit=frame_limit,colour_limit=colour_limit)}
        points.append(point)
        print('POINT',name,'score',point['score'],'eligible',point['eligible'],flush=True)
    selected,passes=select_new(points)
    controls=[p for p in points if p['variant']!='robot' and p['eligible']]
    best_control=max(controls,key=lambda p:p['score']) if controls else None
    better=passes and (best_control is None or selected['score']>=best_control['score'])
    selection={'selectedAt':datetime.now(timezone.utc).isoformat(),'split':'val','datasetFingerprint':audit['fingerprint'],
               'confidenceGrid':list(CONFIDENCES),'modeGrid':list(MODES),'imgsz':512,'preferRed':False,
               'objective':'0.7*five-class macroF1 + 0.3*balanced signal recall INCLUDING missed bodies',
               'constraints':{'observedValFalseGreen':0,'negativeTrafficFPFramesAtMost':frame_limit,'negativeSpecificColourFPBoxesAtMost':colour_limit},
               'negativeSource':str(video),'negativeSha256':video_sha,'points':points,'selected':selected,
               'candidatePassesRegressionGates':passes,'bestEligibleControl':best_control,'candidatePreferredToControls':better,
               'protocol':'Bounded grid declared before training. No TEST inference/selection. Historical TEST is already reused, not a blind holdout.',
               'warning':'Bench selection only, not autonomous-ready; small noisy/agent-assisted labels and ONE correlated VAL negative clip, not independent ground truth or proof of safety.'}
    selection_file=folder/'validation-selection.json'
    selection_file.write_text(json.dumps(selection,ensure_ascii=False,indent=2)+'\n')
    profile={k:selected[k] for k in ('weights','sha256','confidence','trafficMode')}
    profile.update(imgsz=512,device='cpu',preferRed=False,backend='yolo+fusion',heuristicEnabled=False,
                   datasetFingerprint=audit['fingerprint'],selectedAt=selection['selectedAt'],selectedOn='VAL only',
                   candidatePassesRegressionGates=passes,candidatePreferredToControls=better,
                   selectionFile=str(selection_file.relative_to(ROOT)),selectionFileSha256=hashlib.sha256(selection_file.read_bytes()).hexdigest(),
                   testIsReusedBenchmark=True,deployment='bench only; no automatic production replacement',warning=selection['warning'])
    profile_file.parent.mkdir(parents=True,exist_ok=True)
    profile_file.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    return selection


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,default=ROOT/'models/vision/yolo-stop-robot.pt')
    parser.add_argument('--data',type=Path,default=ROOT/'datasets/yolo-stop/data.yaml')
    parser.add_argument('--negative-video',type=Path,default=ROOT/'.cache/vision/video-review/VID_20261117_223120_211.mp4')
    parser.add_argument('--negative-review',type=Path,default=ROOT/'docs/vision-improvement/negative-video-review.json')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'models/vision/training/robot')
    parser.add_argument('--profile',type=Path,default=ROOT/'models/vision/yolo-stop-robot-profile.json')
    args=parser.parse_args()
    result=select(weights=args.weights,data=args.data,video=args.negative_video,negative_review=args.negative_review,
                  folder=args.output_dir,profile_file=args.profile)
    print('SELECTED',result['selected']['name'],'passes',result['candidatePassesRegressionGates'],
          'preferred to controls',result['candidatePreferredToControls'])


if __name__=='__main__':
    main()
