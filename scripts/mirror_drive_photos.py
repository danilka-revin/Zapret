#!/usr/bin/env python3
"""Make the companion ZIP for the Drive album: one horizontal mirror per original filename.

This is an album, not a training split. For paired YOLO images/labels use
prepare_drive_vision_dataset.py --mirror-train instead.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import hashlib
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from zipfile import ZipFile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.prepare_drive_vision_dataset import REVIEW, jpeg


def mirror_album(media: Path, output: Path, *, archive: Path | None = None,
                 manifest: Path = REVIEW / 'photos.md5') -> int:
    if output.exists():
        raise FileExistsError(f'Архив уже существует: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as temp, (ZipFile(archive) if archive else nullcontext()) as source:
        fallback = {Path(n).name: n for n in source.namelist() if n.lower().endswith('.jpg')} if source else {}
        seen = set()
        pairs = ['original,original_md5,mirror,mirror_sha256']
        with ZipFile(Path(temp) / 'album.zip', 'x') as dest:
            for row in manifest.read_text().splitlines():
                expected, name = row.split()
                if name != Path(name).name or name in seen or not name.lower().endswith('.jpg'):
                    raise ValueError(f'Некорректное или повторное имя: {name}')
                seen.add(name)
                direct = media / name
                if direct.is_file():
                    payload = direct.read_bytes()
                elif source and name in fallback:
                    payload = source.read(fallback[name])
                else:
                    raise FileNotFoundError(direct)
                if hashlib.md5(payload).hexdigest() != expected:
                    raise ValueError(f'Фото не совпадает с манифестом: {name}')
                image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f'Не удалось прочитать {name}')
                mirror = jpeg(cv2.flip(image, 1))
                path = f'hflip-{name}'
                dest.writestr(path, mirror)
                pairs.append(f'{name},{expected},{path},{hashlib.sha256(mirror).hexdigest()}')
            dest.writestr('pairs.csv', '\n'.join(pairs) + '\n')
            dest.writestr('README.txt', 'Только зеркальные фото; исходники уже есть в папке yolo. '
                          'Здесь есть и дубли. Для обучения используйте yolo-drive-x2.zip: '
                          'там отражены также рамки и из обучения исключены проверочные кадры.\n')
        (Path(temp) / 'album.zip').rename(output)
    return len(seen)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--media', type=Path, required=True, help='папка с исходными фото Drive')
    parser.add_argument('--archive', type=Path, help='закреплённый ZIP с совпадающими оригиналами, если они не в --media')
    parser.add_argument('--output', type=Path, required=True, help='новый ZIP-файл')
    args = parser.parse_args()
    print(f'Зеркальных фото: {mirror_album(args.media, args.output, archive=args.archive)} — {args.output}')
