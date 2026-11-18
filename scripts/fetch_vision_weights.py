#!/usr/bin/env python3
"""Получить закреплённый YOLOv8n COCO через разрешённый codeload.github.com.

Альтернатива официальному `--weights yolov8n.pt`, если хост GitHub release
assets заблокирован. Зеркало хранит РАСПАКОВАННЫЙ PyTorch checkpoint;
собираем только архив тензоров, не исполняя код репозитория. До загрузки в
PyTorch проверяется SHA-256 всего содержимого checkpoint. Модель и исходные
веса подчиняются условиям лицензирования Ultralytics, не лицензии зеркала.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'AlaynaShaheen/object_detection_using_yolov8_tflite'
REVISION = '13f2ae34cfbb40d91c1b7b0c9c36cf96342288cc'
SOURCE_URL = f'https://codeload.github.com/{REPOSITORY}/zip/{REVISION}'
CONTENT_SHA256 = '4df48185493a8e5284698efc2caddd44053bb88e8ef827b4ea2e84bf502891c0'
DEFAULT_OUTPUT = ROOT / '.cache' / 'vision' / 'pretrained' / 'coco-repacked.pt'


def content_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with zipfile.ZipFile(path) as archive:
        for name in sorted(archive.namelist()):
            relative = name.split('/', 1)[1]
            digest.update(relative.encode() + b'\0' + hashlib.sha256(archive.read(name)).digest())
    return digest.hexdigest()


def fetch(output: Path = DEFAULT_OUTPUT, repo_zip: Path | None = None) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.is_file():
        if repo_zip:
            payload = repo_zip.read_bytes()
        else:
            with urllib.request.urlopen(SOURCE_URL, timeout=90) as reply:
                payload = reply.read()
        temporary = output.with_suffix('.pt.part')
        try:
            with zipfile.ZipFile(io.BytesIO(payload)) as source, zipfile.ZipFile(temporary, 'w') as dest:
                for name in sorted(source.namelist()):
                    if '/yolov8n.pt/' not in name or name.endswith('/'):
                        continue
                    relative = name.split('/yolov8n.pt/', 1)[1]
                    if any(part in ('.', '..') for part in Path(relative).parts):
                        raise ValueError('небезопасное имя в архиве')
                    info = zipfile.ZipInfo('checkpoint/' + relative, (1980, 1, 1, 0, 0, 0))
                    dest.writestr(info, source.read(name))
            if content_digest(temporary) != CONTENT_SHA256:
                raise ValueError('SHA-256 предобученных весов не совпал; файл не будет использован')
            temporary.replace(output)
        finally:
            temporary.unlink(missing_ok=True)
    if content_digest(output) != CONTENT_SHA256:
        raise ValueError(f'неожиданное содержимое существующего checkpoint: {output}')
    provenance = {
        'source': f'https://github.com/{REPOSITORY}/tree/{REVISION}/yolov8n.pt',
        'revision': REVISION, 'contentSha256': CONTENT_SHA256,
        'checkpointVersion': '8.0.0.dev0', 'checkpointDate': '2022-12-30T00:12:08.084698',
        'originalTrainingData': 'coco.yaml',
        'licenseNote': 'Ultralytics model licensing applies; see https://github.com/ultralytics/ultralytics/blob/main/LICENSE',
    }
    output.with_suffix('.json').write_text(json.dumps(provenance, indent=2) + '\n', encoding='utf-8')
    print(output)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--repo-zip', type=Path, help='локальная копия закреплённого репозитория')
    args = parser.parse_args()
    fetch(args.output, args.repo_zip)


if __name__ == '__main__':
    main()
