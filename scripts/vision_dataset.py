#!/usr/bin/env python3
"""Готовит YOLO-датасет знаков полигона из архива ``p25543770-lang/yolo-stop``.

В исходном архиве 54 JPEG и **ни одного** ``.txt`` с разметкой. Поэтому скрипт:

1. скачивает архив с GitHub (или берёт локальный ``--zip``);
2. раскладывает кадры по ``images/train`` и ``images/val`` (детерминированно);
3. размечает их встроенным детектором :class:`HeuristicDetector` — теми же
   правилами, что работают на роботе без PyTorch;
4. пишет ``labels/*.txt`` в формате YOLO, ``data.yaml``, ``dataset.json`` и
   ``annotations.json`` (с цветами светофора).

    python scripts/vision_dataset.py                    # скачать и разметить
    python scripts/vision_dataset.py --zip ~/Downloads/yolo-...zip
    python scripts/vision_dataset.py --no-download     # только переразметить

Сами JPEG в Git не кладутся (см. .gitignore): они скачиваются заново, а в
репозитории хранятся компактные метки и параметры сборки.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import shutil
import sys
import urllib.request
import zipfile
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control.vision.classes import CLASS_NAMES  # noqa: E402
from robot_control.vision.detector import (  # noqa: E402
    HeuristicDetector, cv2_available, yolo_label_lines,
)

SOURCE_URL = "https://codeload.github.com/p25543770-lang/yolo-stop/zip/refs/heads/main"
ARCHIVE_NAME = "yolo-20261117T081703Z-1-001.zip"
DEFAULT_OUT = ROOT / "datasets" / "yolo-stop"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _read_archive(path: Path) -> dict[str, bytes]:
    """Вернуть {имя jpg: байты} из zip-архива репозитория или набора JPEG."""
    with zipfile.ZipFile(path) as archive:
        members = [m for m in archive.infolist()
                   if not m.is_dir() and m.filename.lower().endswith((".jpg", ".jpeg", ".png"))]
        # В архиве репозитория лежит вложенный zip с кадрами.
        nested = [m for m in archive.infolist() if m.filename.lower().endswith(".zip")]
        files: dict[str, bytes] = {}
        if nested and not members:
            for item in nested:
                inner = zipfile.ZipFile(io.BytesIO(archive.read(item)))
                for member in inner.infolist():
                    if member.filename.lower().endswith((".jpg", ".jpeg", ".png")):
                        files[Path(member.filename).name] = inner.read(member)
        else:
            for member in members:
                files[Path(member.filename).name] = archive.read(member)
        return files


def download_archive(destination: Path) -> Path:
    """Скачать репозиторий с данными (только разрешённый хост codeload.github.com)."""
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / "source-repo.zip"
    if not target.is_file():
        print(f"скачиваю {SOURCE_URL} …", flush=True)
        with urllib.request.urlopen(SOURCE_URL, timeout=60) as reply:
            target.write_bytes(reply.read())
    archive = destination / ARCHIVE_NAME
    if not archive.is_file():
        with zipfile.ZipFile(target) as repo:
            for member in repo.infolist():
                if member.filename.lower().endswith(".zip"):
                    archive.write_bytes(repo.read(member))
                    break
            else:
                raise SystemExit("в репозитории не найден zip с кадрами")
    return archive


def split_names(names: list[str], val_fraction: float) -> tuple[list[str], list[str]]:
    """Стабильное разбиение по хэшу имени: одинаковое на любой машине."""
    train, val = [], []
    for name in sorted(names):
        bucket = int(hashlib.sha256(name.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
        (val if bucket < val_fraction else train).append(name)
    if val and not train:
        train, val = val, []
    return train, val


def build(*, zip_path: Path | None, out: Path, val_fraction: float,
          no_download: bool, min_confidence: float) -> dict[str, Any]:
    if not cv2_available():
        raise SystemExit("нужен OpenCV: pip install -r requirements-vision.txt")
    import cv2
    import numpy as np

    out.mkdir(parents=True, exist_ok=True)
    raw_dir = out / "raw"
    raw_dir.mkdir(exist_ok=True)
    archive = zip_path
    if archive is None:
        cached = out / ARCHIVE_NAME
        if cached.is_file():
            archive = cached
        elif not no_download:
            archive = download_archive(out)
    if archive is None or not archive.is_file():
        raise SystemExit("архив не найден: укажите --zip или запустите без --no-download")

    files = _read_archive(archive)
    if not files:
        raise SystemExit("в архиве нет JPEG")
    for name, data in files.items():
        (raw_dir / name).write_bytes(data)
    print(f"кадров: {len(files)}", flush=True)

    detector = HeuristicDetector(min_confidence=min_confidence)
    annotations: dict[str, list[dict[str, Any]]] = {}
    labels: dict[str, str] = {}
    counts: Counter[str] = Counter()
    signals: Counter[str] = Counter()
    for name, data in sorted(files.items()):
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        detections = detector.detect(frame)
        labels[name] = "\n".join(yolo_label_lines(detections))
        annotations[name] = [d.to_dict() for d in detections]
        counts.update(d.cls for d in detections)
        signals.update(d.color or "unknown" for d in detections if d.cls == "traffic_light")

    for part in ("train", "val"):
        (out / "images" / part).mkdir(parents=True, exist_ok=True)
        (out / "labels" / part).mkdir(parents=True, exist_ok=True)
        for old in (out / "labels" / part).glob("*.txt"):
            old.unlink()
        for old in (out / "images" / part).glob("*"):
            if old.is_file():
                old.unlink()

    train, val = split_names(list(files), val_fraction)
    for part, names in (("train", train), ("val", val)):
        for name in names:
            stem = Path(name).stem
            shutil.copy2(raw_dir / name, out / "images" / part / name)
            (out / "labels" / part / f"{stem}.txt").write_text(
                labels.get(name, "") + "\n", encoding="utf-8")

    try:
        relative_out = out.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        relative_out = out.resolve().as_posix()
    data_yaml = (
        "# Сгенерировано scripts/vision_dataset.py — не править вручную.\n"
        "# Ultralytics разрешает path относительно КАТАЛОГА ЗАПУСКА (корень проекта).\n"
        "# scripts/train_vision.py дополнительно создаёт runs/vision/data.yaml с\n"
        "# абсолютным путём, чтобы обучение работало из любого каталога.\n"
        f"path: {relative_out}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n" + "".join(f"  {i}: {name}\n" for i, name in enumerate(CLASS_NAMES))
    )
    (out / "data.yaml").write_text(data_yaml, encoding="utf-8")
    manifest = {
        "source": "https://github.com/p25543770-lang/yolo-stop",
        "archive": ARCHIVE_NAME,
        "archiveSha256": _sha256(archive.read_bytes()),
        "images": len(files), "train": len(train), "val": len(val),
        "valFraction": val_fraction,
        "classes": list(CLASS_NAMES),
        "counts": dict(counts),
        "trafficLightSignals": dict(signals),
        "autoLabel": "robot_control.vision.detector.HeuristicDetector",
        "minConfidence": min_confidence,
    }
    (out / "dataset.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    (out / "annotations.json").write_text(
        json.dumps(annotations, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(f"\nготово: {out / 'data.yaml'}")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zip", type=Path, default=None, help="локальный zip с кадрами")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--no-download", action="store_true",
                        help="не обращаться к сети, использовать уже скачанный архив")
    parser.add_argument("--min-confidence", type=float, default=0.5)
    args = parser.parse_args(argv)
    if not 0.0 <= args.val_fraction < 0.5:
        parser.error("--val-fraction должен быть от 0 до 0.5")
    build(zip_path=args.zip, out=args.out, val_fraction=args.val_fraction,
          no_download=args.no_download, min_confidence=args.min_confidence)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
