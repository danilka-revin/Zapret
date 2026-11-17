#!/usr/bin/env python3
"""Обучить YOLO на знаках полигона и положить веса в ``models/vision/``.

    python scripts/train_vision.py --epochs 80 --imgsz 416
    python scripts/train_vision.py --smoke                 # быстрая проверка пайплайна
    python scripts/train_vision.py --weights yolov8n.pt   # дообучение с COCO-весов
    python scripts/train_vision.py --output /tmp/test.pt  # не трогать боевые веса

Если датасета ещё нет, скрипт сам вызывает ``scripts/vision_dataset.py``
(скачает 54 фото с GitHub и автоматически их разметит). Предобученные веса
не обязательны: без сети модель строится из ``yolov8n.yaml`` и учится с нуля —
на узком наборе из пяти классов этого хватает, но на GPU с COCO-весами
качество заметно выше.

Результат: ``models/vision/yolo-stop.pt`` + ``models/vision/metadata.json``.
``--smoke`` по умолчанию пишет отдельные ``yolo-stop-smoke.pt`` и
``metadata-smoke.json``, чтобы случайно не заменить обученную модель.
После перезапуска киоска сервис сам подхватит боевой файл и переключится с
встроенного детектора на ансамбль (``/api/vision/state`` →
``backend: yolo+heuristic``).
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_control.vision.classes import CLASS_NAMES  # noqa: E402

DATASET = ROOT / "datasets" / "yolo-stop"
MODEL_DIR = ROOT / "models" / "vision"
MODEL_FILE = MODEL_DIR / "yolo-stop.pt"
SMOKE_MODEL_FILE = MODEL_DIR / "yolo-stop-smoke.pt"
RUNS_DIR = ROOT / "runs" / "vision"


def ensure_dataset(no_download: bool) -> Path:
    data_yaml = DATASET / "data.yaml"
    if data_yaml.is_file() and (DATASET / "images" / "train").is_dir():
        return data_yaml
    from vision_dataset import build  # локальный импорт: скрипт рядом

    build(zip_path=None, out=DATASET, val_fraction=0.2,
          no_download=no_download, min_confidence=0.5)
    return data_yaml


def runtime_data_yaml(source: Path) -> Path:
    """Копия data.yaml с абсолютным ``path`` для Ultralytics.

    В репозитории лежит путь относительно корня проекта, но Ultralytics
    разрешает его относительно **текущего каталога**, а не файла data.yaml.
    Поэтому обучение получает временный файл в runs/vision/ с абсолютным
    путём: финальная валидация после последней эпохи читает тот же путь и
    не падает, если скрипт запустили не из корня проекта.
    """
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    lines = source.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if line.strip().startswith("path:"):
            lines[index] = f"path: {DATASET.resolve().as_posix()}"
            break
    target = RUNS_DIR / "data.yaml"
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target


def load_model(weights: str | None, prefer_pretrained: bool):
    """Взять готовые веса, локальный файл или архитектуру без весов."""
    from ultralytics import YOLO

    if weights:
        print(f"базовая модель: {weights}")
        return YOLO(weights), weights
    local = MODEL_DIR / "yolov8n.pt"
    if local.is_file():
        print(f"базовая модель: {local}")
        return YOLO(str(local)), str(local)
    if prefer_pretrained:
        try:
            model = YOLO("yolov8n.pt")  # скачает ~6 МБ, если есть интернет
            print("базовая модель: yolov8n.pt (COCO)")
            return model, "yolov8n.pt"
        except Exception as exc:  # noqa: BLE001
            print(f"предобученные веса недоступны ({exc}) — учу с нуля")
    model = YOLO("yolov8n.yaml")
    print("базовая модель: yolov8n.yaml (с нуля, без внешних скачиваний)")
    return model, "yolov8n.yaml"


def train(*, epochs: int, imgsz: int, batch: int, weights: str | None,
          prefer_pretrained: bool, no_download: bool, smoke: bool,
          export_onnx: bool, output: Path | None = None) -> dict[str, Any]:
    try:
        import ultralytics
    except ImportError:
        raise SystemExit("нужен ultralytics: pip install -r requirements-vision.txt")

    data_yaml = runtime_data_yaml(ensure_dataset(no_download))
    model, base = load_model(weights, prefer_pretrained)
    model_file = output or (SMOKE_MODEL_FILE if smoke else MODEL_FILE)
    if output is None and smoke:
        metadata_file = MODEL_DIR / "metadata-smoke.json"
    elif model_file == MODEL_FILE:
        metadata_file = MODEL_DIR / "metadata.json"
    else:
        metadata_file = model_file.with_suffix(".json")
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    model_file.parent.mkdir(parents=True, exist_ok=True)
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    started = time.time()
    epochs = 2 if smoke else epochs
    print(f"обучение: epochs={epochs} imgsz={imgsz} batch={batch} device=cpu", flush=True)
    results = model.train(
        data=str(data_yaml),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device="cpu",
        workers=1,
        amp=False,                 # на CPU смешанная точность не нужна
        project=str(RUNS_DIR),
        name="train",
        exist_ok=True,
        patience=max(10, epochs // 3),
        plots=not smoke,
        verbose=False,
        save=True,
    )
    best = Path(getattr(results, "save_dir", RUNS_DIR / "train")) / "weights" / "best.pt"
    if not best.is_file():
        raise SystemExit(f"best.pt не найден: {best}")
    shutil.copy2(best, model_file)

    metrics: dict[str, Any] = {}
    try:
        summary = results.results_dict
        metrics = {key: round(float(value), 4) for key, value in summary.items()
                   if isinstance(value, (int, float))}
    except Exception:  # noqa: BLE001
        pass
    if export_onnx:
        try:
            exported = model.export(format="onnx", imgsz=imgsz, simplify=False)
            onnx_path = Path(str(exported))
            if onnx_path.is_file():
                shutil.copy2(onnx_path, model_file.with_suffix(".onnx"))
        except Exception as exc:  # noqa: BLE001
            print(f"экспорт ONNX пропущен: {exc}")

    manifest = json.loads((DATASET / "dataset.json").read_text(encoding="utf-8")) \
        if (DATASET / "dataset.json").is_file() else {}
    metadata = {
        "model": model_file.name,
        "base": base,
        "ultralytics": ultralytics.__version__,
        "classes": list(CLASS_NAMES),
        "epochs": epochs,
        "imgsz": imgsz,
        "batch": batch,
        "durationS": round(time.time() - started, 1),
        "metrics": metrics,
        "dataset": {"images": manifest.get("images"), "counts": manifest.get("counts"),
                    "sha256": manifest.get("archiveSha256")},
        "trafficLight": "цвет определяется отдельно: robot_control/vision/traffic.py",
    }
    metadata_file.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(f"\nвеса: {model_file}")
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--weights", default=None,
                        help="путь к .pt или имя предобученной модели")
    parser.add_argument("--from-scratch", action="store_true",
                        help="не пытаться скачать yolov8n.pt")
    parser.add_argument("--no-download", action="store_true",
                        help="не скачивать датасет (уже лежит в datasets/)")
    parser.add_argument("--smoke", action="store_true", help="2 эпохи для проверки")
    parser.add_argument("--output", type=Path, default=None,
                        help="куда положить .pt (по умолчанию — боевые веса; "
                             "для --smoke — отдельный yolo-stop-smoke.pt)")
    parser.add_argument("--onnx", action="store_true", help="дополнительно экспортировать ONNX")
    args = parser.parse_args(argv)
    train(epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
          weights=args.weights, prefer_pretrained=not args.from_scratch,
          no_download=args.no_download, smoke=args.smoke,
          export_onnx=args.onnx, output=args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
