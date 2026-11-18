"""Optional native camera dependencies. Importing this module does not load either wheel."""
from __future__ import annotations

import sys

REPAIR_COMMAND = ".venv/bin/python scripts/ensure_opencv.py"


def load_opencv():
    """Return (cv2, numpy, reason). Keep the native loader's actual exception."""
    try:
        import cv2
        import numpy
        # A leftover cv2 namespace (e.g. after removing a second OpenCV wheel)
        # can import successfully but cannot capture, encode or decode a frame.
        from cv2 import VideoCapture, imdecode, imencode  # noqa: F401
    except Exception as exc:  # ImportError, missing .so, NumPy ABI/initialization errors, etc.
        if isinstance(exc, ModuleNotFoundError) and exc.name in ("cv2", "numpy"):
            problem = f"Не установлен модуль {exc.name}"
        else:
            problem = "OpenCV/NumPy не загрузились"
        return None, None, f"{problem} ({type(exc).__name__}: {exc})"
    return cv2, numpy, ""


if __name__ == "__main__":  # fresh interpreter before/after pip; never open a device
    cv, np, error = load_opencv()
    if error:
        sys.exit(f"{sys.executable}: {error}")
    print(f"{sys.executable}: OpenCV {cv.__version__} ({cv.__file__}), NumPy {np.__version__} ({np.__file__})")
