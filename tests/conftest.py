"""
Герметичность тестов: данные приложения (конфиг, история) уходят во временный
каталог, а не в ~/.local/share/zapret-control.
"""

import os
import tempfile

os.environ.setdefault("ZAPRET_APP_DIR", tempfile.mkdtemp(prefix="zapret-pytest-"))
