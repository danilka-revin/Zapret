"""
Qt6-интерфейс Zapret Control (PySide6).

Модуль заменяет прежний tkinter-интерфейс: те же возможности плюс стеклянная
тема, большая кнопка включения, автопилот и полноценная кастомизация.
"""

from .app import ZapretWindow, run
from .controller import Controller
from .theme import ThemeManager, UISettings

__all__ = ["run", "ZapretWindow", "Controller", "ThemeManager", "UISettings"]
