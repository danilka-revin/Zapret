"""
Графический интерфейс Zapret Control (стеклянная тёмная тема / glassmorphism).

Реализован на tkinter без внешних зависимостей.
"""

import datetime
import shutil
import subprocess
import sys
import threading
from pathlib import Path

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

from . import APP_NAME, APP_VERSION, app_dir
from . import background
from . import config as config_mod
from . import core, integration


# ----------------------------------------------------------------------------
# Тема
# ----------------------------------------------------------------------------

class Theme:
    BG = "#0a0c16"
    CARD = "#14172b"
    INSET = "#10132a"
    BORDER = "#2b3050"
    TEXT = "#e9ecf8"
    TEXT_DIM = "#9298bd"
    ACCENT = "#5b6cff"
    ACCENT2 = "#33e0c1"
    GOOD = "#3ce0a5"
    BAD = "#ff5d73"
    WARN = "#ffb454"


# ----------------------------------------------------------------------------
# Геометрия
# ----------------------------------------------------------------------------

W, H = 1000, 700
M = 26
COL_GAP = 18
HEADER_Y = 96
CONTENT_BOTTOM = H - M

# Колонки
COL1_X, COL1_W = M, 292
COL2_X = COL1_X + COL1_W + COL_GAP
COL2_W = 292
COL3_X = COL2_X + COL2_W + COL_GAP
COL3_W = W - M - COL3_X

# Карточки (для отрисовки «стекла»)
STATUS_CARD = (COL1_X, HEADER_Y, COL1_X + COL1_W, HEADER_Y + 252)
SERVICES_CARD = (COL1_X, HEADER_Y + 252 + COL_GAP, COL1_X + COL1_W, CONTENT_BOTTOM)
SETTINGS_CARD = (COL2_X, HEADER_Y, COL2_X + COL2_W, HEADER_Y + 252)
ACTIONS_CARD = (COL2_X, HEADER_Y + 252 + COL_GAP, COL2_X + COL2_W, CONTENT_BOTTOM)
LOG_CARD = (COL3_X, HEADER_Y, COL3_X + COL3_W, CONTENT_BOTTOM)

ALL_CARDS = [STATUS_CARD, SERVICES_CARD, SETTINGS_CARD, ACTIONS_CARD, LOG_CARD]


# ----------------------------------------------------------------------------
# Вспомогательные примитивы
# ----------------------------------------------------------------------------

def _shade(hex_color: str, f: float) -> str:
    hex_color = hex_color.lstrip("#")
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    r, g, b = (min(255, int(c * f)) for c in (r, g, b))
    return f"#{r:02x}{g:02x}{b:02x}"


def rounded_rect(canvas, x1, y1, x2, y2, r, **kw):
    if r <= 0:
        return canvas.create_rectangle(x1, y1, x2, y2, **kw)
    pts = [
        x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
        x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
        x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
    ]
    return canvas.create_polygon(pts, smooth=True, **kw)


class GlassButton:
    """Стеклянная кнопка-«пилюля» на canvas."""

    KINDS = {
        "primary": ("#5b6cff", "#7483ff", "#ffffff"),
        "danger": ("#c94f6d", "#db5f7d", "#ffffff"),
        "secondary": ("#1d2140", "#272c52", "#dfe3f5"),
        "ghost": ("#191d38", "#21254a", "#c4c9e6"),
    }

    def __init__(self, canvas, x, y, w, h, text, command, kind="secondary",
                 font=None, enabled=True):
        self.c = canvas
        self.x, self.y, self.w, self.h = x, y, w, h
        self.text = text
        self.command = command
        self.kind = kind
        self.font = font
        self.enabled = enabled
        self.tag = f"btn_{id(self)}"
        self._hover = False
        self._pressed = False
        self._render()

    def _fill(self):
        base, hover, _fg = self.KINDS[self.kind]
        if not self.enabled:
            return _shade(base, 0.55)
        if self._pressed:
            return _shade(base, 0.8)
        if self._hover:
            return hover
        return base

    def _fg(self):
        return self.KINDS[self.kind][2]

    def _render(self):
        c = self.c
        c.delete(self.tag)
        r = self.h / 2
        fill = self._fill()
        rounded_rect(c, self.x, self.y, self.x + self.w, self.y + self.h, r,
                     fill=fill, outline="#3d4370", width=1, tags=self.tag)
        c.create_line(self.x + 10, self.y + 1.5, self.x + self.w - 10, self.y + 1.5,
                      fill="#8f97d6", tags=self.tag)
        c.create_text(self.x + self.w / 2, self.y + self.h / 2, text=self.text,
                      fill=self._fg(), font=self.font, tags=self.tag)
        c.tag_bind(self.tag, "<Enter>", self._on_enter)
        c.tag_bind(self.tag, "<Leave>", self._on_leave)
        c.tag_bind(self.tag, "<Button-1>", self._on_press)
        c.tag_bind(self.tag, "<ButtonRelease-1>", self._on_release)

    def _on_enter(self, _e=None):
        self._hover = True
        self._render()

    def _on_leave(self, _e=None):
        self._hover = False
        self._pressed = False
        self._render()

    def _on_press(self, _e=None):
        self._pressed = True
        self._render()

    def _on_release(self, _e=None):
        was = self._pressed
        self._pressed = False
        self._render()
        if was and self.enabled and self.command:
            self.command()

    def set_text(self, text):
        self.text = text
        self._render()

    def set_kind(self, kind):
        self.kind = kind
        self._render()

    def set_enabled(self, enabled):
        self.enabled = enabled
        self._render()


class Toggle:
    """Переключатель-«пилюля»."""

    TW, TH, K = 46, 26, 20

    def __init__(self, canvas, x, y, on=False, command=None):
        self.c = canvas
        self.x, self.y = x, y
        self.on = on
        self.command = command
        self.tag = f"tgl_{id(self)}"
        self._render()

    def _render(self):
        c = self.c
        c.delete(self.tag)
        bg = Theme.ACCENT if self.on else "#2a2f52"
        rounded_rect(c, self.x, self.y, self.x + self.TW, self.y + self.TH,
                     self.TH / 2, fill=bg, outline="#3d4370", tags=self.tag)
        kx = self.x + self.TW - self.K / 2 - 3 if self.on else self.x + self.K / 2 + 3
        ky = self.y + self.TH / 2
        c.create_oval(kx - self.K / 2, ky - self.K / 2, kx + self.K / 2, ky + self.K / 2,
                      fill="#ffffff", outline="", tags=self.tag)
        c.tag_bind(self.tag, "<Button-1>", self._click)

    def _click(self, _e=None):
        self.on = not self.on
        self._render()
        if self.command:
            self.command(self.on)

    def set(self, on):
        if on != self.on:
            self.on = on
            self._render()


# ----------------------------------------------------------------------------
# Приложение
# ----------------------------------------------------------------------------

class ZapretApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title(f"{APP_NAME} — обход DPI")
        root.resizable(False, False)
        root.configure(bg=Theme.BG)

        self.family = self._pick_family()
        self.f_title = tkfont.Font(root=root, family=self.family, size=22, weight="bold")
        self.f_sub = tkfont.Font(root=root, family=self.family, size=10)
        self.f_card = tkfont.Font(root=root, family=self.family, size=11, weight="bold")
        self.f_label = tkfont.Font(root=root, family=self.family, size=10)
        self.f_small = tkfont.Font(root=root, family=self.family, size=9)
        self.f_mono = tkfont.Font(root=root, family="Courier", size=9)
        self.f_btn = tkfont.Font(root=root, family=self.family, size=10, weight="bold")

        self.cfg = config_mod.load()
        self.status = {}
        self.power_btn = None

        self._build_canvas()
        self._build_header()
        self._build_status_card()
        self._build_services_card()
        self._build_settings_card()
        self._build_actions_card()
        self._build_log_card()

        self._center_on_screen()
        self.log("Добро пожаловать в Zapret Control")
        self.log("Сначала скачайте зависимости, затем нажмите «Запустить».")
        self.refresh_status()

    # -- фон ----------------------------------------------------------------

    def _pick_family(self):
        fams = set(tkfont.families(self.root))
        for f in ("Segoe UI", "Inter", "Ubuntu", "Cantarell", "Noto Sans", "DejaVu Sans"):
            if f in fams:
                return f
        return "TkDefaultFont"

    def _build_canvas(self):
        self.canvas = tk.Canvas(self.root, width=W, height=H, highlightthickness=0,
                                bd=0, bg=Theme.BG)
        self.canvas.place(x=0, y=0, width=W, height=H)
        try:
            cache = app_dir() / "cache"
            cache.mkdir(parents=True, exist_ok=True)
            png_path = cache / f"bg-{W}x{H}.png"
            if not png_path.exists():
                png_path.write_bytes(background.render_ui_png(W, H, ALL_CARDS))
            self.bg_img = tk.PhotoImage(file=str(png_path))
        except Exception:  # noqa: BLE001
            self.bg_img = tk.PhotoImage(width=W, height=H)
            self.bg_img.put(Theme.BG, to=(0, 0, W, H))
        self.canvas.create_image(0, 0, image=self.bg_img, anchor="nw")

    # -- шапка --------------------------------------------------------------

    def _build_header(self):
        self.canvas.create_text(M, 26, anchor="nw", text=APP_NAME,
                                fill=Theme.TEXT, font=self.f_title)
        self.canvas.create_text(M, 56, anchor="nw",
                                text="Обход замедления YouTube · Discord · Telegram",
                                fill=Theme.TEXT_DIM, font=self.f_sub)
        self.canvas.create_text(M, 72, anchor="nw",
                                text=f"zapret · v{APP_VERSION}",
                                fill="#5d6287", font=self.f_small)

        # статусная «пилюля» справа
        pw, ph = 132, 34
        px = W - M - pw
        py = 20
        rounded_rect(self.canvas, px, py, px + pw, py + ph, ph / 2,
                     fill="#14172b", outline=Theme.BORDER)
        self.pill_dot = self.canvas.create_oval(px + 12, py + 11, px + 26, py + 25,
                                                fill=Theme.BAD, outline="")
        self.pill_text = self.canvas.create_text(px + 34, py + ph / 2, anchor="w",
                                                 text="Остановлен", fill=Theme.TEXT,
                                                 font=self.f_label)

    def _center_on_screen(self):
        self.root.update_idletasks()
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        x = max(0, (sw - W) // 2)
        y = max(0, (sh - H) // 2)
        self.root.geometry(f"{W}x{H}+{x}+{y}")

    # -- карточки -----------------------------------------------------------

    def _card_title(self, x1, y1, title, extra=None):
        self.canvas.create_text(x1 + 16, y1 + 20, anchor="w", text=title,
                                fill=Theme.TEXT, font=self.f_card)
        if extra:
            self.canvas.create_text(x1 + 276, y1 + 20, anchor="e", text=extra,
                                    fill=Theme.TEXT_DIM, font=self.f_small)

    def _set_dot(self, dot_id, color):
        self.canvas.itemconfig(dot_id, fill=color)

    # -- карточка «Статус» --------------------------------------------------

    def _build_status_card(self):
        x1, y1, x2, y2 = STATUS_CARD
        self._card_title(x1, y1, "Статус")
        self.dot_ids = {}
        rows = [
            ("run", "Запущено (nfqws)"),
            ("fw", "Файрвол"),
            ("svc", "Системная служба"),
            ("deps", "Зависимости"),
            ("sudo", "Права sudo"),
        ]
        y = y1 + 52
        for key, label in rows:
            self.canvas.create_text(x1 + 16, y, anchor="w", text=label,
                                    fill=Theme.TEXT_DIM, font=self.f_label)
            dot = self.canvas.create_oval(x2 - 24, y - 6, x2 - 12, y + 6,
                                          fill=Theme.WARN, outline="")
            self.dot_ids[key] = dot
            y += 34
        self.backend_text = self.canvas.create_text(
            x1 + 16, y + 6, anchor="w", text="Бэкенд: —",
            fill="#5d6287", font=self.f_small)

    # -- карточка «Сервисы» -------------------------------------------------

    def _build_services_card(self):
        x1, y1, x2, y2 = SERVICES_CARD
        self._card_title(x1, y1, "Сервисы")
        self.toggles = {}

        def row(label, sub, on, cb, y):
            self.canvas.create_text(x1 + 16, y, anchor="w", text=label,
                                    fill=Theme.TEXT, font=self.f_label)
            self.canvas.create_text(x1 + 16, y + 16, anchor="w", text=sub,
                                    fill="#5d6287", font=self.f_small)
            tg = Toggle(self.canvas, x2 - 16 - Toggle.TW, y - 6, on=on, command=cb)
            return tg

        self.toggles["telegram"] = row(
            "Telegram", "обход MTProto и web.telegram.org",
            self.cfg.get("telegram", True), self._on_telegram, y1 + 52)
        self.toggles["gft"] = row(
            "GameFilter TCP", "порты игр 1024–65535",
            self.cfg.get("gamefilter_tcp", False), self._on_gft, y1 + 116)
        self.toggles["gfu"] = row(
            "GameFilter UDP", "порты игр 1024–65535",
            self.cfg.get("gamefilter_udp", False), self._on_gfu, y1 + 180)

        self.canvas.create_text(x1 + 16, y2 - 28, anchor="w",
                                text="YouTube и Discord входят в стратегию general.bat.",
                                fill="#5d6287", font=self.f_small)

    # -- карточка «Настройки» ----------------------------------------------

    def _build_settings_card(self):
        x1, y1, x2, y2 = SETTINGS_CARD
        self._card_title(x1, y1, "Настройки")
        self._style = ttk.Style(self.root)
        try:
            self._style.theme_use("clam")
        except tk.TclError:
            pass
        self._style.configure(
            "Dark.TCombobox",
            fieldbackground="#1a1e3a", background="#1a1e3a",
            foreground=Theme.TEXT, arrowcolor=Theme.TEXT_DIM,
            bordercolor=Theme.BORDER, lightcolor="#1a1e3a", darkcolor="#1a1e3a",
            padding=5, relief="flat",
        )
        self._style.map(
            "Dark.TCombobox",
            fieldbackground=[("readonly", "#1a1e3a")],
            foreground=[("readonly", Theme.TEXT)],
            selectbackground=[("readonly", "#1a1e3a")],
            selectforeground=[("readonly", Theme.TEXT)],
        )
        self.root.option_add("*TCombobox*Listbox.background", "#171a30")
        self.root.option_add("*TCombobox*Listbox.foreground", Theme.TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", Theme.ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

        cw = COL2_W - 32

        self.canvas.create_text(x1 + 16, y1 + 52, anchor="w", text="Стратегия",
                                fill=Theme.TEXT_DIM, font=self.f_small)
        self.strategy_combo = self._combo(x1 + 16, y1 + 70, cw,
                                          self._strategy_values(), self._on_strategy)
        self.canvas.create_text(x1 + 16, y1 + 116, anchor="w", text="Сетевой интерфейс",
                                fill=Theme.TEXT_DIM, font=self.f_small)
        self.iface_combo = self._combo(x1 + 16, y1 + 134, cw,
                                       self._iface_values(), self._on_iface)
        self.canvas.create_text(x1 + 16, y1 + 180, anchor="w", text="Бэкенд файрвола",
                                fill=Theme.TEXT_DIM, font=self.f_small)
        self.backend_combo = self._combo(x1 + 16, y1 + 198, cw,
                                         ["auto", "nftables", "iptables"], self._on_backend)

        # восстанавливаем сохранённые значения
        if self.cfg.get("strategy") in self.strategy_combo["values"]:
            self.strategy_combo.set(self.cfg["strategy"])
        if self.cfg.get("interface", "any") in self.iface_combo["values"]:
            self.iface_combo.set(self.cfg.get("interface", "any"))
        if self.cfg.get("firewall_backend", "auto") in self.backend_combo["values"]:
            self.backend_combo.set(self.cfg.get("firewall_backend", "auto"))

    def _combo(self, x, y, w, values, on_change):
        var = tk.StringVar(value=values[0] if values else "")
        cb = ttk.Combobox(self.root, textvariable=var, values=values,
                          state="readonly", style="Dark.TCombobox", width=2)
        cb.place(x=x, y=y, width=w, height=28)
        cb.bind("<<ComboboxSelected>>", lambda _e: on_change(var.get()))
        return cb

    def _strategy_values(self):
        vals = core.list_strategies()
        return vals or ["general.bat"]

    def _iface_values(self):
        vals = ["any"]
        net = Path("/sys/class/net")
        if net.exists():
            vals += sorted(p.name for p in net.iterdir() if p.is_dir())
        return vals

    # -- карточка «Управление» ---------------------------------------------

    def _build_actions_card(self):
        x1, y1, x2, y2 = ACTIONS_CARD
        self._card_title(x1, y1, "Управление")
        bw = COL2_W - 32
        bx = x1 + 16

        self.power_btn = GlassButton(
            self.canvas, bx, y1 + 46, bw, 60, "Запустить",
            self._toggle_power, kind="primary", font=self.f_btn)

        self.deps_btn = GlassButton(
            self.canvas, bx, y1 + 118, bw, 38, "Скачать зависимости",
            self._download_deps, kind="secondary", font=self.f_label)
        self.shortcut_btn = GlassButton(
            self.canvas, bx, y1 + 163, bw, 38, "Создать ярлык приложения",
            self._toggle_shortcut, kind="secondary", font=self.f_label)
        self.service_btn = GlassButton(
            self.canvas, bx, y1 + 208, bw, 38, "Установить автозапуск",
            self._toggle_service, kind="secondary", font=self.f_label)
        self.update_btn = GlassButton(
            self.canvas, bx, y1 + 253, bw, 38, "Обновить",
            self._update_app, kind="secondary", font=self.f_label)

        l1 = self.canvas.create_text(bx, y2 - 26, anchor="w", text="Права sudo",
                                     fill=Theme.ACCENT2, font=self.f_small)
        l2 = self.canvas.create_text(x2 - 16, y2 - 26, anchor="e", text="Справка",
                                     fill=Theme.ACCENT2, font=self.f_small)
        self.canvas.tag_bind(l1, "<Button-1>", lambda _e: self._setup_permissions())
        self.canvas.tag_bind(l2, "<Button-1>", lambda _e: self._show_help())

    # -- карточка «Журнал» --------------------------------------------------

    def _build_log_card(self):
        x1, y1, x2, y2 = LOG_CARD
        self._card_title(x1, y1, "Журнал")
        clear = self.canvas.create_text(x2 - 16, y1 + 20, anchor="e", text="Очистить",
                                        fill=Theme.ACCENT2, font=self.f_small)
        self.canvas.tag_bind(clear, "<Button-1>", lambda _e: self._clear_log())

        tx, ty = x1 + 14, y1 + 44
        tw = (x2 - x1) - 28
        th = (y2 - y1) - 58
        self.log_text = tk.Text(
            self.root, bg=Theme.INSET, fg="#aeb4d6", insertbackground=Theme.TEXT,
            relief="flat", bd=0, highlightthickness=0, wrap="word", font=self.f_mono,
            state="disabled",
        )
        self.log_text.place(x=tx, y=ty, width=tw, height=th)

    # -- лог ----------------------------------------------------------------

    def log(self, msg: str):
        self.root.after(0, self._append_log, msg)

    def _append_log(self, msg: str):
        try:
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            self.log_text.configure(state="normal")
            self.log_text.insert("end", f"[{ts}] {msg}\n")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        except tk.TclError:
            pass

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # -- фоновые операции ---------------------------------------------------

    def _run_async(self, fn, done=None):
        def worker():
            try:
                fn()
            except Exception as exc:  # noqa: BLE001
                self.log(f"Ошибка: {exc}")
                self.root.after(0, self.refresh_status)
            else:
                if done:
                    self.root.after(0, done)
                self.root.after(0, self.refresh_status)
        threading.Thread(target=worker, daemon=True).start()

    # -- статус -------------------------------------------------------------

    def refresh_status(self):
        self._run_async(self._read_status, self._apply_status)

    def _read_status(self):
        try:
            self.status = core.status()
        except Exception as exc:  # noqa: BLE001
            self.status = {"error": str(exc)}

    def _apply_status(self):
        s = self.status
        if s.get("running"):
            self._set_dot(self.dot_ids["run"], Theme.GOOD)
            self.canvas.itemconfig(self.pill_dot, fill=Theme.GOOD)
            self.canvas.itemconfig(self.pill_text, text="Работает")
            self.power_btn.set_text("Остановить")
            self.power_btn.set_kind("danger")
        else:
            self._set_dot(self.dot_ids["run"], Theme.BAD)
            self.canvas.itemconfig(self.pill_dot, fill=Theme.BAD)
            self.canvas.itemconfig(self.pill_text, text="Остановлен")
            self.power_btn.set_text("Запустить")
            self.power_btn.set_kind("primary")

        self._set_dot(self.dot_ids["fw"], Theme.GOOD if s.get("firewall") else Theme.BAD)
        self._set_dot(self.dot_ids["deps"], Theme.GOOD if s.get("deps_ready") else Theme.WARN)

        svc = s.get("service_installed")
        self._set_dot(self.dot_ids["svc"], Theme.GOOD if s.get("service_active") else
                      (Theme.WARN if svc else Theme.BAD))
        if svc:
            self.service_btn.set_text("Удалить автозапуск")
        else:
            self.service_btn.set_text("Установить автозапуск")

        if s.get("shortcut_installed"):
            self.shortcut_btn.set_text("Удалить ярлык")
        else:
            self.shortcut_btn.set_text("Создать ярлык приложения")

        self._set_dot(self.dot_ids["sudo"], Theme.GOOD if s.get("sudo_ok") else Theme.WARN)
        backend = s.get("backend")
        self.canvas.itemconfig(self.backend_text, text=f"Бэкенд: {backend or '—'}")
        if not s.get("deps_ready"):
            self.log("Зависимости не скачаны — нажмите «Скачать зависимости».")

    # -- действия -----------------------------------------------------------

    def _toggle_power(self):
        if self.status.get("running"):
            self.log("Остановка zapret…")
            self._run_async(lambda: core.stop_zapret())
        else:
            self.log("Запуск zapret…")
            self._run_async(lambda: core.run_zapret(self.cfg))

    def _download_deps(self):
        self.log("Скачивание зависимостей (nfqws + стратегии)…")
        self.log("Это может занять пару минут.")
        self._run_async(lambda: core.ensure_deps(
            self.cfg.get("nfqws_version", "latest"),
            self.cfg.get("strategy_rev", ""),
            self.log,
        ), lambda: self._refresh_combos())

    def _update_app(self):
        self.log("Обновление приложения…")
        from . import update
        self._run_async(lambda: update.update_app(self.log),
                        lambda: self.log("Обновление завершено."))

    def _toggle_service(self):
        if self.status.get("service_installed"):
            self.log("Удаление системной службы…")
            self._run_async(lambda: integration.remove_service())
        else:
            self.log("Установка системной службы…")
            self._run_async(lambda: integration.install_service())

    def _toggle_shortcut(self):
        if integration.shortcut_installed():
            integration.remove_shortcut()
            self.log("Ярлык приложения удалён.")
        else:
            integration.install_shortcut()
            self.log("Ярлык приложения создан в меню приложений.")
        self.refresh_status()

    def _setup_permissions(self):
        if self._open_terminal(
            f"{sys.executable} {app_dir() / 'run.py'} permissions install; "
            f"echo; read -p 'Нажмите Enter…'"
        ):
            self.log("Открыт терминал — введите пароль sudo для настройки прав.")
        else:
            messagebox.showinfo(
                "Права sudo",
                "Не найден эмулятор терминала.\n"
                "Выполните вручную в терминале:\n\n"
                f"  python3 {app_dir() / 'run.py'} permissions install",
            )

    def _open_terminal(self, command: str) -> bool:
        terms = [
            ("x-terminal-emulator", ["-e", "bash", "-c", command]),
            ("gnome-terminal", ["--", "bash", "-c", command]),
            ("konsole", ["-e", "bash", "-c", command]),
            ("xfce4-terminal", ["-e", "bash", "-c", command]),
            ("mate-terminal", ["-e", "bash", "-c", command]),
            ("alacritty", ["-e", "bash", "-c", command]),
            ("kitty", ["bash", "-c", command]),
            ("tilix", ["-e", "bash", "-c", command]),
            ("xterm", ["-e", "bash", "-c", command]),
        ]
        for term, args in terms:
            path = shutil.which(term)
            if path:
                subprocess.Popen([path] + args, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                return True
        return False

    def _show_help(self):
        messagebox.showinfo(
            "Справка",
            "1. Скачайте зависимости.\n"
            "2. Выберите стратегию и интерфейс.\n"
            "3. Нажмите «Запустить».\n\n"
            "Для работы без пароля настройте «Права sudo».\n"
            "«Обновить» скачивает свежий nfqws и стратегии.",
        )

    def _refresh_combos(self):
        self.strategy_combo["values"] = self._strategy_values()
        if self.cfg.get("strategy") in self._strategy_values():
            self.strategy_combo.set(self.cfg.get("strategy"))
        self.log("Зависимости готовы.")

    # -- переключатели ------------------------------------------------------

    def _save_and_restart(self, key, value):
        self.cfg[key] = value
        config_mod.save(self.cfg)
        self.log(f"Настройка {key} = {value} сохранена.")
        if self.status.get("running"):
            self.log("Перезапуск zapret для применения настроек…")
            self._run_async(lambda: core.run_zapret(self.cfg))

    def _on_telegram(self, on):
        self._save_and_restart("telegram", on)

    def _on_gft(self, on):
        self._save_and_restart("gamefilter_tcp", on)

    def _on_gfu(self, on):
        self._save_and_restart("gamefilter_udp", on)

    def _on_strategy(self, value):
        self.cfg["strategy"] = value
        config_mod.save(self.cfg)
        self.log(f"Стратегия: {value}")
        if self.status.get("running"):
            self._run_async(lambda: core.run_zapret(self.cfg))

    def _on_iface(self, value):
        self.cfg["interface"] = value
        config_mod.save(self.cfg)
        self.log(f"Интерфейс: {value}")
        if self.status.get("running"):
            self._run_async(lambda: core.run_zapret(self.cfg))

    def _on_backend(self, value):
        self.cfg["firewall_backend"] = value
        config_mod.save(self.cfg)
        self.log(f"Бэкенд: {value}")
        if self.status.get("running"):
            self.log("Перезапуск zapret для применения настроек…")
            self._run_async(lambda: core.run_zapret(self.cfg))


def main() -> int:
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"Не удалось открыть графический интерфейс: {exc}", file=sys.stderr)
        print("Убедитесь, что у вас есть графический сервер (X11/Wayland).", file=sys.stderr)
        return 1
    ZapretApp(root)
    root.mainloop()
    return 0
