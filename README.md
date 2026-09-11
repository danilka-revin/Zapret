<div align="center">

# 🪟 Zapret Control

**Удобное приложение для Linux с обходом замедления/блокировки**
**YouTube · Discord · Telegram** — в стиле стеклянной тёмной темы (glassmorphism).

Установка, запуск и обновление — **одной командой**.

</div>

---

## 🚀 Установка / запуск / обновление одной командой

```bash
curl -fsSL https://raw.githubusercontent.com/danilka-revin/Zapret/main/install.sh -o /tmp/zc-install.sh && bash /tmp/zc-install.sh
```

Эта команда сама:

1. установит недостающие системные пакеты (`python3-tk`, `nftables`, `git`, `curl`);
2. скачает приложение;
3. скачает зависимости — бинарник **nfqws** и актуальные **стратегии** Flowseal;
4. создаст **ярлык в меню приложений**;
5. запустит графический интерфейс.

Повторный запуск той же команды — это **обновление**: код и зависимости
обновятся до последних версий.

> Если хотите установить из локальной копии репозитория:
> ```bash
> git clone https://github.com/danilka-revin/Zapret && cd Zapret && ./install.sh
> ```

---

## ✨ Возможности

- **Тёмная стеклянная тема** — glassmorphism: полупрозрачные карточки,
  мягкое свечение, акцентные цвета.
- **Обход Telegram** — отдельная стратегия + списки доменов и IP-сетей
  Telegram (`MTProto`, `web.telegram.org`, звонки).
- **YouTube и Discord** — на базе проверенных стратегий Flowseal.
- **Автоподбор стратегии** — встроенные стратегии `general`, `general_alt*`,
  `general_simple_fake*` и т. д.
- **Ярлык приложения** — `.desktop` в меню приложений (создание/удаление
  в один клик).
- **Автозапуск** — установка/удаление systemd-службы.
- **GameFilter** — обход DPI для игр (TCP/UDP).
- **Автообновление** — кнопка «Обновить» или повторный запуск `install.sh`.
- **Журнал** — живой лог всех операций.

## 🖥️ Скриншот-структура интерфейса

- **Статус** — запущено ли `nfqws`, активен ли файрвол, установлена ли служба,
  скачаны ли зависимости, настроены ли права sudo.
- **Сервисы** — переключатели Telegram / GameFilter TCP / GameFilter UDP.
- **Настройки** — выбор стратегии, сетевого интерфейса и бэкенда файрвола.
- **Управление** — запуск/остановка, скачивание зависимостей, ярлык,
  автозапуск, обновление.
- **Журнал** — вывод всех операций в реальном времени.

---

## 📋 Командная строка

Приложение также умеет всё делать без GUI (`python3 run.py …`):

```bash
python3 run.py gui                      # графический интерфейс
python3 run.py start | stop | restart   # запуск/остановка zapret
python3 run.py status                   # статус
python3 run.py ensure-deps              # скачать nfqws и стратегии
python3 run.py shortcut install|remove  # ярлык приложения
python3 run.py service install|remove|start|stop  # systemd-служба
python3 run.py permissions install|remove        # NOPASSWD для nft/nfqws
python3 run.py update                   # самообновление
```

---

## 🔐 Обход Telegram — как это работает

Telegram детектируется и замедляется провайдерами по нескольким протоколам:
`MTProto` (TCP 443), веб-версия и звонки (UDP/QUIC). Приложение добавляет к
основной стратегии отдельные правила:

- **Домены** Telegram (`t.me`, `telegram.org`, CDN `cdn-telegram.org`,
  `web.telegram.org`, `telegra.ph`, прокси-зеркала и т. д.) → файл
  `extras/list-telegram.txt`;
- **IP-сети** дата-центров Telegram (`91.108.0.0/16`, `149.154.0.0/16`,
  `185.76.151.0/24`) → файл `extras/ipset-telegram.txt`;
- **Стратегия** `extras/telegram.bat` — `dpi-desync=fake` с подменой TLS
  ClientHello (Google) для TCP и `fake-quic` для UDP.

Переключатель **«Telegram»** в интерфейсе включает/выключает эти правила,
не затрагивая остальные сервисы. Подход собран из обсуждений
[bol-van/zapret #1860](https://github.com/bol-van/zapret/discussions/1860),
[#1668](https://github.com/bol-van/zapret/discussions/1668) и
[Flowseal #5904](https://github.com/Flowseal/zapret-discord-youtube/discussions/5904).

> Эффективность зависит от провайдера. Если Telegram всё ещё тормозит —
> попробуйте другую стратегию (`general_alt*`) или настройте прокси в
> настройках самого Telegram.

---

## ⚙️ Требования

- Linux (проверено на Ubuntu/Debian, работает на Arch, Fedora и др.)
- `python3` + `python3-tk` (устанавливаются автоматически)
- `nftables` или `iptables`
- архитектуры `x86_64`, `aarch64`, `armv7l` и др. (nfqws подбирается автоматически)

## 📦 Как устроено

```
zapret/            # Python-пакет
├── gui.py         # интерфейс (tkinter, glassmorphism)
├── core.py        # ядро: nfqws, стратегии, файрвол
├── background.py  # генерация стеклянного фона (без зависимостей)
├── integration.py # systemd-служба, ярлык, sudoers
├── telegram.py    # данные для обхода Telegram
├── config.py      # конфигурация
└── update.py      # самообновление
extras/            # telegram.bat + списки Telegram
assets/            # иконка приложения
install.sh         # установка/запуск/обновление одной командой
run.py             # точка входа
```

Данные приложения хранятся в `~/.local/share/zapret-control/`:
`nfqws`, `deps/flowseal/` (стратегии), `config.json`.

---

## 🙏 Благодарности и лицензия

Проект основан на:

- [zapret](https://github.com/bol-van/zapret) — bol-van (MIT);
- [zapret-discord-youtube](https://github.com/Flowseal/zapret-discord-youtube) — Flowseal (MIT);
- [zapret-discord-youtube-linux](https://github.com/Sergeydigl3/zapret-discord-youtube-linux) — Sergeydigl3.

Бинарник `nfqws` и стратегии скачиваются из официальных релизов/репозиториев
напрямую и подчиняются их лицензиям (MIT). Код этого приложения —
см. `LICENSE` (MIT).

> ⚠️ Приложение предназначено для обхода искусственного замедления
> легитимных сервисов. Используйте только там, где это не запрещено
> законодательством вашей страны.
