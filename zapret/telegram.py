"""
Обход Telegram.

Telegram блокируется/замедляется по нескольким протоколам:
  * MTProto (TCP 443)      — детектируется ТСПУ по сигнатурам
  * веб/QUIC (UDP 443)     — web.telegram.org, звонки
  * голосовые звонки (UDP) — STUN/собственный протокол

Подход (собран из обсуждений bol-van/zapret #1860, #1668 и
Flowseal/zapret-discord-youtube #5904): применяем к доменам и IP-сетям
Telegram те же приёмы dpi-desync, что и к остальным сервисам —
fake TLS (подмена ClientHello) для TCP и fake QUIC для UDP.
"""

# Домены Telegram (web, CDN, прокси-зеркала и сервисы)
TELEGRAM_DOMAINS = [
    "t.me", "tg.dev", "tg.org", "tx.me", "teleg.xyz",
    "telegram.ai", "telegram.asia", "telegram.biz", "telegram.cloud",
    "telegram.cn", "telegram.co", "telegram.com", "telegram.de",
    "telegram.dev", "telegram.dog", "telegram.eu", "telegram.fr",
    "telegram.host", "telegram.in", "telegram.info", "telegram.io",
    "telegram.jp", "telegram.me", "telegram.net", "telegram.org",
    "telegram.qa", "telegram.ru", "telegram.services", "telegram.solutions",
    "telegram.space", "telegram.team", "telegram.tech", "telegram.uk",
    "telegram.us", "telegram.website", "telegram.xyz", "telegramapp.org",
    "telegra.ph", "telesco.pe", "nicegram.app", "telegramdownload.com",
    "cdn-telegram.org", "comments.app", "contest.com", "fragment.com",
    "graph.org", "quiz.directory", "tdesktop.com", "telega.one",
    "telegram-cdn.org", "usercontent.dev", "tgram.org", "torg.org",
]

# Публичные IP-сети Telegram (дата-центры)
TELEGRAM_IPS = [
    "91.108.0.0/16",
    "149.154.0.0/16",
    "185.76.151.0/24",
]


def telegram_domains_text() -> str:
    return "\n".join(TELEGRAM_DOMAINS) + "\n"


def telegram_ips_text() -> str:
    return "\n".join(TELEGRAM_IPS) + "\n"


# Отдельная стратегия "только Telegram" (совместима с парсером .bat-стратегий)
TELEGRAM_STRATEGY_BAT = """@echo off
chcp 65001 > nul
:: 65001 - UTF-8
:: Telegram: MTProto (TCP 443) + web/звонки (UDP 443, 50000-50100)

set "BIN=%~dp0bin\\"
set "LISTS=%~dp0lists\\"

start "zapret: telegram" /min "%BIN%winws.exe" --wf-tcp=80,443 --wf-udp=443,50000-50100 ^
--filter-tcp=80,443 --hostlist="%LISTS%list-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fooling=ts --dpi-desync-fake-tls="%BIN%tls_clienthello_www_google_com.bin" --new ^
--filter-tcp=80,443 --ipset="%LISTS%ipset-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fooling=ts --dpi-desync-fake-tls="%BIN%tls_clienthello_www_google_com.bin" --new ^
--filter-udp=443 --hostlist="%LISTS%list-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fake-quic="%BIN%quic_initial_www_google_com.bin" --new ^
--filter-udp=50000-50100 --dpi-desync=fake --dpi-desync-any-protocol=1 --dpi-desync-repeats=6 --dpi-desync-cutoff=n3
"""
