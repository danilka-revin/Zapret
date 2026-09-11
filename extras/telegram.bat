@echo off
chcp 65001 > nul
:: 65001 - UTF-8
:: Telegram: MTProto (TCP 443) + web/звонки (UDP 443, 50000-50100)

set "BIN=%~dp0bin\"
set "LISTS=%~dp0lists\"

start "zapret: telegram" /min "%BIN%winws.exe" --wf-tcp=80,443 --wf-udp=443,50000-50100 ^
--filter-tcp=80,443 --hostlist="%LISTS%list-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fooling=ts --dpi-desync-fake-tls="%BIN%tls_clienthello_www_google_com.bin" --new ^
--filter-tcp=80,443 --ipset="%LISTS%ipset-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fooling=ts --dpi-desync-fake-tls="%BIN%tls_clienthello_www_google_com.bin" --new ^
--filter-udp=443 --hostlist="%LISTS%list-telegram.txt" --hostlist-exclude="%LISTS%list-exclude.txt" --ipset-exclude="%LISTS%ipset-exclude.txt" --dpi-desync=fake --dpi-desync-repeats=6 --dpi-desync-fake-quic="%BIN%quic_initial_www_google_com.bin" --new ^
--filter-udp=50000-50100 --dpi-desync=fake --dpi-desync-any-protocol=1 --dpi-desync-repeats=6 --dpi-desync-cutoff=n3
