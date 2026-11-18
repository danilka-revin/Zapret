/* Бортовой монитор для конечных пользователей.
 *
 * Показывает только общую информацию: АКБ, колёсные модули, обзор лидара,
 * последние события журнала и грузовой кейс по PIN. Настройки, руление и
 * тяга сюда не входят — их закрывает и сервер (режим «монитор»), поэтому
 * странице не нужно самой решать, что «нельзя».
 */
(function () {
  'use strict';

  const POLL_MS = 700;
  const LOG_MS = 4000;
  const LIDAR_RANGE_M = 10;
  const MAX_DIGITS = 8;
  const MIN_DIGITS = 4;

  // Встроенный предпросмотр — iframe на другом домене: браузер не сохраняет
  // cookie SameSite=Lax, поэтому режим подтверждается подписанным токеном из
  // адреса страницы (?m=…). Тот же токен добавляем к каждому запросу.
  const MODE_PARAM = 'm';
  const MODE_TOKEN = new URLSearchParams(window.location.search).get(MODE_PARAM) || '';

  const $ = (id) => document.getElementById(id);
  const setText = (el, text) => { if (el) el.textContent = text; };
  const num = (value, digits) => {
    if (value === null || value === undefined || !Number.isFinite(+value)) return '—';
    return (+value).toFixed(digits === undefined ? 1 : digits);
  };

  let pin = '';
  let caseOpen = false;
  let submitting = false;
  let lastState = null;
  let logSeq = 0;
  const lidarDots = [];

  // ----------------------------------- часы ----------------------------------
  function tickClock() {
    const now = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    setText($('mon-clock'), pad(now.getHours()) + ':' + pad(now.getMinutes()));
  }
  tickClock();
  setInterval(tickClock, 10000);

  // ------------------------------- общая связь -------------------------------
  function link(tone, text) {
    const chip = $('mon-link');
    if (!chip) return;
    chip.setAttribute('data-tone', tone);
    chip.textContent = text;
  }

  function withMode(path, method) {
    if (!MODE_TOKEN) return path;
    if (method && method !== 'GET') return path;
    const join = path.includes('?') ? '&' : '?';
    return path + join + MODE_PARAM + '=' + encodeURIComponent(MODE_TOKEN);
  }

  async function getJson(path, signal) {
    const response = await fetch(withMode(path), { cache: 'no-store', credentials: 'same-origin', signal });
    const body = await response.json().catch(() => ({}));
    return { response, body };
  }

  // ----------------------------------- АКБ -----------------------------------
  function renderBattery(snapshot) {
    // The JBD owner marks disconnected/stale snapshots even when they still
    // contain the last sample. Neither that sample nor chassis estimates are live.
    const data = (snapshot.connected === true && snapshot.data) || {};
    const soc = Number.isFinite(data.soc) && data.soc >= 0 && data.soc <= 100 ? data.soc : null;
    setText($('mon-soc'), soc === null ? '—' : String(Math.round(soc)));
    const bar = $('mon-soc-bar');
    if (bar) bar.style.width = soc === null ? '0%' : soc + '%';
    setText($('mon-volts'), num(data.volts, 1) + ' В');
    setText($('mon-amps'), num(data.amps, 1) + ' А');
    const temps = Array.isArray(data.temperatures) ? data.temperatures.filter(Number.isFinite) : [];
    setText($('mon-temp'), num(temps.length ? Math.max(...temps) : null, 0) + ' °C');
    let note = data.directionText || snapshot.reason || 'Нет данных BMS';
    if (soc !== null) {
      if (soc < 10) note = 'Критический разряд — сообщите оператору';
      else if (soc < 20) note = 'Низкий заряд — поездка может не состояться';
    }
    setText($('mon-soc-note'), note);
    setText($('mon-batt-note'), soc === null ? 'нет данных о заряде' : Math.round(soc) + '% заряда');
  }

  async function pollBattery() {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2500);
    try {
      const { response, body } = await getJson('api/battery', controller.signal);
      if (!response.ok || !body.ok || !body.battery) throw new Error('battery');
      renderBattery(body.battery);
    } catch (error) {
      renderBattery({reason: 'Нет связи с сервером BMS'});
    } finally {
      clearTimeout(timeout);
      setTimeout(pollBattery, 1000);
    }
  }

  // ---------------------------------- шасси ----------------------------------
  function renderChassis(data) {
    const motors = Array.isArray(data.motors) ? data.motors : [];
    const online = motors.filter((m) => m && m.online).length;
    setText($('mon-chassis-meta'),
      online + ' из 4 на связи' + (data.linkOk ? ' · шина ок' : ' · нет шины'));
    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const motor = motors.find((m) => m && m.id === id) || {};
      const card = $('mon-wheel-' + id);
      let tone = 'idle';
      let state = 'нет связи';
      if (motor.fault) { tone = 'err'; state = 'ошибка'; }
      else if (motor.online === false) { tone = 'idle'; state = 'нет связи'; }
      else if (motor.online) {
        if (motor.temp != null && +motor.temp > 70) { tone = 'warn'; state = 'нагрев'; }
        else { tone = 'ok'; state = motor.moving ? 'в движении' : 'готов'; }
      }
      if (card) {
        card.setAttribute('data-tone', tone);
        const chip = card.querySelector('.wheel-state');
        if (chip) chip.textContent = state;
        const dial = card.querySelector('.wheel-dial i');
        const angle = Number.isFinite(+motor.angle) ? +motor.angle : null;
        if (dial) dial.style.transform = 'translateX(-50%) rotate(' + (angle === null ? 0 : angle) + 'deg)';
        const value = card.querySelector('.wheel-num');
        if (value) value.textContent = angle === null ? '—' : angle.toFixed(0) + '°';
      }
    });
  }

  // ---------------------------------- лидар ----------------------------------
  function formatRange(meters) {
    return meters < 1 ? meters.toFixed(2) : (meters < 10 ? meters.toFixed(1) : String(Math.round(meters)));
  }

  function initLidarView() {
    const canvas = $('mon-lidar-canvas');
    if (!canvas || !window.RCLidarView || typeof window.RCLidarView.attach !== 'function') return;
    window.RCLidarView.attach(canvas);
    if (typeof canvas.addEventListener === 'function') {
      canvas.addEventListener('rc-lidar-view', (event) => {
        const radius = event && event.detail && event.detail.radius;
        setText($('mon-lidar-range'), Number.isFinite(+radius) ? formatRange(+radius) + ' м' : '10 м');
      });
    }
  }

  function renderLidar(data) {
    const points = Array.isArray(data.lidarPoints) ? data.lidarPoints : null;
    const scan = points || [];
    const status = data.lidarStatus || {};
    if (window.RCKinectView && typeof window.RCKinectView.setLidarScan === 'function') {
      window.RCKinectView.setLidarScan(scan, status);
    }
    if (window.RCKinectView && typeof window.RCKinectView.active === 'function' && window.RCKinectView.active()) {
      return;
    }
    const canvas = $('mon-lidar-canvas');
    const empty = $('mon-lidar-empty');
    const fresh = !!status.connected && scan.length > 0;
    setText($('mon-lidar-count'), fresh ? String(scan.length) : '—');

    let near = null;
    if (fresh) {
      scan.forEach((p) => {
        const r = +p.r;
        if (Number.isFinite(r) && r >= 0.05 && (near === null || r < near)) near = r;
      });
    }
    setText($('mon-lidar-near'), near === null ? '—' : near.toFixed(2) + ' м');

    setText($('mon-lidar-meta'), fresh
      ? 'сканирование · ' + scan.length + ' точек · ' + (status.source || 'лидар')
      : (status.reason || 'скана лидара нет'));
    if (empty) {
      empty.hidden = fresh;
      empty.textContent = fresh ? '' : (status.reason || 'скана лидара нет');
    }
    if (!canvas) return;
    if (window.RCLidarView && typeof window.RCLidarView.render === 'function') {
      window.RCLidarView.render(canvas, fresh ? scan : [], {
        stamp: status.scan_at != null ? status.scan_at : null,
        lidarOffsetM: status.lidar_offset_m != null ? status.lidar_offset_m : null,
      });
      return;
    }
    drawLidar(canvas, scan, fresh);
  }

  function drawLidar(canvas, scan, fresh) {
    const ratio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth || 480;
    const height = canvas.clientHeight || 320;
    if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
      canvas.width = Math.round(width * ratio);
      canvas.height = Math.round(height * ratio);
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);

    const cx = width / 2;
    const cy = height / 2;
    const scale = (Math.min(width, height) / 2 - 14) / LIDAR_RANGE_M;

    // сетка дальности: 2 / 4 / 6 / 8 / 10 м
    ctx.strokeStyle = 'rgba(46,46,52,.9)';
    ctx.lineWidth = 1;
    for (let ring = 2; ring <= LIDAR_RANGE_M; ring += 2) {
      ctx.beginPath();
      ctx.arc(cx, cy, ring * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    // робот в центре
    ctx.fillStyle = '#c0505a';
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();

    if (!fresh) {
      // без скана — честно пустое поле, без выдуманных точек
      return;
    }
    ctx.fillStyle = '#c8d6ff';
    scan.forEach((point) => {
      const angle = +point.a;
      const range = +point.r;
      if (!Number.isFinite(angle) || !Number.isFinite(range) || range > LIDAR_RANGE_M) return;
      const x = cx + Math.cos(angle) * range * scale;
      const y = cy - Math.sin(angle) * range * scale;
      ctx.fillRect(x - 1.3, y - 1.3, 2.6, 2.6);
    });
    if (lidarDots.length === 0) lidarDots.push(scan.length);
  }

  // ---------------------------------- журнал ---------------------------------
  function renderLog(entries, total) {
    const box = $('mon-log');
    if (!box) return;
    const items = (entries || []).slice(-9);
    if (!items.length) {
      box.innerHTML = '<p class="log-empty">Пока событий нет</p>';
      return;
    }
    box.innerHTML = '';
    items.forEach((entry) => {
      const row = document.createElement('div');
      row.className = 'log-line';
      row.setAttribute('data-level', entry.level || 'info');
      const time = document.createElement('time');
      time.textContent = entry.time || new Date((entry.ts || 0) * 1000).toLocaleTimeString('ru-RU');
      const text = document.createElement('span');
      text.textContent = (entry.msg || '') + (entry.repeat > 1 ? ' ×' + entry.repeat : '');
      row.appendChild(time);
      row.appendChild(text);
      box.appendChild(row);
    });
    setText($('mon-log-meta'), 'всего записей: ' +
      (Number.isFinite(+total) && +total > 0 ? +total : items.length));
  }

  // ----------------------------------- кейс ----------------------------------
  function renderCase(lock) {
    const state = lock || {};
    caseOpen = !!state.open;
    const icon = $('mon-case-icon');
    if (icon) icon.setAttribute('data-open', caseOpen ? '1' : '0');
    setText($('mon-case-state'), caseOpen ? 'Открыт' : 'Закрыт');
    setText($('mon-case-chip'), caseOpen ? 'можно загружать' : 'закрыт');
    const pinBox = $('mon-case-pin');
    if (pinBox) pinBox.hidden = caseOpen;
    if (caseOpen) {
      setText($('mon-case-detail'), 'можно загружать и выгружать груз');
      setText($('mon-case-msg'), '');
    } else if (state.blocked) {
      setText($('mon-case-detail'),
        'ввод заблокирован ещё ' + Math.ceil((state.remainingMs || 0) / 1000) + ' с');
    } else {
      setText($('mon-case-detail'), 'введите PIN, чтобы открыть');
    }
    const note = $('mon-case-note');
    if (note) {
      note.textContent = caseOpen
        ? 'Кейс открыт по PIN. Закройте его крышкой — или с обычного бортового экрана ' +
          '(PIN 7232): с монитора закрытие недоступно.'
        : 'Кейс открывается только PIN-кодом. Закрывается крышкой или с обычного бортового ' +
          'экрана — на мониторе этой команды нет.';
    }
    renderDots();
  }

  function renderDots() {
    const box = $('mon-case-dots');
    if (!box) return;
    const shown = Math.max(pin.length, MIN_DIGITS);
    while (box.children.length < shown) {
      const dot = document.createElement('span');
      dot.className = 'case-dot';
      box.appendChild(dot);
    }
    while (box.children.length > shown) box.removeChild(box.lastElementChild);
    Array.from(box.children).forEach((dot, index) => {
      dot.setAttribute('data-on', index < pin.length ? '1' : '0');
    });
    const submit = $('mon-case-open');
    if (submit) submit.disabled = submitting || caseOpen || pin.length < MIN_DIGITS;
  }

  function caseMessage(text, tone) {
    const el = $('mon-case-msg');
    if (!el) return;
    el.textContent = text;
    if (tone) el.setAttribute('data-tone', tone);
    else el.removeAttribute('data-tone');
  }

  function addDigit(digit) {
    if (submitting || caseOpen) return;
    if (pin.length >= MAX_DIGITS) {
      caseMessage('Не больше 8 цифр.', 'err');
      return;
    }
    pin += digit;
    renderDots();
  }

  function eraseDigit() {
    if (submitting) return;
    pin = pin.slice(0, -1);
    renderDots();
  }

  async function openCase() {
    if (submitting || caseOpen) return;
    if (pin.length < MIN_DIGITS) {
      caseMessage('PIN кейса — 4…8 цифр.', 'err');
      return;
    }
    submitting = true;
    renderDots();
    caseMessage('Проверяю PIN…');
    try {
      const response = await fetch(withMode('api/kiosk_lock/open', 'POST'), {
        method: 'POST',
        cache: 'no-store',
        credentials: 'same-origin',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pin }),
      });
      const body = await response.json().catch(() => ({}));
      if (response.ok && body.ok) {
        pin = '';
        renderCase(body.lock || { open: true });
        caseMessage('Кейс открыт. Загружайте или выгружайте груз.', 'ok');
        return;
      }
      pin = '';
      renderDots();
      if (body.reason === 'blocked') {
        caseMessage('Слишком много попыток: подождите ' +
          Math.ceil((body.remainingMs || 30000) / 1000) + ' с.', 'err');
      } else if (body.reason === 'wrong') {
        caseMessage('Неверный PIN кейса.' +
          (typeof body.attemptsLeft === 'number' ? ' Осталось попыток: ' + body.attemptsLeft + '.' : ''), 'err');
      } else if (body.reason === 'monitor_mode') {
        caseMessage(body.error || 'Действие недоступно на мониторе.', 'err');
      } else if (body.reason === 'format') {
        caseMessage('PIN кейса — 4…8 цифр.', 'err');
      } else {
        caseMessage(body.error || 'Кейс не открылся: проверьте PIN.', 'err');
      }
    } catch (error) {
      pin = '';
      renderDots();
      caseMessage('Нет связи с бортовым сервером.', 'err');
    } finally {
      submitting = false;
      renderDots();
    }
  }

  const pad = $('mon-case-pin');
  if (pad) {
    pad.addEventListener('click', (event) => {
      const button = event.target.closest('button');
      if (!button || button.disabled) return;
      if (button.dataset.digit) addDigit(button.dataset.digit);
      else if (button.dataset.action === 'erase') eraseDigit();
      else if (button.dataset.action === 'open') openCase();
    });
  }

  function isSensorFullscreen() {
    return !!(window.RCKinectView && typeof window.RCKinectView.isFullscreen === 'function' &&
      window.RCKinectView.isFullscreen());
  }

  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey || event.altKey || event.metaKey) return;
    if (isSensorFullscreen()) return;
    const target = event.target;
    if (target && target.id === 'kinect-3d-canvas' &&
        (event.key === 'ArrowLeft' || event.key === 'ArrowRight' ||
         event.key === 'ArrowUp' || event.key === 'ArrowDown')) {
      return;
    }
    if (/^[0-9]$/.test(event.key)) { addDigit(event.key); event.preventDefault(); }
    else if (event.key === 'Backspace') { eraseDigit(); event.preventDefault(); }
    else if (event.key === 'Enter') { openCase(); event.preventDefault(); }
    else if (event.key === 'Escape') { pin = ''; renderDots(); event.preventDefault(); }
  });

  // ------------------------------------ выход --------------------------------
  const exit = $('mon-exit');
  if (exit) {
    exit.addEventListener('click', async () => {
      exit.disabled = true;
      try {
        await fetch(withMode('api/kiosk_mode/exit', 'POST'), {
          method: 'POST', cache: 'no-store', credentials: 'same-origin',
          headers: { 'Content-Type': 'application/json' }, body: '{}',
        });
      } catch (error) { /* всё равно уходим на стартовый экран */ }
      window.location.assign('/?choose=1');
    });
  }

  // -------------------------------- опрос состояния --------------------------
  function currentLidarPayload() {
    const data = lastState || {};
    return {
      lidarPoints: Array.isArray(data.lidar) ? data.lidar.map((p) => ({a: p.a, r: p.r})) : null,
      lidarStatus: data.lidarStatus || {},
    };
  }

  async function pollState() {
    try {
      const { response, body } = await getJson('api/kiosk_state');
      if (!response.ok || !body.ok) throw new Error((body && body.error) || 'state');
      const data = body.data || {};
      lastState = data;
      if (window.RCRobotModel && typeof window.RCRobotModel.update === 'function') {
        window.RCRobotModel.update(data);
      }
      const speed = Number.isFinite(data.wheelSpeedMps) ? data.wheelSpeedMps : data.speedMps;
      setText($('mon-speed'), num(speed, 2) + ' м/с');
      renderChassis(data);
      renderLidar(currentLidarPayload());
      setText($('mon-source'), 'источник: ' + (data.source || '—'));
      link('ok', '● связь с бортом');
    } catch (error) {
      link('warn', '○ нет связи');
      if (window.RCKinectView && typeof window.RCKinectView.setLidarScan === 'function') {
        window.RCKinectView.setLidarScan([], {connected: false, age_s: null});
      }
      setText($('mon-lidar-meta'), 'нет связи с бортовым сервером');
      setText($('mon-speed'), '— м/с');
    }
  }

  async function pollVisionField() {
    if (!window.RCKinectView || typeof window.RCKinectView.mode !== 'function' ||
        window.RCKinectView.mode() !== 'combined') {
      return;
    }
    try {
      const { response, body } = await getJson('api/vision/field');
      if (response.ok && body && body.ok && typeof window.RCKinectView.setVisionMarkers === 'function') {
        window.RCKinectView.setVisionMarkers(body.field);
      }
    } catch (error) { /* знаки в 3D необязательны */ }
  }

  async function pollCase() {
    try {
      const { response, body } = await getJson('api/kiosk_lock');
      if (response.ok && body && body.ok) renderCase(body.lock || {});
    } catch (error) { /* состояние кейса обновится на следующем опросе */ }
  }

  async function pollLog() {
    try {
      const { response, body } = await getJson('api/kiosk_logs?limit=60');
      if (!response.ok || !body.ok) return;
      const entries = (body.data && body.data.entries) || [];
      const last = entries.length ? entries[entries.length - 1].seq : 0;
      if (last !== logSeq) {
        logSeq = last;
        renderLog(entries, (body.data && body.data.count) || entries.length);
      }
    } catch (error) { /* журнал не критичен для экрана */ }
  }

  initLidarView();
  renderDots();
  renderCase({});
  pollBattery();
  pollState();
  pollCase();
  pollLog();
  setInterval(pollState, POLL_MS);
  setInterval(pollCase, 2500);
  setInterval(pollLog, LOG_MS);
  if (window.RCKinectView) setInterval(pollVisionField, 1000);
  const redrawLidar = () => {
    const canvas = $('mon-lidar-canvas');
    if (canvas) renderLidar(currentLidarPayload());
  };
  window.addEventListener('resize', redrawLidar);
  window.addEventListener('rc-sensor-layout', () => {
    redrawLidar();
    if (window.RCKinectView && window.RCKinectView.mode && window.RCKinectView.mode() === 'combined') {
      void pollVisionField();
    }
  });
})();
