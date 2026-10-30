/* Бортовой экран: телеметрия и функции без PIN; PIN нужен только для кейса. */
(function () {
  'use strict';

  const LIDAR_RANGE_M = 10;
  const POLL_MS = 250;

  const $ = (id) => document.getElementById(id);
  const setText = (el, txt) => { if (el) el.textContent = txt; };
  const fmt = (v, d) => {
    if (v === null || v === undefined || !Number.isFinite(+v)) return '—';
    return (+v).toFixed(d == null ? 1 : d);
  };

  // SC v16 — отдельное поле только для показа. Старое rpm оставлено для
  // совместимости с прежними платами; отсутствие данных не выдаём за ноль.
  function rpmText(motor, decimals = 1) {
    if (!live || !motor || motor.online === false) return '—';
    if (typeof motor.scRpm === 'number' && Number.isFinite(motor.scRpm) && motor.scRpm >= 0) {
      return fmt(motor.scRpm, decimals);
    }
    // В v16 tach=false означает отключённую одометрию, а не отсутствие SC.
    if (motor.firmware >= 16) return '—';
    if (motor.tach === false) return 'нет';
    return typeof motor.rpm === 'number' && Number.isFinite(motor.rpm) && motor.rpm >= 0
      ? fmt(motor.rpm, decimals) : '—';
  }

  function setSteeringAuthorized(on) {
    if (!on) stopDrive();
    steeringAuthorized = !!on;
    renderSteeringControls(lastData || {});
  }

  let lastData = null;
  let live = false;
  let lastLiveAt = 0;
  let liveTimer = null;
  // На киоске PIN нужен только для открытия грузового отсека. Серверная
  // авторизация руления/тяги остаётся отдельной настройкой веб-пульта.
  let steeringAuthorized = true;
  let stateFetchPending = false;
  // Ползунок задаёт число; кнопка включает направление до СТОП.
  const SERVO_IDS = ['FL', 'FR', 'RL', 'RR'];
  const SERVO_MAX = 150;
  let servoCommonSpeed = 0;
  const servoSpeeds = { FL: 0, FR: 0, RL: 0, RR: 0 };
  const servoTargets = { FL: 0, FR: 0, RL: 0, RR: 0 };
  const servoDirections = { FL: 'forward', FR: 'forward', RL: 'forward', RR: 'forward' };
  const servoRuns = new Map();
  const servoPending = new Map();
  const moduleDriveNote = Object.create(null);
  let driveTimer = null;
  let driveLastTick = 0;
  let driveSending = false;
  let driveFlush = Promise.resolve();
  let driveMessage = null;
  const pendingSteer = Object.create(null);
  const pendingZero = Object.create(null);
  const pendingCalib = Object.create(null);
  const steerResult = Object.create(null);
  // Сколько даём колесу доехать и остановиться, прежде чем признать команду
  // неисполненной. Поворот на 90° — 1,2 с плюс доводка по потенциометру
  // (до трёх проходов по ~0,3 с): четыре секунды покрывают худший случай и
  // при этом не держат экран заблокированным, как прежние 15 с.
  const STEER_SETTLE_MS = 4000;
  // Аварийный срок на случай молчащей телеметрии (колесо отвалилось от шины).
  const STEER_TELEMETRY_TIMEOUT_MS = 15000;

  function setLive(ok) {
    // Дебаунс «нет связи»: не моргаем всем окном на каждый промах сети.
    // Бортовой экран раньше тонул в opacity .75 каждые ~20 с на 2 с: лидар ищет
    // порт ~2.2 с и держит тред сервера, fetchState получает AbortError на
    // 1100 мс и live гас; теперь live -> false только после 3200 мс без
    // успешных ответов, а live -> true — сразу при следующем удачном опросе.
    if (ok) {
      lastLiveAt = Date.now();
      if (!live) {
        live = true;
        if (liveTimer) { clearTimeout(liveTimer); liveTimer = null; }
      }
      return;
    }
    if (!live) return;
    const elapsed = Date.now() - lastLiveAt;
    const grace = 3200;
    if (elapsed >= grace) {
      live = false;
      return;
    }
    if (liveTimer) clearTimeout(liveTimer);
    liveTimer = setTimeout(() => {
      if (Date.now() - lastLiveAt >= grace) live = false;
    }, grace - elapsed + 20);
  }

  async function fetchState() {
    // Не запускаем параллельные 250-мс опросы: медленный ответ не должен
    // перезаписать более свежий скан лидара или телеметрию.
    // + таймаут, чтобы зависший запрос не держал UI в «нет связи» 2 сек.
    if (stateFetchPending) return;
    stateFetchPending = true;
    const ctrl = new AbortController();
    const t = setTimeout(() => ctrl.abort(), 1100);
    try {
      const r = await fetch('api/kiosk_state', { cache: 'no-store', signal: ctrl.signal });
      if (!r.ok) throw new Error('http ' + r.status);
      const j = await r.json();
      if (j && j.ok && j.data) {
        lastData = j.data;
        if (window.RCRemoteControls && window.RCRemoteLegacyState) window.RCRemoteLegacyState(j.data);
        // Потеря платы или E-STOP не должны автоматически перезапустить её
        // после восстановления связи. Калибровка руля и тахометр здесь ни при чём.
        if ((servoRuns.size || hasServoTargets()) && !servoLinksReady(j.data)) releaseDrive();
        if (window.RCRobotModel) window.RCRobotModel.update(j.data);
        setLive(true);
      } else {
        setLive(false);
        if (window.RCRemoteControls) {
          releaseDrive();
          if (window.RCRemoteLegacyStop) window.RCRemoteLegacyStop();
          lastData = null;
        }
      }
    } catch (e) {
      // AbortError или сеть — не сразу гасим весь экран, даём grace-период
      setLive(false);
      if (window.RCRemoteControls) {
        releaseDrive();
        if (window.RCRemoteLegacyStop) window.RCRemoteLegacyStop();
        lastData = null;
      }
    } finally {
      clearTimeout(t);
      stateFetchPending = false;
    }
  }

  function clockText(d) {
    const pad = (n) => String(n).padStart(2, '0');
    return pad(d.getHours()) + ':' + pad(d.getMinutes());
  }

  function batteryPct(b) {
    if (b && b.soc != null && Number.isFinite(+b.soc)) return Math.max(0, Math.min(100, +b.soc));
    return null;
  }

  function renderBattery(d) {
    // A configured JBD BMS is authoritative; never fall back to simulated SOC
    // when its USB link is lost. Range is not provided by the JBD registers.
    const bms = window._kioskBms;
    const useBms = Boolean(bms && bms.configured);
    const measured = useBms && bms.connected && bms.data ? bms.data : {};
    const b = useBms ? {
      soc: measured.soc, volts: measured.volts, amps: measured.amps,
      tempC: measured.temperatures && measured.temperatures.length ? Math.max(...measured.temperatures) : null,
      rangeKm: null, state: bms.connected ? 'JBD BMS · ' + bms.reason : 'JBD BMS · нет свежих данных'
    } : (d.battery || {});
    const pct = batteryPct(b);
    const socEl = $('k-soc');
    const bar = $('k-soc-bar');
    setText(socEl, pct == null ? '—' : Math.round(pct));
    if (bar) bar.style.width = pct == null ? '0%' : pct + '%';

    let stateText = b.state || '—';
    document.body.classList.remove('soc-low', 'soc-crit');
    if (pct != null) {
      if (pct < 10) { stateText = 'КРИТИЧЕСКИЙ РАЗРЯД'; document.body.classList.add('soc-crit'); }
      else if (pct < 20) { stateText = 'Низкий заряд'; document.body.classList.add('soc-low'); }
    }
    setText($('k-soc-state'), stateText);

    // Подсказка на главном экране: панель АКБ — вход на страницу 4 (все банки).
    const more = $('battery-more');
    if (more) {
      let hint = 'Подробно: все банки и защиты →';
      if (bms && bms.configured === false) hint = 'BMS не подключена · настроить →';
      else if (useBms && !bms.connected) hint = 'BMS не отвечает · проверить →';
      else if (useBms && measured.cellCount) hint = 'Подробно: ' + measured.cellCount + ' банок и защиты →';
      more.textContent = hint;
      more.dataset.tone = bms && (bms.configured === false || (useBms && !bms.connected)) ? 'warn' : '';
    }

    setText($('k-volts'), fmt(b.volts, 1));
    setText($('k-amps'), fmt(b.amps, 1));
    setText($('k-power'), fmt(useBms ? (measured.watts == null ? null : measured.watts / 1000) : d.powerKw, 2));
    setText($('k-range'), fmt(b.rangeKm, 1));
    setText($('k-btemp'), fmt(b.tempC, 0));
    setText($('speed'), fmt(d.speedMps, 1));
  }

  function modTone(m) {
    if (!m) return 'idle';
    if (m.fault) return 'err';
    if (m.online === false) return 'idle';
    if (m.temp != null && +m.temp > 70) return 'warn';
    if (m.online) {
      if (m.moving) return 'move';
      return 'ok';
    }
    return 'idle';
  }

  function modStateText(m) {
    if (!m) return '—';
    if (m.fault) return 'ошибка';
    if (m.online === false) return 'нет связи';
    if (m.moving) return 'в движении';
    return 'онлайн';
  }

  function renderChassis(d) {
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const link = !!d.linkOk;
    const moving = motors.filter((m) => m && m.moving).length;
    const online = motors.filter((m) => m && m.online).length;
    const parts = [];
    if (d.mode) parts.push(d.mode);
    parts.push('модулей: ' + online + '/4');
    if (moving) parts.push('рулевых модулей в движении ' + moving);
    parts.push(link ? 'связь: ок' : 'нет связи');
    setText($('chassis-meta'), parts.join(' · '));

    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const m = motors.find((x) => x.id === id) || {};
      const tone = modTone(m);
      const card = $('mod-' + id);
      const wheel = $('wheel-' + id);
      if (card) card.setAttribute('data-tone', tone);
      if (wheel) {
        wheel.setAttribute('data-tone', tone);
        wheel.classList.toggle('is-driving', !!m.moving);
      }
      const chip = $('mod-state-' + id);
      if (chip) {
        chip.textContent = modStateText(m);
        chip.className = 'chip ' + tone;
      }
      const hasAngle = m.angle != null && Number.isFinite(+m.angle);
      const angle = hasAngle ? Math.max(-90, Math.min(90, +m.angle)) : 0;
      setText($('angle-' + id), hasAngle ? Math.round(angle) + '°' : '—');
      if (wheel) wheel.style.transform = 'rotate(' + angle.toFixed(1) + 'deg)';
      // 0: нет достаточных недавних переходов; обрыв и остановка неотличимы.
      setText($('rpm-' + id), rpmText(m));
      setText($('temp-' + id), m.temp == null ? '—' : fmt(m.temp, 0));
    });
  }

  function renderMode(d) {
    const pill = $('mode-pill');
    if (!pill) return;
    const link = !!d.linkOk;
    if (!link) {
      pill.textContent = 'НЕТ СВЯЗИ';
      pill.className = 'mode-pill err';
    } else if (d.mode) {
      const t = String(d.mode);
      pill.textContent = t;
      if (/ошиб|неисправ|авар/i.test(t)) pill.className = 'mode-pill err';
      else if (/режим|автоном|готов/i.test(t)) pill.className = 'mode-pill';
      else pill.className = 'mode-pill idle';
    } else {
      pill.textContent = 'ОЖИДАНИЕ';
      pill.className = 'mode-pill idle';
    }
  }

  function lidarPoints(d) {
    if (!Array.isArray(d.lidar)) return [];
    return d.lidar.filter((p) => p && Number.isFinite(p.a) && Number.isFinite(p.r) && p.r >= 0);
  }

  // Откуда реально читается скан (GET /api/lidar): драйвер ROS 2 с топиком или
  // USB-порт приложения. Подпись под картой — единственное место, где оператор
  // видит, кто сейчас держит лидар, поэтому ROS 2 назван явно.
  function lidarSourceText(d) {
    const st = d && d.lidarStatus;
    if (!st) return '';
    const ros = st.ros;
    if (ros && ros.connected) {
      const parts = ['ROS 2', String(ros.topic || '/scan')];
      const supervisor = ros.supervisor;
      if (supervisor && supervisor.driver) {
        parts.push(String(supervisor.driver_package || supervisor.driver));
      }
      if (ros.hz) parts.push(ros.hz + ' Гц');
      return parts.join(' · ');
    }
    const det = st.detected;
    if (!det || !det.port) return '';
    const parts = [String(det.port).split('/').pop()];
    if (det.protocol) parts.push(String(det.protocol));
    if (det.baud) parts.push(det.baud + ' бод');
    if (st.source === 'peer') parts.push('скан от пульта');
    return parts.join(' · ');
  }

  // Карта в стиле RViz — модуль lidar_view.js (сетка, оси, шасси, зум, Decay Time).
  function drawLidar(canvas, pts, d) {
    if (!canvas) return;
    const stamp = d && d.lidarStatus ? d.lidarStatus.scan_at : null;
    if (window.RCLidarView) window.RCLidarView.render(canvas, pts, { stamp });
  }

  // На основной карте остаётся короткая подпись, а подробная диагностика,
  // причина отсутствия скана и список USB-портов обновляются в панели настроек.

  function renderLidar(d) {
    const canvas = $('lidar-canvas');
    const pts = lidarPoints(d);
    const has = pts.length > 0;
    const visibleCount = pts.filter((point) => point.r <= LIDAR_RANGE_M).length;

    const source = has ? lidarSourceText(d) : '';
    setText($('lidar-meta'), has
      ? 'в реальном времени · ' + pts.length + ' точек · на карте: ' + visibleCount + '/' + pts.length + (source ? ' · ' + source : '')
      : 'нет скана');
    setText($('lidar-count'), has ? String(pts.length) : '—');
    const empty = $('lidar-empty');
    if (empty) {
      empty.hidden = has;
      if (!has) setText(empty, 'скана лидара нет');
      empty.title = '';
    }
    drawLidar(canvas, pts, d);
  }

  // Экран рисуется по блокам, и сбой одного не гасит остальные. Раньше любое
  // исключение в render() обрывало и loop() (он без try/catch), и бортовой
  // экран замирал целиком: ни часов, ни карты лидара — при живом сервере.
  const failedSteps = {};

  function step(name, draw) {
    try {
      draw();
    } catch (err) {
      if (!failedSteps[name]) {
        failedSteps[name] = true;
        console.warn('Бортовой экран: блок ' + name + ' не нарисован:', err && err.message ? err.message : err);
      }
    }
  }

  function render() {
    const d = lastData || {};
    step('clock', () => setText($('clock'), clockText(new Date())));
    step('link', () => {
      const dataChip = $('data-chip');
      if (!dataChip) return;
      if (!live) {
        dataChip.textContent = 'НЕТ СВЯЗИ С СЕРВЕРОМ';
        dataChip.className = 'chip err';
      } else if (d.linkOk === false) {
        dataChip.textContent = 'нет данных от робота';
        dataChip.className = 'chip warn';
      } else {
        dataChip.textContent = 'в реальном времени';
        dataChip.className = 'chip ok';
      }
    });
    step('stale', () => document.body.classList.toggle('is-stale', !live));

    step('mode', () => renderMode(d));
    step('battery', () => renderBattery(d));
    step('chassis', () => renderChassis(d));
    step('lidar', () => renderLidar(d));
    step('steering', () => renderSteeringControls(d));
    // Живой скан идёт и на страницу карты: там он рисуется поверх сетки.
    step('map-scan', () => {
      if (window.RCMapView) window.RCMapView.setScan(lidarPoints(d));
    });
  }

  function loop() {
    try {
      render();
    } catch (err) {
      console.warn('Бортовой экран: сбой отрисовки:', err && err.message ? err.message : err);
    } finally {
      // Инкрементальная перерисовка каждые 250 мс без полной пересборки DOM.
      // Раньше связка rAF+setTimeout давала двойную задержку и лишние кадры;
      // теперь — ровный таймер, а тяжёлые canvas (лидар/камеры) рисуются только
      // когда есть новые данные, без сброса ширины окна.
      setTimeout(() => requestAnimationFrame(loop), 250);
    }
  }

  // Дальность в карточке — радиус обзора карты: меняется при зуме щипком,
  // колесом или кнопками «+ −» и сразу возвращается к 10 м по «⊙».
  function bindLidarRange() {
    const canvas = $('lidar-canvas');
    if (!canvas || !window.RCLidarView) return;
    window.RCLidarView.attach(canvas);
    canvas.addEventListener('rc-lidar-view', (event) => {
      const r = event.detail && event.detail.radius;
      setText($('lidar-range'), Number.isFinite(+r) ? fmtRange(+r) + ' м' : '—');
    });
  }
  function fmtRange(m) {
    return m < 1 ? m.toFixed(2) : (m < 10 ? m.toFixed(1) : String(Math.round(m)));
  }

  function boot() {
    fetchState();
    setInterval(fetchState, POLL_MS);
    bindLidarRange();
    loop();
    initKioskControls();
    initSwipePage();
    initSteeringControls();
    if (window.RCRemoteControls && window._openSwipe) {
      const view = document.body.dataset.remoteView === 'battery' ? 'battery' : 'motors';
      window._openSwipe(view);
    }
    // Страница карты лидара: опрос сервера и кнопки (сброс, настройки).
    if (window.RCMapView) window.RCMapView.init();
  }

  /* =======================================================================
     КИОСК: функции доступны без PIN; PIN-клавиатура вызывается только при
     открытии грузового отсека. Руление и тяга сохраняют свои лимиты и watchdog.
     ======================================================================= */
  function initKioskControls() {
    const overlay = $('ctrl-overlay');
    const backdrop = $('ctrl-backdrop');
    const sheet = overlay ? overlay.querySelector('.ctrl-sheet') : null;
    const openBtn = $('k-ctrl-btn');
    const closeBtn = $('ctrl-close');
    const pinView = $('pin-view');
    const ctrlView = $('ctrl-view');
    const pinDots = $('pin-dots');
    const pinDisplay = $('pin-display');
    const pinMsg = $('pin-msg');
    const pinPad = $('pin-pad');
    const pinSubmit = $('pin-submit');
    const pinCancel = $('pin-cancel');
    const ctrlBack = $('ctrl-back');
    const ctrlLogout = $('ctrl-logout');
    const lockChip = $('ctrl-lock-chip');
    const lockDetail = $('ctrl-lock-detail');
    const attemptsEl = $('ctrl-attempts');
    const subtitle = $('ctrl-subtitle');
    const titleEl = document.getElementById('ctrl-title');
    const compartmentBtn = $('btn-compartment');
    const btnSteeringControls = $('btn-steering-controls');
    const compartmentTitle = $('compartment-title');
    const compartmentHint = $('compartment-hint');
    const autoscanBtn = $('btn-autoscan');
    const autoscanHint = $('autoscan-hint');
    const autoscanState = $('autoscan-state');
    const scanBeam = $('lidar-scan-beam');
    const scanLabel = $('lidar-scan-label');
    const lidarSection = document.querySelector('.lidar');
    const btnThemes = $('btn-themes');
    const themesView = $('themes-view');
    const themesGrid = $('themes-grid');
    const themesBack = $('themes-back');
    const themesToStats = $('themes-to-stats');
    const btnResetMap = $('btn-reset-map');
    const btnMapView = $('btn-map-view');
    const resetView = $('reset-view');
    const resetBack = $('reset-back');
    const resetCancel = $('reset-cancel');
    const resetConfirm = $('reset-confirm');
    const resetConfirmCheck = $('reset-confirm-check');
    const resetMsg = $('reset-msg');

    if (!overlay || !openBtn || !pinDots) return;

    let pinBuf = '';
    let lockState = null;   // {open, blocked, remainingMs, attemptsLeft}
    let lockTimer = null;
    let busy = false;
    let dragging = false;
    let lastHoveredBtn = null;
    // авто-сканирование лидара (по умолчанию выкл)
    let autoScanOn = false;
    try { autoScanOn = localStorage.getItem('kiosk_autoscan') === '1'; } catch(e) {}

    // ---------- helpers ----------
    function setHidden(el, h) { if (el) el.hidden = h; }
    function setMsg(text, kind) {
      if (!pinMsg) return;
      pinMsg.textContent = text || '';
      pinMsg.className = 'pin-msg' + (kind ? ' ' + kind : '');
    }
    function vibrate(pattern) {
      try { if (navigator.vibrate) navigator.vibrate(pattern); } catch (e) {}
    }
    function updatePinDisplay() {
      const dots = pinDots.querySelectorAll('span');
      dots.forEach((sp, i) => sp.classList.toggle('filled', i < pinBuf.length));
      if (pinDisplay) {
        if (!pinBuf.length) pinDisplay.textContent = '••••';
        else {
          const masked = '•'.repeat(pinBuf.length);
          const empty = '·'.repeat(Math.max(0, 4 - pinBuf.length));
          // показываем 4 точки минимум, до 8
          pinDisplay.textContent = masked + (pinBuf.length < 4 ? empty.slice(pinBuf.length) : '');
          // если >4, показываем все
          if (pinBuf.length > 4) pinDisplay.textContent = masked;
        }
      }
      if (pinSubmit) pinSubmit.disabled = busy || pinBuf.length < 4;
    }
    function shakeDots() {
      pinDots.classList.remove('shake');
      void pinDots.offsetWidth;
      pinDots.classList.add('shake');
      setTimeout(() => pinDots.classList.remove('shake'), 420);
    }
    function setBusy(b) {
      busy = b;
      if (pinSubmit) pinSubmit.disabled = b || pinBuf.length < 4;
      pinPad.querySelectorAll('.pad-btn').forEach(btn => {
        if (lockState && lockState.blocked) btn.disabled = true;
        else btn.disabled = b;
      });
      if (compartmentBtn) compartmentBtn.disabled = b || !!(lockState && lockState.blocked && !lockState.open);
    }

    function showPinView() {
      pinBuf = '';
      setHidden(pinView, false);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'PIN запрашивается только для открытия кейса';
      if (titleEl) titleEl.textContent = 'Открыть отсек';
      const hint = $('pin-hint');
      if (hint) hint.textContent = 'Введите PIN-код · 4–8 цифр';
      if (pinSubmit) pinSubmit.textContent = 'Открыть отсек';
      if (pinCancel) pinCancel.textContent = 'Назад к функциям';
      setMsg('', '');
      updatePinDisplay();
    }
    function showCtrlView() {
      setHidden(pinView, true);
      setHidden(ctrlView, false);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (subtitle) subtitle.textContent = 'функции доступны без PIN · PIN нужен только для открытия кейса';
      if (titleEl) titleEl.textContent = 'Функции';
    }

    function openOverlay() {
      pinBuf = '';
      showCtrlView();
      setMsg('', '');
      overlay.hidden = false;
      overlay.setAttribute('aria-hidden', 'false');
      document.body.style.overflow = 'hidden';
      if (sheet) sheet.focus();
      refreshLockStatus();
      vibrate(10);
    }

    function closeOverlay() {
      void stopDrive();
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(pinView, true);
      setHidden(resetView, true);
      setHidden(themesView, true);
      setMsg('', '');
      if (openBtn) openBtn.focus();
    }

    function dismissOverlayForSteering() {
      void stopDrive();
      overlay.hidden = true;
      overlay.setAttribute('aria-hidden', 'true');
      document.body.style.overflow = '';
      pinBuf = '';
      if (lockTimer) { clearInterval(lockTimer); lockTimer = null; }
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, true);
      if (window._showSteeringPage) window._showSteeringPage();
    }

    // Старое имя оставлено для совместимости с обработчиками: авторизация для
    // управления больше не требуется, поэтому сразу открываем экран руления.
    window._openSteeringAuth = () => {
      if (window._showSteeringPage) window._showSteeringPage();
      else openOverlay();
    };

    // ---------- lock API ----------
    async function fetchLock() {
      try {
        const r = await fetch('api/kiosk_lock', { cache: 'no-store' });
        const j = await r.json();
        if (j && j.ok && j.lock) {
          setSteeringAuthorized(!!j.steeringAuthorized);
          return j.lock;
        }
      } catch (e) {}
      return null;
    }
    async function refreshLockStatus() {
      const lock = await fetchLock();
      if (lock) {
        lockState = lock;
        renderLock(lock);
        if (lock.blocked) startBlockTimer(lock.remainingMs);
        else stopBlockTimer();
      }
    }
    function renderLock(lock) {
      lockState = lock;
      const open = !!lock.open;
      const blocked = !!lock.blocked;
      if (lockChip) {
        lockChip.textContent = open ? 'ОТСЕК ОТКРЫТ' : blocked ? 'БЛОКИРОВКА' : 'отсек закрыт';
        lockChip.className = 'ctrl-chip ' + (open ? 'open' : blocked ? 'blocked' : 'closed');
      }
      if (lockDetail) {
        if (open) lockDetail.textContent = blocked
          ? 'закрыть можно без PIN'
          : 'закрой после разгрузки';
        else if (blocked) lockDetail.textContent = 'пауза ' + Math.ceil(lock.remainingMs / 1000) + ' с';
        else lockDetail.textContent = 'готов к открытию';
      }
      if (attemptsEl) {
        if (blocked) attemptsEl.textContent = '';
        else if (lock.attemptsLeft != null && lock.attemptsLeft < 5) attemptsEl.textContent = 'попыток: ' + lock.attemptsLeft;
        else attemptsEl.textContent = '';
      }
      if (compartmentBtn && compartmentTitle && compartmentHint) {
        if (open) {
          compartmentBtn.disabled = false;
          compartmentTitle.textContent = 'Закрыть отсек';
          compartmentHint.textContent = blocked ? 'закрыть можно без PIN' : 'нажми чтобы запереть';
          compartmentBtn.classList.add('opened');
        } else if (blocked) {
          compartmentBtn.disabled = true;
          compartmentTitle.textContent = 'Заблокировано';
          compartmentHint.textContent = 'ещё ' + Math.ceil(lock.remainingMs / 1000) + ' с';
          compartmentBtn.classList.remove('opened');
        } else {
          compartmentBtn.disabled = false;
          compartmentTitle.textContent = 'Открыть отсек';
          compartmentHint.textContent = 'грузовой отсек закрыт';
          compartmentBtn.classList.remove('opened');
        }
      }
      setBusy(false);
      // если блокировка — дизаблим пад
      if (blocked) {
        pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = true);
        if (pinSubmit) pinSubmit.disabled = true;
        setMsg('Слишком много попыток. Подожди ' + Math.ceil(lock.remainingMs/1000) + ' с', 'err');
      } else {
        pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = busy);
        updatePinDisplay();
      }
    }
    let blockInterval = null;
    function startBlockTimer(ms) {
      stopBlockTimer();
      let rem = ms;
      blockInterval = setInterval(() => {
        rem -= 250;
        if (rem <= 0) {
          stopBlockTimer();
          refreshLockStatus();
          setMsg('Блокировка снята — вводи PIN заново', 'ok');
          pinPad.querySelectorAll('.pad-btn').forEach(b => b.disabled = false);
          updatePinDisplay();
        } else {
          const remaining = Math.ceil(rem / 1000);
          if (lockState && lockState.open) {
            if (lockChip) lockChip.textContent = 'ОТСЕК ОТКРЫТ';
            if (lockDetail) lockDetail.textContent = 'закрыть можно без PIN';
            if (compartmentTitle) compartmentTitle.textContent = 'Закрыть отсек';
            if (compartmentHint) compartmentHint.textContent = 'закрыть можно без PIN';
            if (compartmentBtn) compartmentBtn.disabled = false;
            setMsg('PIN временно заблокирован · кейс можно закрыть без PIN', 'warn');
          } else {
            if (lockChip) lockChip.textContent = 'БЛОКИРОВКА';
            if (lockDetail) lockDetail.textContent = 'пауза ' + remaining + ' с';
            if (compartmentTitle) compartmentTitle.textContent = 'Заблокировано';
            if (compartmentHint) compartmentHint.textContent = 'ещё ' + remaining + ' с';
            setMsg('Заблокировано. Ещё ' + remaining + ' с', 'err');
          }
        }
      }, 250);
    }
    function stopBlockTimer() { if (blockInterval) { clearInterval(blockInterval); blockInterval = null; } }

    // ---------- авто-сканирование ----------
    function updateAutoscanUI(on) {
      autoScanOn = !!on;
      try { localStorage.setItem('kiosk_autoscan', autoScanOn ? '1' : '0'); } catch(e) {}
      if (autoscanBtn) {
        autoscanBtn.setAttribute('aria-pressed', autoScanOn ? 'true' : 'false');
        autoscanBtn.classList.toggle('is-on', autoScanOn);
      }
      if (autoscanHint) autoscanHint.textContent = autoScanOn ? 'включено — лидар сканирует' : 'выкл — нажми чтобы включить';
      if (autoscanState) autoscanState.textContent = autoScanOn ? '●' : '○';
      if (scanBeam) scanBeam.hidden = !autoScanOn;
      if (scanLabel) scanLabel.hidden = !autoScanOn;
      if (lidarSection) lidarSection.classList.toggle('is-scanning', autoScanOn);
    }
    function toggleAutoscan() {
      updateAutoscanUI(!autoScanOn);
      vibrate(autoScanOn ? [20,30,20] : 12);
      setMsg(autoScanOn ? 'Авто-сканирование включено' : 'Авто-сканирование выключено', autoScanOn ? 'ok' : 'warn');
      setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Авто-')) setMsg('', ''); }, 1600);
    }

    // ========================= ТЕМЫ: 20 видов =========================
    const THEMES = [
      {id:'red',      name:'Чёрно-красный', hint:'базовый',            accent:'#c0505a', bg:'#000000', panel:'#101012'},
      {id:'rus',      name:'RUS SLAM',      hint:'оригинал лайм',      accent:'#9fe36b', bg:'#070d0a', panel:'#12201a'},
      {id:'lavender', name:'Lavender',      hint:'WEB TACTICS',        accent:'#c9b3ff', bg:'#0a0a14', panel:'#181826'},
      {id:'ocean',    name:'Ocean',         hint:'голубой',            accent:'#4fc3f7', bg:'#061018', panel:'#122636'},
      {id:'sunset',   name:'Sunset',        hint:'оранжевый',          accent:'#ff8c42', bg:'#120a06', panel:'#241a0f'},
      {id:'crimson',  name:'Crimson',       hint:'малиновый',          accent:'#ff4d6d', bg:'#120607', panel:'#241214'},
      {id:'nord',     name:'Nord',          hint:'скандинавский',      accent:'#88c0d0', bg:'#0b0f14', panel:'#15202e'},
      {id:'dracula',  name:'Dracula',       hint:'фиолетовый',         accent:'#bd93f9', bg:'#0e0e14', panel:'#1e1e2e'},
      {id:'light',    name:'Светлая',       hint:'инверсия',           accent:'#0a0a0a', bg:'#f4f4f6', panel:'#ffffff'},
      {id:'cyber',    name:'Cyber',         hint:'неон циан',          accent:'#00ffcc', bg:'#050507', panel:'#10181e'},
      {id:'forest',   name:'Forest',        hint:'лес',                accent:'#7ddc52', bg:'#0a120a', panel:'#142814'},
      {id:'amber',    name:'Amber',         hint:'янтарь',             accent:'#ffb84d', bg:'#120e06', panel:'#241d0f'},
      {id:'teal',     name:'Teal',          hint:'бирюза',             accent:'#2ec4b6', bg:'#061214', panel:'#0f2a26'},
      {id:'violet',   name:'Violet',        hint:'фиолет',             accent:'#a78bfa', bg:'#0f0a14', panel:'#22182e'},
      {id:'midnight', name:'Midnight',      hint:'ночной синий',       accent:'#60a5fa', bg:'#060a14', panel:'#0f1a2e'},
      {id:'sand',     name:'Sand',          hint:'песочный',           accent:'#e6c07a', bg:'#12120a', panel:'#242414'},
      {id:'graphite', name:'Graphite',      hint:'графит',             accent:'#d0d0d4', bg:'#0e0e0e', panel:'#1a1a1a'},
      {id:'mint',     name:'Mint',          hint:'мята',               accent:'#6bd39a', bg:'#0a1412', panel:'#142722'},
      {id:'peach',    name:'Peach',         hint:'персик',             accent:'#ffb088', bg:'#14100a', panel:'#2a2018'},
      {id:'ice',      name:'Ice',           hint:'лёд',                accent:'#7dd3e0', bg:'#0a1418', panel:'#14272e'},
    ];
    // Чёрно-красный — базовое оформление экрана, как было до «20 тем».
    // Сохранённое старшее значение 'bw' (бывшая тема по умолчанию) считаем
    // этим же базовым оформлением.
    let currentTheme = 'red';
    try {
      const saved = localStorage.getItem('kiosk_theme');
      if (saved && saved !== 'bw') currentTheme = saved;
    } catch(e) {}
    function applyTheme(id) {
      if (id === 'bw') id = 'red';
      const t = THEMES.find(x => x.id === id);
      if (!t) return;
      currentTheme = id;
      document.body.setAttribute('data-theme', id);
      try { localStorage.setItem('kiosk_theme', id); } catch(e) {}
      // подсветка активной карточки в сетке
      if (themesGrid) {
        themesGrid.querySelectorAll('.theme-card').forEach(c => {
          c.classList.toggle('is-active', c.getAttribute('data-theme') === id);
        });
      }
    }
    function renderThemes() {
      if (!themesGrid) return;
      themesGrid.innerHTML = '';
      THEMES.forEach(th => {
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'theme-card' + (th.id === currentTheme ? ' is-active' : '');
        btn.setAttribute('data-theme', th.id);
        btn.setAttribute('role', 'listitem');
        btn.setAttribute('aria-label', th.name);
        btn.innerHTML = '<div class="theme-preview" style="--preview-bg:' + th.bg + ';--preview-panel:' + th.panel + ';--preview-accent:' + th.accent + '"><span class="theme-dot"></span></div><b>' + th.name + '</b><small>' + th.hint + '</small>';
        btn.addEventListener('click', () => {
          applyTheme(th.id);
          vibrate(14);
          setMsg('Тема: ' + th.name, 'ok');
          setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Тема:')) setMsg('', ''); }, 1200);
        });
        themesGrid.appendChild(btn);
      });
    }
    function showThemes() {
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, false);
      if (subtitle) subtitle.textContent = 'выбери тему — 20 видов';
      if (titleEl) titleEl.textContent = 'Темы';
      renderThemes();
      if (themesView) themesView.scrollTop = 0;
    }
    function hideThemesToCtrl() {
      setHidden(themesView, true);
      setHidden(ctrlView, false);
      setHidden(pinView, true);
      if (subtitle) subtitle.textContent = 'доступ открыт · выбери действие';
      if (titleEl) titleEl.textContent = 'Функции';
    }
    function showReset() {
      setHidden(pinView, true);
      setHidden(ctrlView, true);
      setHidden(themesView, true);
      setHidden(resetView, false);
      if (subtitle) subtitle.textContent = 'сброс карты — подтверждение';
      if (titleEl) titleEl.textContent = 'Сброс карты';
      if (resetConfirmCheck) resetConfirmCheck.checked = false;
      if (resetConfirm) resetConfirm.disabled = true;
      if (resetMsg) { resetMsg.textContent = ''; resetMsg.className = 'reset-msg'; }
      if (resetView) resetView.scrollTop = 0;
      vibrate(10);
    }
    function hideResetToCtrl() {
      setHidden(resetView, true);
      setHidden(ctrlView, false);
      setHidden(pinView, true);
      setHidden(themesView, true);
      if (subtitle) subtitle.textContent = 'доступ открыт · выбери действие';
      if (titleEl) titleEl.textContent = 'Функции';
    }
    async function doResetMap() {
      if (!resetConfirmCheck || !resetConfirmCheck.checked) {
        if (resetMsg) { resetMsg.textContent = 'Поставь галочку подтверждения'; resetMsg.className = 'reset-msg err'; }
        vibrate([30,30,30]);
        return;
      }
      if (resetConfirm) { resetConfirm.disabled = true; resetConfirm.textContent = 'Сбрасываю…'; }
      try {
        // Сброс карты: сервер чистит сетку и перезаписывает файл карты на
        // рабочем столе (файл не удаляется — обновляется). Старый адрес
        // api/map/reset больше не существует, поэтому зовём api/kiosk_map/reset.
        let resetOk = false;
        let resetError = '';
        try {
          const response = await fetch('api/kiosk_map/reset', { method: 'POST' });
          const body = await response.json().catch(() => null);
          resetOk = !!(response.ok && body && body.ok);
          if (!resetOk) resetError = (body && body.error) || ('HTTP ' + response.status);
        } catch(e) { resetError = 'нет связи с сервером карты'; }
        try { localStorage.removeItem('lidar_map'); localStorage.removeItem('slam_grid'); localStorage.removeItem('kiosk_lidar'); } catch(e) {}
        try {
          if (window.grid && window.grid.fill) { window.grid.fill(0); if (window.explored !== undefined) window.explored = 0; if (window.distDirty !== undefined) window.distDirty = true; }
          if (window._lidarGrid && window._lidarGrid.fill) window._lidarGrid.fill(0);
        } catch(e) {}
        const lidarCanvas = document.getElementById('lidar-canvas');
        if (lidarCanvas && window.RCLidarView) window.RCLidarView.clear(lidarCanvas);
        if (lidarCanvas) {
          const ctx = lidarCanvas.getContext('2d');
          if (ctx) {
            const dpr = window.devicePixelRatio || 1;
            const w = lidarCanvas.clientWidth || 300, h = lidarCanvas.clientHeight || 300;
            if (lidarCanvas.width !== Math.round(w*dpr) || lidarCanvas.height !== Math.round(h*dpr)) {
              lidarCanvas.width = Math.round(w*dpr); lidarCanvas.height = Math.round(h*dpr);
            }
            ctx.setTransform(dpr,0,0,dpr,0,0);
            ctx.clearRect(0,0,w,h);
            ctx.fillStyle = '#101012';
            ctx.fillRect(0,0,w,h);
            ctx.fillStyle = '#8b8b93';
            ctx.font = '12px Inter, sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('карта сброшена', w/2, h/2);
            ctx.strokeStyle = '#2a2428';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.arc(w/2, h/2, Math.min(w,h)/2 - 10, 0, Math.PI*2); ctx.stroke();
          }
        }
        const countEl = document.getElementById('lidar-count');
        const metaEl = document.getElementById('lidar-meta');
        if (countEl) countEl.textContent = '—';
        if (metaEl) metaEl.textContent = 'карта сброшена';
        const emptyEl = document.getElementById('lidar-empty');
        if (emptyEl) { emptyEl.textContent = 'карта сброшена — ждём новый скан'; emptyEl.hidden = false; }
        if (resetMsg) {
          resetMsg.textContent = resetOk
            ? 'Карта лидара сброшена, файл карты обновлён'
            : ('Не удалось сбросить карту: ' + (resetError || 'ошибка сервера'));
          resetMsg.className = 'reset-msg ' + (resetOk ? 'ok' : 'err');
        }
        vibrate([20,40,20]);
        setTimeout(() => {
          hideResetToCtrl();
          setMsg(resetOk ? 'Карта лидара сброшена' : ('Сброс не удался: ' + (resetError || 'ошибка')),
                resetOk ? 'ok' : 'err');
          setTimeout(() => { if (pinMsg && pinMsg.textContent.includes('Карта')) setMsg('', ''); }, 1800);
        }, 900);
      } catch (e) {
        if (resetMsg) { resetMsg.textContent = 'Ошибка сброса'; resetMsg.className = 'reset-msg err'; }
        vibrate([40,80,40]);
      } finally {
        if (resetConfirm) { resetConfirm.disabled = false; resetConfirm.textContent = 'Да, сбросить'; }
      }
    }
    // применить сохранённую тему сразу (и на статистике без PIN)
    applyTheme(currentTheme);

    // ---------- pad handling (сюда водить пальцами) ----------
    function pushDigit(d) {
      if (lockState && lockState.blocked) return;
      if (pinBuf.length >= 8) { shakeDots(); vibrate([30,40,30]); return; }
      pinBuf += d;
      updatePinDisplay();
      setMsg('', '');
      vibrate(12);
      // haptic + visual
      const btn = pinPad.querySelector('[data-k="' + d + '"]');
      if (btn) { btn.classList.add('is-pressed'); setTimeout(() => btn.classList.remove('is-pressed'), 120); }
    }
    function backspace() {
      if (!pinBuf.length) return;
      pinBuf = pinBuf.slice(0, -1);
      updatePinDisplay();
      vibrate(10);
    }
    function clearPin() {
      pinBuf = '';
      updatePinDisplay();
      setMsg('', '');
      vibrate(10);
    }
    function handleKey(k) {
      if (k >= '0' && k <= '9') pushDigit(k);
      else if (k === 'C' || k === 'c' || k === 'С' || k === 'с') clearPin();
      else if (k === '⌫' || k === 'Backspace') backspace();
    }

    async function submitPin() {
      if (busy) return;
      if (pinBuf.length < 4) { setMsg('PIN — 4…8 цифр', 'warn'); shakeDots(); vibrate([20,30,20]); return; }
      if (lockState && lockState.blocked) { setMsg('Подожди — ввод заблокирован', 'err'); return; }
      setBusy(true);
      setMsg('Проверяю PIN и открываю отсек…', 'warn');
      try {
        const response = await fetch('api/kiosk_lock/open', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ pin: pinBuf })
        });
        const result = await response.json();
        if (result && result.lock) renderLock(result.lock);
        else await refreshLockStatus();
        if (response.ok && result && result.ok) {
          pinBuf = '';
          updatePinDisplay();
          setMsg('', '');
          if (lockDetail) lockDetail.textContent = 'отсек открыт — забери груз';
          vibrate([20, 40, 20]);
          showCtrlView();
          return;
        }

        const reason = result && result.reason;
        if (reason === 'blocked' || (result && result.lock && result.lock.blocked)) {
          shakeDots(); vibrate([40,80,40,80,80]);
          setMsg('Заблокировано на ' + Math.ceil(((result && result.remainingMs) ||
            (lockState && lockState.remainingMs) || 30000) / 1000) + ' с', 'err');
        } else if (reason === 'wrong') {
          shakeDots(); vibrate([30,30,30]);
          setMsg('Неверный PIN. Осталось попыток: ' + (result.attemptsLeft != null ? result.attemptsLeft : '?'), 'err');
        } else if (reason === 'format') {
          shakeDots(); setMsg('PIN — только цифры, 4…8', 'warn');
        } else {
          shakeDots(); setMsg((result && result.error) || 'Не удалось открыть отсек', 'err');
        }
        setBusy(false);
        updatePinDisplay();
      } catch (error) {
        setMsg('Нет связи с роботом', 'err');
        setBusy(false);
      }
    }

    async function toggleCompartment() {
      if (busy) return;
      const open = !!(lockState && lockState.open);
      if (!open && lockState && lockState.blocked) {
        setMsg('Отсек заблокирован', 'err');
        return;
      }
      if (!open) {
        // PIN спрашиваем именно при открытии кейса, а не при входе в функции.
        showPinView();
        return;
      }
      setBusy(true);
      setMsg('Запираю…', 'warn');
      try {
        const response = await fetch('api/kiosk_lock/close', {
          method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}'
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) throw new Error((result && result.error) || 'Не удалось закрыть отсек.');
        if (result.lock) renderLock(result.lock);
        else await refreshLockStatus();
        vibrate(20);
        if (lockDetail) lockDetail.textContent = 'отсек заперт';
        setMsg('Отсек закрыт', 'ok');
      } catch (error) {
        setMsg(error.message || 'Нет связи', 'err');
      } finally {
        setBusy(false);
      }
    }

    // ---------- events ----------
    openBtn.addEventListener('click', openOverlay);
    if (backdrop) backdrop.addEventListener('click', closeOverlay);
    if (closeBtn) closeBtn.addEventListener('click', closeOverlay);
    if (pinCancel) pinCancel.addEventListener('click', () => {
      pinBuf = '';
      setMsg('', '');
      showCtrlView();
    });
    if (ctrlBack) ctrlBack.addEventListener('click', closeOverlay);
    if (ctrlLogout) ctrlLogout.addEventListener('click', () => {
      // запереть если открыто и выйти
      if (lockState && lockState.open) {
        fetch('api/kiosk_lock/close', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
          .finally(closeOverlay);
      } else closeOverlay();
    });

    // === пад: тап + вести пальцем — сюда водить пальцами ===
    // pointerdown сразу вводит цифру (тап), а drag вводит следующие
    let pointerHandled = false;
    function handlePadAction(k) {
      if (k === 'C') clearPin();
      else if (k === '⌫') backspace();
      else if (/^[0-9]$/.test(k)) pushDigit(k);
    }
    // клик-фолбек для старых браузеров (если pointer не сработал)
    pinPad.addEventListener('click', (e) => {
      if (pointerHandled) { pointerHandled = false; return; }
      const btn = e.target.closest('.pad-btn');
      if (!btn || btn.disabled) return;
      const k = btn.getAttribute('data-k');
      handlePadAction(k);
    });

    pinPad.addEventListener('pointerdown', (e) => {
      const btn = e.target.closest('.pad-btn');
      if (!btn || btn.disabled) return;
      dragging = true;
      lastHoveredBtn = btn;
      pointerHandled = true;
      // сразу вводим цифру под пальцем — тап без движения уже работает
      handlePadAction(btn.getAttribute('data-k'));
      // визуал
      btn.classList.add('is-pressed');
      setTimeout(() => btn.classList.remove('is-pressed'), 140);
      try { pinPad.setPointerCapture(e.pointerId); } catch (ex) {}
      // флаг сбросится в click-хендлере или таймаутом, чтобы следующий тап не игнорил
      setTimeout(() => { pointerHandled = false; }, 600);
      e.preventDefault();
    });
    pinPad.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      const el = document.elementFromPoint(e.clientX, e.clientY);
      const btn = el ? el.closest('.pad-btn') : null;
      if (btn && btn !== lastHoveredBtn && !btn.disabled) {
        lastHoveredBtn = btn;
        const k = btn.getAttribute('data-k');
        // чтобы не наспамить удержанием на одной кнопке — только при смене кнопки
        if (/^[0-9]$/.test(k)) {
          pushDigit(k);
          btn.classList.add('is-pressed');
          setTimeout(() => btn.classList.remove('is-pressed'), 120);
        } else if (k === '⌫') backspace();
        else if (k === 'C') clearPin();
      }
    });
    function stopDrag(e) {
      if (dragging) {
        dragging = false;
        lastHoveredBtn = null;
        try { pinPad.releasePointerCapture(e.pointerId); } catch(e2){}
      }
    }
    pinPad.addEventListener('pointerup', stopDrag);
    pinPad.addEventListener('pointercancel', stopDrag);
    pinPad.addEventListener('pointerleave', (e) => {
      // не сбрасываем сразу — вдруг вернётся
    });

    if (pinSubmit) pinSubmit.addEventListener('click', submitPin);
    if (btnSteeringControls) btnSteeringControls.addEventListener('click', dismissOverlayForSteering);
    if (compartmentBtn) compartmentBtn.addEventListener('click', toggleCompartment);
    if (autoscanBtn) autoscanBtn.addEventListener('click', toggleAutoscan);
    if (btnThemes) btnThemes.addEventListener('click', showThemes);
    if (themesBack) themesBack.addEventListener('click', hideThemesToCtrl);
    if (themesToStats) themesToStats.addEventListener('click', closeOverlay);
    if (btnResetMap) btnResetMap.addEventListener('click', showReset);
    if (btnMapView) {
      btnMapView.addEventListener('click', () => {
        // Карта лидара — страница внутри киоска: открываем её только кнопкой.
        closeOverlay();
        if (window._openSwipe) window._openSwipe('map');
      });
    }
    if (resetBack) resetBack.addEventListener('click', hideResetToCtrl);
    if (resetCancel) resetCancel.addEventListener('click', hideResetToCtrl);
    if (resetConfirmCheck) resetConfirmCheck.addEventListener('change', () => {
      if (resetConfirm) resetConfirm.disabled = !resetConfirmCheck.checked;
    });
    if (resetConfirm) resetConfirm.addEventListener('click', doResetMap);

    // keyboard
    document.addEventListener('keydown', (e) => {
      if (overlay.hidden) {
        // быстрый вызов по цифрам? не, только когда открыто — PIN клавиатура
        if (e.key === 'Enter' && document.activeElement === openBtn) openOverlay();
        return;
      }
      // когда оверлей открыт
      if (e.key === 'Escape') {
        e.preventDefault();
        if (resetView && !resetView.hidden) { hideResetToCtrl(); return; }
        if (themesView && !themesView.hidden) { hideThemesToCtrl(); return; }
        closeOverlay(); return;
      }
      if (resetView && !resetView.hidden) {
        if (e.key === 'Enter' && resetConfirm && !resetConfirm.disabled) { e.preventDefault(); doResetMap(); }
        return;
      }
      if (themesView && !themesView.hidden) {
        // в выборе темы — Esc уже обработан, остальные клавиши не трогаем
        return;
      }
      // если в pinView
      if (!pinView.hidden) {
        if (e.key >= '0' && e.key <= '9') { e.preventDefault(); pushDigit(e.key); }
        else if (e.key === 'Backspace') { e.preventDefault(); backspace(); }
        else if (e.key.toLowerCase() === 'c' || e.key === 'Delete') { e.preventDefault(); clearPin(); }
        else if (e.key === 'Enter') { e.preventDefault(); submitPin(); }
      } else {
        // в ctrlView — Enter на отсеке
        if (e.key === 'Enter') {
          e.preventDefault();
          toggleCompartment();
        }
      }
    });

    // init state
    updatePinDisplay();
    refreshLockStatus();
    updateAutoscanUI(autoScanOn);
    // Обновляем состояние замка кейса, даже когда панель функций закрыта.
    setInterval(refreshLockStatus, 1500);
  }

  /* ============ СВАЙП-СТРАНИЦА: поворот модулей + камеры/логи ============ */
  function initSwipePage() {
    const page = $('swipe-page');
    const closeBtn = $('swipe-close');
    const cameraView = $('swipe-cameras-view');
    const motorView = $('swipe-motors-view');
    const batteryView = $('swipe-battery-view');
    const swipeTitle = $('swipe-title-text');
    const swipeTitleHint = $('swipe-title-hint');
    const kiosk = document.querySelector('.kiosk');
    const hintBar = $('swipe-hint-bar');
    const logList = $('log-list');
    const logCount = $('log-count');
    const logStatus = $('log-status');
    const logClear = $('log-clear');
    const logCopy = $('log-copy');
    const logAuto = $('log-autoscroll');
    const camGearBtn = $('cam-settings-open');
    const mapView = $('swipe-map-view');
    if (!page || !kiosk) return;

    let isOpen = false;
    let activeSwipeView = 'cameras';
    let activeSwipeDirection = 'left';
    let camLoopsStarted = false;
    let mapRefreshPending = false;

    const sheet = page.querySelector('.swipe-sheet');
    const backdrop = $('swipe-backdrop');

    function selectSwipeView(name) {
      const showMotors = name === 'motors';
      const showBattery = name === 'battery';
      const showMap = name === 'map';
      if (activeSwipeView === 'motors' && !showMotors) void stopDrive();
      activeSwipeView = showMotors ? 'motors'
        : (showBattery ? 'battery' : (showMap ? 'map' : 'cameras'));
      // Карта открывается кнопкой справа, как руление; свайпом на неё не попасть.
      activeSwipeDirection = (showMotors || showMap) ? 'right' : 'left';
      page.dataset.openDirection = activeSwipeDirection;
      if (cameraView) cameraView.hidden = showMotors || showBattery || showMap;
      if (motorView) motorView.hidden = !showMotors;
      if (batteryView) batteryView.hidden = !showBattery;
      if (mapView) mapView.hidden = !showMap;
      // Маленькая кнопка ⚙ настройки портов камер — только на вкладке камер.
      if (camGearBtn) camGearBtn.hidden = activeSwipeView !== 'cameras';
      if (swipeTitle) {
        swipeTitle.textContent = showMotors ? 'Руление и тяга'
          : (showBattery ? '4 · Аккумулятор'
            : (showMap ? 'Карта лидара' : 'Камеры и логи'));
      }
      if (swipeTitleHint) {
        swipeTitleHint.textContent = showBattery
          ? 'Все банки · свайп вправо — назад к статистике'
          : (showMotors ? 'свайп влево — назад к статистике'
            : (showMap ? 'карта строится лидаром · закрыть кнопкой «← Назад»'
              : 'камеры и журнал · свайп вправо — назад к статистике'));
      }
      if (!showBattery) setText($('swipe-live'), '● LIVE');
      if (window.RCCameraView && activeSwipeView !== 'cameras') window.RCCameraView.setActive(false);
      if (isOpen && activeSwipeView === 'cameras') { startCamLoops(); fetchLogs(); }
      if (showMap && isOpen && !page.hidden) refreshMapPage();
    }

    function refreshMapPage() {
      if (mapRefreshPending) return;
      mapRefreshPending = true;
      // Never let rendering a still-hidden canvas abort page navigation.
      // Wait for layout, then fetch immediately instead of waiting for polling.
      requestAnimationFrame(() => {
        mapRefreshPending = false;
        if (!isOpen || activeSwipeView !== 'map') return;
        try {
          if (window.RCMapView) {
            if (window.RCMapView.show) window.RCMapView.show();
            else window.RCMapView.draw();
          } else {
            setText($('map-msg'), 'Не загружен модуль карты. Откройте карту кнопкой в шапке.');
          }
        } catch (err) {
          setText($('map-msg'), 'Не удалось нарисовать карту: ' + (err.message || err));
        }
      });
    }

    function openSwipe(view) {
      if (isOpen) {
        if (view) selectSwipeView(view);
        return;
      }
      const overlay = $('ctrl-overlay');
      if (overlay && !overlay.hidden) return;
      isOpen = true;
      selectSwipeView(view || 'motors');
      page.hidden = false;
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      void page.offsetWidth;
      page.classList.add('is-open');
      document.body.classList.add('swipe-open');
      page.setAttribute('aria-hidden', 'false');
      if (hintBar) hintBar.style.opacity = '0';
      if (activeSwipeView === 'map') refreshMapPage();
    }
    function closeSwipe() {
      if (!isOpen) return;
      isOpen = false;
      if (window.RCCameraView) window.RCCameraView.setActive(false);
      if (activeSwipeView === 'motors') void stopDrive();
      page.classList.remove('is-open');
      document.body.classList.remove('swipe-open');
      page.setAttribute('aria-hidden', 'true');
      if (hintBar) hintBar.style.opacity = '';
      if (sheet) sheet.style.transform = '';
      if (backdrop) backdrop.style.opacity = '';
      setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
    }
    window._openSwipe = (view) => openSwipe(view || 'motors');
    window._closeSwipe = closeSwipe;
    window._showSteeringPage = () => {
      if (!isOpen) openSwipe('motors');
      else selectSwipeView('motors');
    };
    selectSwipeView('cameras');
    const camerasOpen = $('k-cameras-open');
    if (camerasOpen) camerasOpen.addEventListener('click', () => openSwipe('cameras'));
    {
      const button = $('swipe-battery-open');
      if (button) button.addEventListener('click', () => openSwipe('battery'));
    }
    // Вся панель АКБ на главном экране открывает страницу 4 — её не нужно искать.
    const batteryPanel = $('battery-panel');
    if (batteryPanel) {
      // Свайп, начатый на панели, — это жест страниц, а не нажатие.
      let panelDown = null;
      batteryPanel.addEventListener('pointerdown', (event) => {
        panelDown = { x: event.clientX, y: event.clientY };
      });
      batteryPanel.addEventListener('click', (event) => {
        const moved = panelDown && Number.isFinite(event.clientX) &&
          (Math.abs(event.clientX - panelDown.x) > 12 || Math.abs(event.clientY - panelDown.y) > 12);
        panelDown = null;
        if (!moved) openSwipe('battery');
      });
      batteryPanel.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          openSwipe('battery');
        }
      });
    }

    if (closeBtn) closeBtn.addEventListener('click', closeSwipe);
    if (backdrop) backdrop.addEventListener('click', closeSwipe);
    // клик по листу вне? — закрывать только по бэкдропу

    // запрет выделения мышью — чтобы тяга не красила текст, кроме логов
    kiosk.addEventListener('selectstart', (e) => e.preventDefault());
    kiosk.addEventListener('dragstart', (e) => e.preventDefault());
    page.addEventListener('selectstart', (e) => {
      if (e.target.closest && e.target.closest('.log-list')) return;
      e.preventDefault();
    });
    page.addEventListener('dragstart', (e) => e.preventDefault());

    // Страницы свайп-экрана. Слева от статистики — «камеры и логи», справа —
    // «руление и тяга». Между собой страницы больше не перелистываются:
    // карта лидара открывается только кнопкой («Карта» в шапке или
    // «Функции → Карта лидара»), свайпом на неё не попасть.
    const SWIPE_NEXT = {
      cameras: { right: null, left: null },
      map: { right: null, left: null },
      motors: { right: null, left: null },
      battery: { right: null, left: null },
    };

    let tStartX = 0, tStartY = 0, tTracking = false, dragging = false, dragType = null;
    let tDragView = null;
    let tDragFrom = null;
    let tDragDir = 'right';

    function directionForView(view) {
      return (view === 'motors' || view === 'map') ? 'right' : 'left';
    }
    function signedSwipeDistance(dx, direction) {
      return direction === 'right' ? dx : -dx;
    }
    function gestureDirection(dx) {
      return dx > 0 ? 'right' : 'left';
    }
    // Что делает свайп в сторону dir на странице view: соседних страниц нет,
    // поэтому любой горизонтальный жест просто закрывает открытую страницу.
    function gestureFor(view, dir) {
      const table = SWIPE_NEXT[view] || { right: null, left: null };
      const next = table[dir];
      if (next) return { type: 'next', view: next };
      return { type: 'close', view: view };
    }
    function swipeProgress(dx, vw, type, direction) {
      const distance = vw * 0.52;
      const signed = signedSwipeDistance(dx, direction);
      return Math.max(0, Math.min(1, type === 'open' ? signed / distance : 1 - signed / distance));
    }
    function setPageProgress(px, direction, type) {
      const progress = Math.max(0, Math.min(1, px));
      // Лист едет за пальцем: при открытии — навстречу жесту, при закрытии и
      // переходе на соседнюю страницу — по жесту.
      const sign = type === 'open'
        ? (direction === 'right' ? -1 : 1)
        : (direction === 'right' ? 1 : -1);
      const offset = sign * (1 - progress) * 105;
      if (sheet) sheet.style.transform = 'translateX(' + offset + '%)';
      const shown = type === 'next' ? 1 : progress;
      if (backdrop) backdrop.style.opacity = String(shown);
      if (kiosk) {
        kiosk.style.transform = 'scale(' + (1 - shown * 0.015) + ')';
        kiosk.style.filter = 'brightness(' + (1 - shown * 0.08) + ')';
      }
    }
    function beginDrag(direction, keepResting) {
      if (!keepResting) page.dataset.openDirection = direction;
      page.hidden = false;
      page.classList.add('is-open');
      if (sheet) sheet.style.transition = 'none';
      if (backdrop) backdrop.style.transition = 'none';
      if (kiosk) kiosk.style.transition = 'none';
      void page.offsetWidth;
      dragging = true;
    }
    function endDragCleanup() {
      if (sheet) { sheet.style.transition = ''; sheet.style.transform = ''; }
      if (backdrop) { backdrop.style.transition = ''; backdrop.style.opacity = ''; }
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      dragging = false;
      dragType = null;
      tDragView = null;
      tDragFrom = null;
      if (hintBar) hintBar.style.opacity = '';
    }
    function dragCompletes(dx, vw, type, direction) {
      const travelled = signedSwipeDistance(dx, direction);
      return travelled > vw * 0.18 || (Math.abs(dx) > 70 && travelled > 0);
    }
    function finishSwipe(type, view, shouldRemainOpen) {
      if (shouldRemainOpen) {
        if (type === 'open') openSwipe(view);
        else {
          page.hidden = false;
          page.classList.add('is-open');
          document.body.classList.add('swipe-open');
          page.setAttribute('aria-hidden', 'false');
        }
      } else if (type === 'close') {
        closeSwipe();
      } else {
        page.classList.remove('is-open');
        document.body.classList.remove('swipe-open');
        page.setAttribute('aria-hidden', 'true');
        setTimeout(() => { if (!isOpen) page.hidden = true; }, 480);
      }
    }

    function tStart(x, y) {
      if (isOpen && activeSwipeView === 'map') return;
      tStartX = x; tStartY = y; tTracking = true; dragging = false; dragType = null;
      tDragView = null; tDragFrom = null;
    }
    function tMove(x, y) {
      if (!tTracking) return;
      if (isOpen && activeSwipeView === 'map') { tTracking = false; return; }
      const dx = x - tStartX;
      const dy = y - tStartY;
      if (!dragging) {
        if (Math.abs(dx) < 10) return;
        if (Math.abs(dx) < Math.abs(dy) * 1.1) { tTracking = false; return; }
        const overlay = $('ctrl-overlay');
        if (!isOpen && overlay && !overlay.hidden) { tTracking = false; return; }
        const dir = gestureDirection(dx);
        if (!isOpen) {
          dragType = 'open';
          tDragView = dir === 'right' ? 'motors' : 'cameras';
          selectSwipeView(tDragView);
          beginDrag(directionForView(tDragView));
          page.setAttribute('aria-hidden', 'false');
        } else {
          const gesture = gestureFor(activeSwipeView, dir);
          dragType = gesture.type;
          tDragView = gesture.view;
          tDragFrom = activeSwipeView;
          tDragDir = dir;
          if (dragType === 'next') {
            // Соседняя страница: содержимое меняется сразу, лист тянется за пальцем.
            selectSwipeView(tDragView);
            beginDrag(directionForView(tDragView));
          } else {
            beginDrag(activeSwipeDirection, true);
          }
        }
        document.body.classList.add('swipe-open');
      }
      const vw = window.innerWidth || 400;
      const direction = dragType === 'open' ? directionForView(tDragView) : tDragDir;
      const progress = swipeProgress(dx, vw, dragType, direction);
      setPageProgress(progress, direction, dragType);
      if (hintBar) hintBar.style.opacity = String(dragType === 'open' ? 1 - progress : progress);
    }
    function tEnd(x, y) {
      if (!tTracking && !dragging) return;
      const wasDragging = dragging;
      const dx = x - tStartX;
      const dy = y - tStartY;
      const vw = window.innerWidth || 400;
      tTracking = false;
      if (!wasDragging) {
        if (Math.abs(dx) < 60 || Math.abs(dx) < Math.abs(dy) * 1.1) return;
        if (!isOpen) {
          if (dx > 65) openSwipe('motors');
          else if (dx < -65) openSwipe('cameras');
        } else {
          const gesture = gestureFor(activeSwipeView, gestureDirection(dx));
          if (Math.abs(dx) <= 65) return;
          if (gesture.type === 'next') selectSwipeView(gesture.view);
          else closeSwipe();
        }
        return;
      }
      const type = dragType;
      const view = tDragView;
      const from = tDragFrom;
      const direction = type === 'open' ? directionForView(view) : tDragDir;
      const completed = dragCompletes(dx, vw, type, direction);
      endDragCleanup();
      if (type === 'next') {
        // Переход на соседнюю страницу: доводим жест или возвращаемся назад.
        selectSwipeView(completed ? view : from);
        return;
      }
      finishSwipe(type, view, type === 'open' ? completed : !completed);
    }

    // Touch gestures on the base screen and on the open panel.
    kiosk.addEventListener('touchstart', (event) => {
      if (event.touches.length !== 1) return;
      tStart(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    kiosk.addEventListener('touchmove', (event) => {
      if (event.touches.length !== 1) return;
      tMove(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    kiosk.addEventListener('touchend', (event) => {
      const touch = event.changedTouches[0];
      if (touch) tEnd(touch.clientX, touch.clientY);
    }, {passive: true});
    page.addEventListener('touchstart', (event) => {
      if (event.touches.length !== 1) return;
      // Карта открывается и закрывается только кнопками: свайп на ней не
      // перелистывает страницы и не тянет панель.
      if (isOpen && activeSwipeView === 'map') return;
      if (event.target.closest && event.target.closest('input, button, select, textarea, .steer-dial, .cam-viewport, .map-canvas-wrap')) return;
      tStart(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    page.addEventListener('touchmove', (event) => {
      if (event.touches.length !== 1) return;
      tMove(event.touches[0].clientX, event.touches[0].clientY);
    }, {passive: true});
    page.addEventListener('touchend', (event) => {
      const touch = event.changedTouches[0];
      if (touch) tEnd(touch.clientX, touch.clientY);
    }, {passive: true});

    // Mouse drag mirrors touch behavior for desktop previews.
    let mTracking = false, mStartX = 0, mStartY = 0, mDragging = false, mType = null;
    let mView = null, mOrigin = null, mFrom = null, mDir = 'right';
    function mStart(x, y, origin) {
      mStartX = x; mStartY = y; mTracking = true; mDragging = false; mType = null;
      mView = null; mOrigin = origin; mFrom = null;
    }
    kiosk.addEventListener('mousedown', (event) => {
      if (event.button !== 0) return;
      const overlay = $('ctrl-overlay');
      if (overlay && !overlay.hidden) return;
      mStart(event.clientX, event.clientY, 'kiosk');
    });
    page.addEventListener('mousedown', (event) => {
      if (event.button !== 0) return;
      // На странице карты нет свайп-навигации — только кнопки.
      if (isOpen && activeSwipeView === 'map') return;
      if (event.target.closest && event.target.closest('input, button, select, textarea, .steer-dial, .cam-viewport, .map-canvas-wrap')) return;
      mStart(event.clientX, event.clientY, 'page');
    });
    window.addEventListener('mousemove', (event) => {
      if (!mTracking) return;
      if (isOpen && activeSwipeView === 'map') { mTracking = false; mDragging = false; return; }
      const dx = event.clientX - mStartX;
      const dy = event.clientY - mStartY;
      if (!mDragging) {
        if (Math.abs(dx) < 12) return;
        if (Math.abs(dx) < Math.abs(dy) * 1.1) { mTracking = false; return; }
        const overlay = $('ctrl-overlay');
        if (!isOpen && overlay && !overlay.hidden) { mTracking = false; return; }
        const dir = gestureDirection(dx);
        if (!isOpen) {
          mType = 'open';
          mView = dir === 'right' ? 'motors' : 'cameras';
          mDir = dir;
          selectSwipeView(mView);
          page.dataset.openDirection = directionForView(mView);
        } else {
          const gesture = gestureFor(activeSwipeView, dir);
          mType = gesture.type;
          mView = gesture.view;
          mFrom = activeSwipeView;
          mDir = dir;
          if (mType === 'next') {
            selectSwipeView(mView);
            page.dataset.openDirection = directionForView(mView);
          }
        }
        page.hidden = false;
        page.classList.add('is-open');
        if (sheet) sheet.style.transition = 'none';
        if (backdrop) backdrop.style.transition = 'none';
        if (kiosk) kiosk.style.transition = 'none';
        void page.offsetWidth;
        mDragging = true;
        dragging = true;
        dragType = mType;
        document.body.classList.add('swipe-open');
        if (!isOpen) page.setAttribute('aria-hidden', 'false');
      }
      const vw = window.innerWidth || 400;
      const direction = mType === 'open' ? directionForView(mView) : mDir;
      setPageProgress(swipeProgress(dx, vw, mType, direction), direction, mType);
      if (hintBar) hintBar.style.opacity = String(mType === 'open'
        ? 1 - swipeProgress(dx, vw, mType, direction)
        : swipeProgress(dx, vw, mType, direction));
      event.preventDefault();
    });
    window.addEventListener('mouseup', (event) => {
      if (!mTracking && !mDragging) return;
      const wasDrag = mDragging;
      const dx = event.clientX - mStartX;
      const dy = event.clientY - mStartY;
      const vw = window.innerWidth || 400;
      const type = mType;
      const view = mView;
      const from = mFrom;
      const dir = mDir;
      mTracking = false; mDragging = false; dragging = false; dragType = null;
      if (sheet) { sheet.style.transition = ''; sheet.style.transform = ''; }
      if (backdrop) { backdrop.style.transition = ''; backdrop.style.opacity = ''; }
      if (kiosk) { kiosk.style.transition = ''; kiosk.style.transform = ''; kiosk.style.filter = ''; }
      if (hintBar) hintBar.style.opacity = '';
      if (!wasDrag) {
        if (Math.abs(dx) < 80 || Math.abs(dx) < Math.abs(dy) * 1.1) { mOrigin = null; return; }
        if (mOrigin === 'kiosk' && !isOpen) {
          openSwipe(dx > 0 ? 'motors' : 'cameras');
        } else if (mOrigin === 'page' && isOpen) {
          const gesture = gestureFor(activeSwipeView, gestureDirection(dx));
          if (gesture.type === 'next') selectSwipeView(gesture.view);
          else closeSwipe();
        }
        mOrigin = null;
        return;
      }
      const direction = type === 'open' ? directionForView(view) : dir;
      const completed = dragCompletes(dx, vw, type, direction);
      if (type === 'next') {
        selectSwipeView(completed ? view : from);
      } else {
        finishSwipe(type, view, type === 'open' ? completed : !completed);
      }
      mOrigin = null; mView = null; mType = null; mFrom = null;
    });

    // Esc закрывает: сначала окно настройки камер, потом сам свайп-экран
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && isOpen) {
        e.preventDefault();
        if (camSettingsOpen) { closeCamSettingsModal(); return; }
        closeSwipe();
      }
    });

    // также клик по оверлею вне? не нужно

    // --- КАМЕРЫ: кадры бортового сервера, без захвата USB из браузера ---
    // + настройка портов: передняя и задняя камеры — отдельные V4L2-устройства
    let camSettings = null;
    let camPorts = [];
    let camSettingsOpen = false;

    function camMsg(text, tone) {
      const el = $('cam-settings-msg');
      if (!el) return;
      el.textContent = text || '';
      if (el.dataset) el.dataset.tone = tone || '';
    }

    function renderCamSettings() {
      const cur = $('cam-settings-current');
      const selF = $('cam-front-select');
      const selR = $('cam-rear-select');
      const hintF = $('cam-front-hint');
      const hintR = $('cam-rear-hint');
      if (!camSettings) {
        if (cur) cur.textContent = 'загрузка…';
        return;
      }
      const front = (camSettings.front || '').trim();
      const rear = (camSettings.rear || '').trim();
      if (cur) {
        const tf = front || 'авто (/dev/video0)';
        const tr = rear || 'авто (/dev/video1)';
        cur.textContent = 'передняя: ' + tf + ' · задняя: ' + tr;
      }
      // Заполняем селекты только если список портов уже есть
      const ports = Array.isArray(camPorts) ? camPorts : [];
      const opts = [{port:'', label:'Авто (/dev/video0 для передней, /dev/video1 для задней)'}];
      ports.forEach((p) => {
        const exists = p.exists ? '' : ' (нет устройства)';
        opts.push({port: p.port, label: p.port + exists + (p.note ? ' — ' + p.note : '')});
      });
      // Добавить текущие значения, если их нет в списке (кастомный путь)
      [front, rear].forEach((v) => {
        if (v && !opts.some((o) => o.port === v)) opts.push({port: v, label: v + ' (текущий)'});
      });
      function fillSelect(sel, value, fallbackLabel) {
        if (!sel) return;
        const curVal = sel.value;
        const keep = value || '';
        // Пересоздаём только если набор изменился
        const key = JSON.stringify(opts.map(o=>o.port)) + '|' + keep;
        if (sel.dataset.key === key && sel.options.length === opts.length) {
          sel.value = keep;
          return;
        }
        sel.dataset.key = key;
        sel.innerHTML = '';
        opts.forEach((o) => {
          const opt = document.createElement('option');
          opt.value = o.port;
          opt.textContent = o.label;
          sel.appendChild(opt);
        });
        sel.value = keep;
      }
      fillSelect(selF, front, 'передняя');
      fillSelect(selR, rear, 'задняя');
      if (hintF) hintF.textContent = front ? 'выбран: ' + front : 'авто — /dev/video0';
      if (hintR) hintR.textContent = rear ? 'выбран: ' + rear : 'авто — /dev/video1';
    }

    async function fetchCamSettings() {
      try {
        const r = await fetch('api/camera/settings', {cache:'no-store'});
        const j = await r.json();
        if (j && j.ok) {
          camSettings = j.settings || {front:'', rear:''};
          const opts = j.options && j.options.ports ? j.options.ports : (j.ports || []);
          if (Array.isArray(opts) && opts.length) camPorts = opts;
          else if (Array.isArray(j.cameras)) {
            // fallback из /api/cameras — построим список из камер
          }
          renderCamSettings();
        } else {
          camMsg(j && j.error ? j.error : 'не удалось загрузить настройки камер', 'error');
        }
      } catch (e) {
        camMsg('нет связи с /api/camera/settings', 'error');
      }
    }

    async function rescanCamPorts() {
      camMsg('обновляю список видеопортов…', '');
      try {
        const r = await fetch('api/camera/rescan', {method:'POST', cache:'no-store'});
        const j = await r.json();
        if (j && j.ok) {
          camPorts = j.ports || j.options && j.options.ports || camPorts;
          camMsg('список обновлён: найдено ' + (camPorts.length || 0) + ' портов', 'ok');
          renderCamSettings();
          setTimeout(() => camMsg('', ''), 1600);
        } else {
          camMsg(j && j.error ? j.error : 'не удалось обновить список', 'error');
        }
      } catch (e) {
        camMsg('нет связи с сервером', 'error');
      }
    }

    async function saveCamSettings() {
      const selF = $('cam-front-select');
      const selR = $('cam-rear-select');
      const front = selF ? selF.value.trim() : '';
      const rear = selR ? selR.value.trim() : '';
      if (front && rear && front === rear) {
        camMsg('передняя и задняя камеры не могут использовать один порт', 'error');
        return;
      }
      camMsg('сохраняю…', '');
      const btn = $('cam-save');
      if (btn) btn.disabled = true;
      try {
        const r = await fetch('api/camera/settings', {
          method:'POST', headers:{'Content-Type':'application/json'}, cache:'no-store',
          body: JSON.stringify({front, rear})
        });
        const j = await r.json().catch(()=>null);
        if (!r.ok || !j || !j.ok) {
          camMsg(j && j.error ? j.error : 'не удалось сохранить (HTTP ' + r.status + ')', 'error');
          return;
        }
        camSettings = j.settings || {front, rear};
        camMsg('сохранено в .env: передняя ' + (front || 'авто') + ', задняя ' + (rear || 'авто') + ' — камеры переключены', 'ok');
        renderCamSettings();
        // Перезапустить бэкенд-стримы локально без перезагрузки всего окна
        if (window.RCCameraView) {
          window.RCCameraView.setActive(false);
          window.RCCameraView.setActive(isOpen && activeSwipeView === 'cameras');
        }
        setTimeout(() => camMsg('', ''), 2200);
      } catch (e) {
        camMsg('нет связи с сервером', 'error');
      } finally {
        if (btn) btn.disabled = false;
      }
    }

    // Окно настройки портов камер: маленькая кнопка ⚙ в шапке открывает
    // отдельное окно; закрытие — ✕, клик по фону или Esc.
    function openCamSettingsModal() {
      const modal = $('cam-settings-modal');
      if (!modal) return;
      camSettingsOpen = true;
      modal.hidden = false;
      if (camGearBtn) camGearBtn.setAttribute('aria-expanded', 'true');
      void fetchCamSettings();
    }

    function closeCamSettingsModal() {
      const modal = $('cam-settings-modal');
      if (!modal) return;
      camSettingsOpen = false;
      modal.hidden = true;
      if (camGearBtn) camGearBtn.setAttribute('aria-expanded', 'false');
    }

    function bindCamSettingsUI() {
      const btnOpen = $('cam-settings-open');
      const btnClose = $('cam-modal-close');
      const modalBackdrop = $('cam-modal-backdrop');
      const btnScan = $('cam-scan');
      const btnSave = $('cam-save');
      const btnReset = $('cam-reset');
      if (btnOpen) btnOpen.addEventListener('click', openCamSettingsModal);
      if (btnClose) btnClose.addEventListener('click', closeCamSettingsModal);
      if (modalBackdrop) modalBackdrop.addEventListener('click', closeCamSettingsModal);
      if (btnScan) btnScan.addEventListener('click', () => { void rescanCamPorts(); void fetchCamSettings(); });
      if (btnSave) btnSave.addEventListener('click', () => { void saveCamSettings(); });
      if (btnReset) btnReset.addEventListener('click', () => {
        const selF = $('cam-front-select');
        const selR = $('cam-rear-select');
        if (selF) selF.value = '';
        if (selR) selR.value = '';
        void saveCamSettings();
      });
      // При открытии вкладки камер — подгрузить настройки
      const camView = $('swipe-cameras-view');
      if (camView) {
        const mo = new MutationObserver(() => {
          if (!camView.hidden) void fetchCamSettings();
        });
        mo.observe(camView, {attributes:true, attributeFilter:['hidden']});
      }
    }

    function startCamLoops() {
      if (!camLoopsStarted) {
        camLoopsStarted = true;
        bindCamSettingsUI();
        void fetchCamSettings();
      }
      if (window.RCCameraView) window.RCCameraView.setActive(true);
    }

    // --- ЛОГИ: настоящий журнал робота (/api/kiosk_logs) ---
    // Панель показывает события этого процесса: подключение и потеря лидара,
    // ошибки портов, проверки PIN, команды руления и тяги. Выдуманных строк
    // здесь нет: если событий не было, панель так и пишет.
    const KNOWN_LEVELS = ['info', 'ok', 'warn', 'err', 'debug'];
    const LOG_STATUS_BASE = 'журнал робота · /api/kiosk_logs';
    let logs = [];
    let logIndex = new Map();
    const LOG_GROUPS = {wheels:'Колёса и тяга', battery:'АКБ и питание', lidar:'Лидар',
      cameras:'Камеры', navigation:'Карта и навигация', system:'Система'};
    const groupFilter = $('log-group-filter');
    const importantFilter = $('log-important');
    const groupOpen = new Map();
    let journalId = null;
    let logRevision = 0;
    let renderedRevision = -1;
    let hiddenThroughSeq = 0;

    function logGroup(entry) {
      if (Object.prototype.hasOwnProperty.call(LOG_GROUPS, entry.group)) return entry.group;
      const source = String(entry.source || '').toLowerCase().replace(/^robot_control\./, '');
      if (/^(lidar|ros_lidar|лидар)/.test(source)) return 'lidar';
      if (/^(wheel|master|мастер|колёс|колес|руление|тяга|управление)/.test(source)) return 'wheels';
      if (/^(battery|bms|акб|питание)/.test(source)) return 'battery';
      if (/^(camera|камер)/.test(source)) return 'cameras';
      if (/^(navigation|slam_map|карта|навигац)/.test(source)) return 'navigation';
      return 'system';
    }
    function selectedLogGroup() {
      return groupFilter && Object.prototype.hasOwnProperty.call(LOG_GROUPS, groupFilter.value) ? groupFilter.value : 'all';
    }
    function visibleLogs() {
      const selected = selectedLogGroup();
      return logs.filter(row => (selected === 'all' || row.group === selected) &&
        (!(importantFilter && importantFilter.checked) || row.level === 'err' || row.level === 'warn'));
    }
    function trimLogs() {
      while (logs.length > 400) {
        const dropped = logs.shift();
        if (dropped && dropped.key) logIndex.delete(dropped.key);
      }
    }

    function clockLabel(entry) {
      const label = entry && entry.time;
      if (typeof label === 'string' && label) return label;
      const ts = Number(entry && entry.ts);
      const d = Number.isFinite(ts) ? new Date(ts * 1000) : new Date();
      return String(d.getHours()).padStart(2,'0') + ':' + String(d.getMinutes()).padStart(2,'0')
        + ':' + String(d.getSeconds()).padStart(2,'0');
    }

    function setLogStatus(text) {
      if (logStatus) logStatus.textContent = text;
    }

    // Строка, рождённая самим экраном (очистка списка, копирование). Это тоже
    // настоящее действие оператора, поэтому подписываем его «экран:».
    function pushLocalLog(level, msg) {
      const now = new Date();
      const ts = String(now.getHours()).padStart(2,'0') + ':' + String(now.getMinutes()).padStart(2,'0')
        + ':' + String(now.getSeconds()).padStart(2,'0');
      logs.push({key: null, ts, level, msg, repeat: 1, local: true, group: 'system'});
      trimLogs(); logRevision++;
      renderLogs();
    }

    // Запись журнала с сервера. Повторяющиеся события (лидар, удержание тяги)
    // сервер сворачивает в счётчик «×N» — обновляем ту же строку, а не плодим.
    function addJournalEntry(entry) {
      if (!entry || typeof entry !== 'object') return;
      const seq = Number(entry.seq);
      const msg = String(entry.msg || '');
      if (!Number.isFinite(seq) || !msg) return;
      // seq and firstTs survive updates to the last timestamp/message of a
      // collapsed entry. Using ts+msg here used to duplicate every repeat.
      const key = seq + '|' + String(entry.firstTs || '');
      if (seq <= hiddenThroughSeq) return;
      const known = logIndex.get(key);
      const level = KNOWN_LEVELS.indexOf(String(entry.level)) >= 0 ? String(entry.level) : 'info';
      const row = {key, seq, ts: clockLabel(entry), level, msg,
                   repeat: Number(entry.repeat) > 0 ? Number(entry.repeat) : 1,
                   source: String(entry.source || ''), group: logGroup(entry), local: false};
      if (known) {
        if (known.repeat === row.repeat && known.ts === row.ts && known.msg === row.msg &&
            known.level === row.level && known.group === row.group && known.source === row.source) return;
        Object.assign(known, row);
        logs.splice(logs.indexOf(known), 1); logs.push(known);
      } else {
        logIndex.set(key, row); logs.push(row);
      }
      trimLogs(); logRevision++;
    }

    function renderLogs() {
      if (!logList) return;
      const scrollBefore = logList.scrollTop;
      const frag = document.createDocumentFragment();
      logList.innerHTML = '';
      const shown = visibleLogs();
      Object.entries(LOG_GROUPS).forEach(([id, label]) => {
        const items = shown.filter(row => row.group === id);
        if (!items.length) return;
        const group = document.createElement('details');
        group.className = 'log-group';
        group.setAttribute('data-group', id);
        const errors = items.filter(row => row.level === 'err').length;
        const warnings = items.filter(row => row.level === 'warn').length;
        group.setAttribute('data-alert', errors ? 'error' : warnings ? 'warn' : 'none');
        group.open = selectedLogGroup() !== 'all' ||
          !!(importantFilter && importantFilter.checked) ||
          (groupOpen.has(id) ? groupOpen.get(id) : id !== 'lidar');
        let observedOpen = group.open;
        group.addEventListener('toggle', () => {
          // Detached groups can fire toggle after a re-render: don't overwrite
          // the operator's current choice with an obsolete DOM node.
          if (group.isConnected && observedOpen !== group.open) {
            observedOpen = group.open; groupOpen.set(id, group.open);
          }
        });
        const title = document.createElement('summary');
        const name = document.createElement('b'); name.textContent = label;
        const count = document.createElement('span'); count.className = 'log-group-count';
        count.textContent = items.length + ' записей' + (errors ? ' · ошибок: ' + errors : '') +
          (warnings ? ' · предупреждений: ' + warnings : '');
        title.appendChild(name); title.appendChild(count); group.appendChild(title);
        items.forEach((l) => {
          const row = document.createElement('div');
          row.className = 'log-line'; row.setAttribute('data-level', l.level);
          if (l.local) row.setAttribute('data-local', '1');
          const ts = document.createElement('span'); ts.className='log-ts'; ts.textContent = l.ts;
          const lvl = document.createElement('span'); lvl.className='log-lvl '+l.level; lvl.textContent = l.level;
          const msg = document.createElement('span'); msg.className='log-msg';
          msg.textContent = l.repeat > 1 ? l.msg + '  ×' + l.repeat : l.msg;
          row.appendChild(ts); row.appendChild(lvl); row.appendChild(msg);
          if (l.source) row.title = 'источник: ' + l.source;
          group.appendChild(row);
        });
        frag.appendChild(group);
      });
      if (!shown.length) {
        const empty = document.createElement('p'); empty.className = 'log-empty';
        empty.textContent = logs.length ? 'Нет событий по выбранному фильтру' : 'Событий ещё не было';
        frag.appendChild(empty);
      }
      logList.appendChild(frag);
      if (logCount) logCount.textContent = String(shown.length);
      if (logAuto && logAuto.checked) logList.scrollTop = logList.scrollHeight;
      else logList.scrollTop = scrollBefore;
      renderedRevision = logRevision;
    }

    // Инкрементальное обновление логов без пересборки всего окна: только новые строки
    async function fetchLogs() {
      let data = null;
      let problem = '';
      const ctrl = new AbortController();
      const t = setTimeout(() => ctrl.abort(), 1100);
      try {
        const r = await fetch('api/kiosk_logs?limit=400', {cache:'no-store', signal: ctrl.signal});
        if (r.status === 401 || r.status === 403) {
          setLogStatus('журнал робота · нужен вход оператора (8090 — без входа)');
          return;
        }
        if (!r.ok) {
          setLogStatus('журнал робота · экран отвечает ' + r.status);
          return;
        }
        const j = await r.json().catch(() => null);
        if (!j) {
          setLogStatus('журнал робота · ответ не JSON — на пульте нужен вход оператора');
          return;
        }
        data = (j.ok && j.data) ? j.data : null;
      } catch (e) {
        if (e.name === 'AbortError') problem = 'таймаут /api/kiosk_logs';
        else problem = 'нет связи с /api/kiosk_logs';
      } finally { clearTimeout(t); }
      if (problem) {
        setLogStatus('журнал робота · ' + problem);
        return;
      }
      if (!data || !Array.isArray(data.entries)) {
        setLogStatus('журнал робота · ответ без записей');
        return;
      }
      if (data.journalId && journalId !== data.journalId) {
        journalId = data.journalId;
        logs = logs.filter(row => row.local); logIndex = new Map(); hiddenThroughSeq = 0;
        logRevision++;
      }
      data.entries.forEach(addJournalEntry);
      if (renderedRevision !== logRevision) renderLogs();
      describeJournal(data);
    }

    // Русские окончания: «1 запись · 2 записи · 5 записей».
    function ruCount(n, forms) {
      const abs = Math.abs(Number(n) || 0) % 100;
      const last = abs % 10;
      if (abs > 10 && abs < 20) return forms[2];
      if (last === 1) return forms[0];
      if (last >= 2 && last <= 4) return forms[1];
      return forms[2];
    }

    // Подпись под журналом: сколько настоящих записей, сколько ошибок, откуда
    // данные и не потерялись ли старые строки из-за размера буфера.
    function describeJournal(data) {
      const levels = data.levels || {};
      const errors = Number(levels.err) || 0;
      const count = Number(data.count) || 0;
      if (!count && !logs.length) {
        setLogStatus(LOG_STATUS_BASE + ' · событий ещё не было');
        return;
      }
      const parts = ['журнал робота',
                     count + ' ' + ruCount(count, ['запись', 'записи', 'записей'])];
      if (errors) parts.push(errors + ' ' + ruCount(errors, ['ошибка', 'ошибки', 'ошибок']));
      if (Number(data.dropped)) parts.push('вытеснено ' + data.dropped);
      if (data.source) parts.push('источник: ' + data.source);
      setLogStatus(parts.join(' · '));
    }

    if (groupFilter) groupFilter.addEventListener('change', renderLogs);
    if (importantFilter) importantFilter.addEventListener('change', renderLogs);
    setInterval(fetchLogs, 2500);
    fetchLogs();

    if (logClear) logClear.addEventListener('click', () => {
      logs.forEach(row => { if (row.key) hiddenThroughSeq = Math.max(hiddenThroughSeq, row.seq); });
      logs = []; logIndex = new Map(); logRevision++; renderLogs();
      pushLocalLog('info','экран: список очищен, журнал на роботе не тронут');
    });
    if (logCopy) logCopy.addEventListener('click', async () => {
      const shown = visibleLogs();
      const txt = Object.entries(LOG_GROUPS).map(([id, label]) => {
        const items = shown.filter(row => row.group === id);
        return items.length ? label + '\n' + items.map(l => l.ts+' ['+l.level.toUpperCase()+'] '+
          (l.source ? '['+l.source+'] ' : '') + l.msg + (l.repeat > 1 ? ' ×'+l.repeat : '')).join('\n') : '';
      }).filter(Boolean).join('\n\n');
      try { await navigator.clipboard.writeText(txt); pushLocalLog('ok','экран: список скопирован в буфер'); }
      catch(e) { pushLocalLog('warn','экран: не удалось скопировать — выдели вручную'); }
    });

    // также свайп индикатор — при первом посещении подмигнуть
    setTimeout(() => {
      if (!isOpen && hintBar) {
        try { hintBar.animate([{transform:'translateX(-50%) scale(1)'},{transform:'translateX(-50%) scale(1.04)'},{transform:'translateX(-50%) scale(1)'}], {duration: 900, iterations: 1}); } catch(e) {}
      }
    }, 1800);
  }

  /* ============ Руление и безопасная тяга BLDC ============ */
  function masterPortHint(d) {
    const info = d && d.masterPort;
    if (!info) return '';
    const configured = String(info.configured || '');
    const active = String(info.active || '');
    if (active) return ' Порт: ' + active + '.';
    const list = Array.isArray(info.candidates) ? info.candidates.filter(Boolean) : [];
    const tried = list.length ? list.join(', ') : (configured || '—');
    const error = info.error ? ' Ошибка: ' + String(info.error) + '.' : '';
    return ' Порт ' + (configured || '—') + ' не ответил' +
      (tried && tried !== configured ? ' (проверены: ' + tried + ')' : '') + '.' + error +
      ' Проверьте USB-кабель, RC_SLAM_PORT и scripts/master_check.py.';
  }

  const STEER_IDS = ['FL', 'FR', 'RL', 'RR'];
  /** Запасное значение, пока сервер не ответил: совпадает с прошивкой. */
  const STEER_FALLBACK_LIMIT = 90;
  let steerPresetBusy = false;
  let steerPresetMessage = null;
  let steerPreviewMode = 'live';
  let steerPreviewTimer = null;
  let steerLimitDeg = STEER_FALLBACK_LIMIT;
  const steerPreviewAngles = { FL: 0, FR: 0, RL: 0, RR: 0 };

  function steerClamp(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return null;
    if (window.RCSteerPresets && typeof window.RCSteerPresets.clampAngle === 'function') {
      return window.RCSteerPresets.clampAngle(n);
    }
    return Math.max(-steerLimitDeg, Math.min(steerLimitDeg, Math.round(n)));
  }

  function steerPresetList() {
    if (!window.RCSteerPresets) return [];
    if (typeof window.RCSteerPresets.list === 'function') return window.RCSteerPresets.list();
    return Array.isArray(window.RCSteerPresets.PRESETS) ? window.RCSteerPresets.PRESETS : [];
  }

  function steerPresetById(id) {
    if (window.RCSteerPresets && typeof window.RCSteerPresets.get === 'function') {
      return window.RCSteerPresets.get(id);
    }
    return steerPresetList().find((preset) => preset.id === id) || null;
  }

  /**
   * Предел угла приходит с сервера (steeringControl.limitDeg) и расходится по
   * ползункам, подписям и пресетам. Раньше число было зашито в HTML и в пяти
   * местах JS: экран показывал один ход, а плата принимала другой — отсюда
   * «кривое» руление. Теперь источник один, и пресет, который железо не тянет,
   * честно помечен недоступным вместо тихой обрезки угла.
   */
  function syncSteerLimit(control) {
    const presets = window.RCSteerPresets;
    const reported = Number(control && control.limitDeg);
    const next = Number.isFinite(reported) && reported > 0 ? Math.round(reported) : STEER_FALLBACK_LIMIT;
    if (presets && typeof presets.setLimit === 'function') presets.setLimit(next);
    const changed = next !== steerLimitDeg;
    steerLimitDeg = next;
    const text = presets && typeof presets.limitText === 'function' ? presets.limitText() : ('±' + next + '°');
    document.querySelectorAll('[data-steer-limit]').forEach((element) => { element.textContent = text; });
    [$('steer-master-slider')].concat(STEER_IDS.map((id) => $('steer-slider-' + id))).forEach((slider) => {
      if (!slider) return;
      slider.min = String(-next);
      slider.max = String(next);
      const value = steerClamp(slider.value);
      if (value !== null && Number(slider.value) !== value) slider.value = String(value);
    });
    document.querySelectorAll('[data-steer-range-labels]').forEach((box) => {
      const spans = box.querySelectorAll('span');
      if (spans.length >= 3) {
        spans[0].textContent = '−' + next + '° ← влево';
        spans[2].textContent = 'вправо → +' + next + '°';
      }
    });
    // Доступность пресетов зависит от предела: пересобираем кнопки при смене.
    if (changed) buildSteerPresetButtons();
    return changed;
  }

  function steerModeText(angle) {
    if (window.RCSteerPresets && typeof window.RCSteerPresets.modeFor === 'function') {
      return window.RCSteerPresets.modeFor(angle);
    }
    const n = steerClamp(angle);
    if (n === null) return 'Угол не задан';
    if (n === 0) return 'Прямо · движение вперёд/назад';
    return (n < 0 ? 'Краб влево' : 'Краб вправо') + ' · диагональ';
  }

  function updateSteerDiagram() {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    STEER_IDS.forEach((id) => {
      const wheel = panel.querySelector('[data-preview-wheel="' + id + '"]');
      if (wheel) wheel.style.transform = 'rotate(' + steerPreviewAngles[id] + 'deg)';
    });
    const values = STEER_IDS.map((id) => steerPreviewAngles[id]);
    const uniform = values.every((value) => Math.abs(value - values[0]) <= 1);
    setText($('steer-master-value'), uniform ? Math.round(values[0]) + '°' : 'разные');
    setText($('steer-hero-angle'), uniform ? Math.round(values[0]) + '°' : '—');
    setText($('steer-hero-mode'), uniform ? steerModeText(values[0]) : 'Колёса под разными углами');
  }

  function setSteerPreviewAngles(angles, mode) {
    if (angles) {
      STEER_IDS.forEach((id) => {
        const value = steerClamp(angles[id]);
        if (value !== null) steerPreviewAngles[id] = value;
      });
    }
    if (mode) steerPreviewMode = mode;
    updateSteerDiagram();
  }

  function steerLiveAngles(data, key) {
    const motors = Array.isArray(data && data.motors) ? data.motors : [];
    const out = {};
    STEER_IDS.forEach((id) => {
      const motor = motors.find((item) => item && item.id === id);
      const value = motor && motor.online === true && motor[key] !== null &&
        motor[key] !== undefined && Number.isFinite(+motor[key]) ? +motor[key] : null;
      out[id] = value;
    });
    return out;
  }

  function steerPreviewUniform() {
    const values = STEER_IDS.map((id) => steerPreviewAngles[id]);
    return values.every((value) => Math.abs(value - values[0]) <= 1) ? values[0] : null;
  }

  function steerPresetBlockedReason() {
    const d = lastData || {};
    const control = d.steeringControl || {};
    if (!control.available) {
      return String(control.source || '') === 'master-peer'
        ? 'Руление и пресеты доступны на бортовом экране (порт 8090).'
        : 'Источник master не подключён: пресеты недоступны.';
    }
    if (!steeringAuthorized) return 'Управление недоступно: разблокируйте PIN.';
    const drive = d.drive || {};
    if (drive.active || Number(drive.commandedPercent || 0) !== 0) {
      return 'Сначала остановите тягу кнопкой СТОП.';
    }
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const find = (id) => motors.find((item) => item && item.id === id) || null;
    const offline = STEER_IDS.filter((id) => {
      const motor = find(id);
      return !(motor && motor.online === true);
    });
    if (offline.length) return 'Нет связи: ' + offline.join(', ') + '.';
    const unhomed = STEER_IDS.filter((id) => {
      const motor = find(id);
      return !(motor && motor.homed === true);
    });
    if (unhomed.length) return 'Не откалиброваны: ' + unhomed.join(', ') + '. Сначала калибровка.';
    const moving = STEER_IDS.filter((id) => {
      const motor = find(id);
      return !!(motor && (motor.moving || motor.calibrating || motor.cycling));
    });
    if (moving.length) return 'Дождитесь остановки: ' + moving.join(', ') + '.';
    if (Object.keys(pendingSteer).length || Object.keys(pendingZero).length || Object.keys(pendingCalib).length) {
      return 'Предыдущая команда ещё выполняется — дождитесь её.';
    }
    return '';
  }

  function previewAllCards(angles) {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    STEER_IDS.forEach((id) => {
      const angle = steerClamp(angles[id]);
      if (angle === null) return;
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const target = $('steer-target-' + id);
      if (slider) slider.value = String(angle);
      if (target) target.textContent = angle + '°';
      if (card) card.dataset.steerDirty = 'true';
    });
    setSteerPreviewAngles(angles, 'manual');
  }

  function renderSteerPresetBar(data, motors, available, tractionActive) {
    const panel = $('swipe-motors-view');
    if (!panel || !$('steer-preset-grid')) return;
    const slider = $('steer-master-slider');
    const apply = $('steer-apply-all');
    const status = $('steer-preset-status');
    let uniform = null;

    if (steerPreviewMode === 'live') {
      const targets = steerLiveAngles(data, 'targetAngle');
      const currents = steerLiveAngles(data, 'angle');
      const use = STEER_IDS.map((id) => (targets[id] === null ? currents[id] : targets[id]));
      const complete = use.every((value) => value !== null);
      if (complete) {
        const angles = {};
        STEER_IDS.forEach((id, index) => {
          const value = steerClamp(use[index]);
          angles[id] = value === null ? 0 : value;
        });
        setSteerPreviewAngles(angles, 'live');
        uniform = steerPreviewUniform();
        // Ползунок не трогаем, пока его держат: телеметрия вырывала ручку
        // из-под пальца на середине жеста (опрос экрана — каждые 250 мс).
        if (slider && uniform !== null && !steerPresetBusy && document.activeElement !== slider) {
          slider.value = String(Math.round(uniform));
        }
      }
      updateSteerDiagram();
      if (!complete) {
        setText($('steer-master-value'), '—');
        setText($('steer-hero-angle'), '—');
        setText($('steer-hero-mode'), 'Нет свежих данных от всех колёс');
      }
    } else {
      uniform = steerPreviewUniform();
    }

    const reason = steerPresetBlockedReason();
    if (apply) {
      apply.disabled = steerPresetBusy || !!reason;
      apply.textContent = steerPresetBusy ? 'Применяю…' : 'Применить ко всем колёсам';
    }
    if (slider) slider.disabled = steerPresetBusy || !available || !steeringAuthorized;
    panel.querySelectorAll('[data-steer-preset]').forEach((button) => {
      const preset = steerPresetById(button.dataset.steerPreset);
      if (!preset) return;
      const active = preset.available && uniform !== null && Math.abs(uniform - preset.angle) <= 1;
      button.classList.toggle('is-active', active);
      button.setAttribute('aria-pressed', String(active));
      // Недоступный пресет остаётся нажимаемым: по нажатию экран показывает
      // причину. Рабочий гаснет, пока команда всё равно была бы отклонена —
      // кнопка не обещает того, чего сейчас сделать нельзя.
      button.disabled = steerPresetBusy || (preset.available && !!reason);
      if (preset.reason) button.title = preset.reason;
    });

    const now = Date.now();
    if (steerPresetMessage && steerPresetMessage.until <= now) steerPresetMessage = null;
    if (!status) return;
    if (steerPresetMessage) {
      status.textContent = steerPresetMessage.text;
      status.dataset.error = String(!!steerPresetMessage.error);
    } else if (!window.RCSteerPresets) {
      status.textContent = 'Пресеты не загрузились — перезагрузите экран.';
      status.dataset.error = 'true';
    } else if (reason) {
      status.textContent = reason;
      status.dataset.error = 'true';
    } else if (tractionActive) {
      status.textContent = 'Тяга включена — сначала СТОП.';
      status.dataset.error = 'true';
    } else {
      status.textContent = 'Колёса готовы. Выберите режим.';
      status.dataset.error = 'false';
    }
  }

  function errorText(reason, detail) {
    const limit = Number(detail && detail.limit);
    const limitText = Number.isFinite(limit) && limit > 0
      ? '±' + Math.round(limit) + '°'
      : (window.RCSteerPresets && typeof window.RCSteerPresets.limitText === 'function'
        ? window.RCSteerPresets.limitText() : '±' + steerLimitDeg + '°');
    const messages = {
      pin_required: 'PIN истёк · разблокируйте',
      control_not_supported: 'управление только на бортовом экране',
      module_offline: 'нет свежей связи',
      serial_unavailable: 'порт мастера недоступен',
      not_homed: 'сначала калибровка или 0° вручную',
      module_busy: 'дождитесь остановки модуля / выключите тягу',
      angle_limit: 'угол за пределами ' + limitText,
      command_queue_full: 'очередь команд занята',
      invalid_module: 'неизвестный модуль',
      invalid_angle: 'неверный угол',
      estop: 'E-STOP взведён — снимите защёлку',
      payload: 'неверный формат команды',
      command_failed: 'команда не выполнена'
    };
    return messages[reason] || 'команда не принята';
  }

  async function applySteerAngles(angles, label) {
    if (steerPresetBusy) return false;
    const reason = steerPresetBlockedReason();
    if (reason) {
      steerPresetMessage = { text: reason, error: true, until: Date.now() + 9000 };
      renderSteeringControls(lastData || {});
      return false;
    }
    const targets = {};
    for (const id of STEER_IDS) {
      const angle = steerClamp(angles[id]);
      if (angle === null) {
        steerPresetMessage = { text: 'Не задан угол для ' + id + '.', error: true, until: Date.now() + 9000 };
        renderSteeringControls(lastData || {});
        return false;
      }
      targets[id] = angle;
    }
    steerPresetBusy = true;
    steerPresetMessage = { text: 'Применяю «' + label + '»…', error: false, until: Date.now() + 20000 };
    renderSteeringControls(lastData || {});
    for (const id of STEER_IDS) {
      const angle = targets[id];
      try {
        const response = await fetch('api/kiosk_steer', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id, angle })
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) {
            setSteeringAuthorized(false);
          }
          const message = errorText(result && result.reason, result);
          steerResult[id] = { text: 'пресет: ' + message, error: true, until: Date.now() + 10000 };
          steerPresetMessage = {
            text: 'Остановлено на ' + id + ': ' + message + '. Остальные колёса не трогаем.',
            error: true, until: Date.now() + 12000
          };
          steerPresetBusy = false;
          scheduleSteerPreviewLive();
          renderSteeringControls(lastData || {});
          return false;
        }
        const sent = steerClamp(result.angle);
        pendingSteer[id] = { angle: sent === null ? angle : sent, sentAt: Date.now() };
        const card = panelCard(id);
        if (card) card.dataset.steerDirty = 'false';
        steerResult[id] = { text: 'пресет · ' + angle + '°', error: false, until: Date.now() + 8000 };
      } catch (error) {
        steerPresetMessage = {
          text: 'Нет связи с сервером на ' + id + '. Проверьте сеть.',
          error: true, until: Date.now() + 12000
        };
        steerPresetBusy = false;
        scheduleSteerPreviewLive();
        renderSteeringControls(lastData || {});
        return false;
      }
    }
    steerPresetBusy = false;
    scheduleSteerPreviewLive();
    steerPresetMessage = {
      text: '«' + label + '»: ' + steerModeText(targets.FL) + '. Ползунок движения — в самом низу экрана.',
      error: false, until: Date.now() + 10000
    };
    renderSteeringControls(lastData || {});
    return true;
  }

  function scheduleSteerPreviewLive() {
    if (steerPreviewTimer) clearTimeout(steerPreviewTimer);
    steerPreviewTimer = setTimeout(() => {
      steerPreviewTimer = null;
      steerPreviewMode = 'live';
      renderSteeringControls(lastData || {});
    }, 5000);
  }

  function panelCard(id) {
    const panel = $('swipe-motors-view');
    return panel ? panel.querySelector('[data-steer-card="' + id + '"]') : null;
  }

  async function chooseSteerPreset(preset) {
    if (!preset || steerPresetBusy) return;
    if (!preset.available) {
      steerPresetMessage = { text: preset.reason || 'Пресет недоступен.', error: true, until: Date.now() + 12000 };
      renderSteeringControls(lastData || {});
      return;
    }
    const angles = {};
    STEER_IDS.forEach((id) => { angles[id] = preset.angle; });
    previewAllCards(angles);
    const slider = $('steer-master-slider');
    if (slider) slider.value = String(preset.angle);
    updateSteerDiagram();
    await applySteerAngles(angles, preset.title);
  }

  async function applyMasterAngles() {
    const slider = $('steer-master-slider');
    if (!slider) return;
    const angle = steerClamp(slider.value);
    if (angle === null) return;
    const angles = {};
    STEER_IDS.forEach((id) => { angles[id] = angle; });
    previewAllCards(angles);
    await applySteerAngles(angles, 'Угол всех колёс ' + angle + '°');
  }

  function buildSteerPresetButtons() {
    const grid = $('steer-preset-grid');
    if (!grid) return;
    grid.innerHTML = '';
    const presets = steerPresetList();
    if (!presets.length) {
      const note = document.createElement('p');
      note.className = 'steer-preset-status';
      note.textContent = 'Пресеты не загрузились — перезагрузите экран.';
      grid.appendChild(note);
      return;
    }
    presets.forEach((preset) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'steer-preset';
      button.dataset.steerPreset = preset.id;
      button.setAttribute('aria-pressed', 'false');
      if (!preset.available) {
        button.classList.add('is-unavailable');
        button.title = preset.reason || 'Недоступно';
      }
      const icon = document.createElement('span');
      icon.className = 'steer-preset-icon';
      icon.setAttribute('aria-hidden', 'true');
      const iconAngles = preset.available
        ? [preset.angle, preset.angle, preset.angle, preset.angle]
        : [steerLimitDeg, -steerLimitDeg, steerLimitDeg, -steerLimitDeg];
      iconAngles.forEach((angle) => {
        const wheel = document.createElement('i');
        wheel.style.setProperty('--wheel-angle', angle + 'deg');
        icon.appendChild(wheel);
      });
      const title = document.createElement('b');
      title.textContent = preset.title;
      const hint = document.createElement('small');
      hint.textContent = preset.hint;
      button.appendChild(icon);
      button.appendChild(title);
      button.appendChild(hint);
      button.addEventListener('click', () => {
        // Пресет берём свежим: предел мог прийти с сервера уже после отрисовки.
        void chooseSteerPreset(steerPresetById(preset.id) || preset);
      });
      grid.appendChild(button);
    });
  }

  function renderSteeringControls(data) {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const d = data || {};
    const control = d.steeringControl || {};
    const available = control.available === true;
    // Предел руля берём из ответа сервера: ползунки, подписи и пресеты
    // подстраиваются под то, что реально принимает плата.
    syncSteerLimit(control);
    const source = String(control.source || d.source || 'unknown');
    const motors = Array.isArray(d.motors) ? d.motors : [];
    const drive = d.drive || {};
    const tractionActive = !!drive.active || Number(drive.commandedPercent || 0) !== 0;
    const sourceStatus = $('steer-source-status');
    const authStatus = $('steer-auth-status');
    renderSteerPresetBar(d, motors, available, tractionActive);

    if (sourceStatus) {
      if (!available && String(control.source || '') === 'master-peer') {
        sourceStatus.textContent = 'Руление и тяга — только на бортовом экране (порт 8090). Здесь показана телеметрия.';
      } else if (!available) {
        sourceStatus.textContent = 'Ручной поворот доступен только при RC_SLAM_SOURCE=master и подключённом UART мастера.';
      } else if (d.linkOk === false) {
        sourceStatus.textContent = 'Мастер выбран, но свежей связи с рулевыми модулями пока нет.' + masterPortHint(d);
      } else if (control.calibration) {
        sourceStatus.textContent = 'Мастер подключён · ноль руля — «Калибровка» по оптодатчику D18; «0° вручную» — если датчика нет.';
      } else {
        sourceStatus.textContent = 'Мастер подключён · перед поворотом задайте ручной программный ноль каждому модулю.';
      }
    }
    if (authStatus) authStatus.textContent = steeringAuthorized
      ? (window.RCRemoteControls ? 'оператор · управление через бортовой сервер' : 'доступ без PIN')
      : 'управление недоступно';

    const now = Date.now();
    ['FL', 'FR', 'RL', 'RR'].forEach((id) => {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const currentEl = $('steer-current-' + id);
      const targetEl = $('steer-target-' + id);
      const stateEl = $('steer-state-' + id);
      const sendButton = panel.querySelector('[data-steer-send="' + id + '"]');
      const originButton = panel.querySelector('[data-steer-origin="' + id + '"]');
      const calibButton = panel.querySelector('[data-steer-calib="' + id + '"]');
      if (!card || !slider) return;

      const motor = motors.find((m) => m && m.id === id) || null;
      const current = motor && motor.angle != null && Number.isFinite(+motor.angle) ? +motor.angle : null;
      const target = motor && motor.targetAngle != null && Number.isFinite(+motor.targetAngle) ? +motor.targetAngle : null;
      const online = !!(motor && motor.online === true && current != null);
      const homed = !!(motor && motor.homed === true);
      const moving = !!(motor && motor.moving);
      const calibrating = !!(motor && motor.calibrating);
      const busy = moving || calibrating;
      let pending = pendingSteer[id] || null;
      let zeroPending = pendingZero[id] || null;
      let calibPending = pendingCalib[id] || null;
      const homeSource = motor && motor.homeSource ? String(motor.homeSource) : (homed ? 'manual' : 'none');
      const calibError = motor && Number(motor.calibError) > 0 ? Number(motor.calibError) : 0;
      const calibErrorText = (motor && motor.calibErrorText) || (calibError ? 'код ' + calibError : '');

      // Команда считается исполненной, когда колесо остановилось: телеметрия
      // знает и то, что оно стоит, и какую цель ему дали. Раньше ждали
      // попадания ровно в ±1° от заданного угла, а иначе держали колесо (а с
      // ним и все пресеты) 15 секунд: колесо, вставшее в паре градусов от цели
      // после доводки по потенциометру, «зависало» молча, и оператор не
      // понимал, что произошло и почему руль не слушается.
      if (pending && !busy) {
        const want = pending.angle;
        // Доехало — угол совпал. Не доехало, но команда дошла до платы — колесо
        // встало там, где смогло (упор, остаток после доводки по потенциометру).
        // И то и другое значит: ждать больше нечего, колесо можно разблокировать.
        const arrived = current != null && Math.abs(current - want) <= 1;
        const delivered = target != null && Math.abs(target - want) <= 1;
        const timedOut = now - pending.sentAt
          > (current == null ? STEER_TELEMETRY_TIMEOUT_MS : STEER_SETTLE_MS);
        if (arrived || delivered || timedOut) {
          delete pendingSteer[id];
          pending = null;
          if (!arrived && current != null) {
            // Не доехало: говорим прямо, вместо молчаливой блокировки. Иначе
            // оператор видит «команда отправлена» и не понимает, почему тяга
            // не включается из-за невыровненного руля.
            steerResult[id] = {
              text: 'не доехало до ' + want + '°: стоит на ' + current.toFixed(1) + '°',
              error: true,
              until: now + 10000,
            };
          }
        }
      }
      if (zeroPending && homed) {
        delete pendingZero[id];
        zeroPending = null;
      } else if (zeroPending && now - zeroPending.sentAt > 15000) {
        delete pendingZero[id];
        zeroPending = null;
      }
      // Калибровка: ждём, что модуль начал её (calibrating), и показываем итог.
      if (calibPending && calibrating) {
        calibPending.started = true;
      } else if (calibPending && calibPending.started && !calibrating) {
        delete pendingCalib[id];
        calibPending = null;
        steerResult[id] = homed && homeSource === 'opto'
          ? { text: 'ноль найден по датчику', error: false, until: now + 8000 }
          : { text: 'ошибка калибровки: ' + (calibErrorText || 'ноль не найден'), error: true, until: now + 15000 };
      } else if (calibPending && !calibPending.started && now - calibPending.sentAt > 8000) {
        delete pendingCalib[id];
        calibPending = null;
        steerResult[id] = { text: 'калибровка не началась · проверьте прошивку v2', error: true, until: now + 8000 };
      }

      let value = pending ? pending.angle
        : (moving && target != null ? target
          : (current != null ? current : Number(slider.value)));
      const clamped = steerClamp(value);
      value = clamped === null ? 0 : clamped;
      const dirty = card.dataset.steerDirty === 'true';
      // Нет свежих данных от колеса — ползунок не трогаем: раньше он прыгал в 0°
      // и показывал цель, которой на борту не существует.
      const hasData = !!pending || (moving && target != null) || current != null;
      if (!dirty && hasData) slider.value = String(value);
      if (currentEl) currentEl.textContent = current == null ? '—' : Math.round(current) + '°';
      // Ползунок ограничен ходом руля, а при калибровке цель — до ±135°: показываем настоящую.
      if (targetEl) targetEl.textContent = (!dirty && busy && target != null ? Math.round(target) : Math.round(+slider.value || 0)) + '°';

      const ready = available && steeringAuthorized && online && homed && !busy && !pending && !zeroPending &&
        !calibPending && !tractionActive && !steerPresetBusy;
      const canSetZero = available && steeringAuthorized && online && !busy && !pending && !zeroPending &&
        !calibPending && !tractionActive && !steerPresetBusy;
      const canCalibrate = canSetZero && !!control.calibration;
      slider.disabled = !ready;
      if (window.RCSteerDial) window.RCSteerDial.sync(slider, online ? current : null);
      card.querySelectorAll('[data-steer-adjust="' + id + '"], [data-steer-zero="' + id + '"]').forEach((button) => {
        button.disabled = !ready;
      });
      if (originButton) originButton.disabled = !canSetZero;
      if (calibButton) {
        calibButton.disabled = !canCalibrate;
        calibButton.hidden = !control.calibration && available;
      }
      const homeEl = $('steer-home-' + id);
      if (homeEl) {
        homeEl.dataset.home = !online ? 'none' : (calibError && !homed ? 'error' : homeSource);
        homeEl.textContent = !online ? 'ноль: —'
          : homeSource === 'opto' && homed ? 'ноль: по датчику'
          : homeSource === 'manual' && homed ? 'ноль: вручную'
          : calibError ? 'ноль: нет · ' + calibErrorText
          : 'ноль: не задан';
        homeEl.title = homeEl.textContent;
      }
      const optoEl = $('steer-opto-' + id);
      if (optoEl) {
        const opto = online && motor && typeof motor.opto === 'boolean' ? motor.opto : null;
        optoEl.dataset.opto = opto === null ? 'unknown' : (opto ? 'on' : 'off');
        optoEl.textContent = opto === null ? 'датчик: —' : (opto ? 'датчик: закрыт' : 'датчик: свободен');
      }
      if (sendButton) sendButton.disabled = !ready || !dirty;
      card.dataset.ready = String(ready);

      let label;
      if (!available) label = 'нет управления';
      else if (!steeringAuthorized) label = 'управление недоступно';
      else if (!online) label = 'нет связи';
      else if (calibrating) label = 'калибровка по датчику…';
      else if (calibPending) label = 'калибровка в очереди';
      else if (moving) label = 'поворачивается';
      else if (tractionActive) label = 'тяга включена · сначала стоп';
      else if (zeroPending) label = 'ноль в очереди';
      else if (pending) label = 'команда отправлена';
      else if (!homed && calibError) label = 'ошибка калибровки';
      else if (!homed) label = control.calibration ? 'нужна калибровка' : 'нужен ручной ноль';
      else label = homeSource === 'opto' ? 'готов · ноль по датчику' : 'готов · ноль вручную';

      const message = steerResult[id];
      if (message && message.until > now) {
        label = message.text;
        card.dataset.error = String(!!message.error);
      } else {
        delete steerResult[id];
        card.dataset.error = 'false';
      }
      if (stateEl) stateEl.textContent = label;
    });

    if (window.RCPotMode) window.RCPotMode.render(document.querySelector('.steer-calib-bar'), d, steeringAuthorized);
    const calibAll = $('steer-calib-all');
    if (calibAll) {
      const online = motors.filter((m) => m && m.online === true);
      const anyBusy = online.some((m) => m.moving || m.calibrating || m.cycling);
      const anyPending = Object.keys(pendingCalib).length > 0;
      calibAll.disabled = !(available && control.calibration && steeringAuthorized && online.length > 0 &&
        !anyBusy && !anyPending && !tractionActive && !steerPresetBusy);
      calibAll.textContent = online.some((m) => m.calibrating) ? 'Калибровка идёт…' : 'Калибровать все колёса';
    }
    renderMasterMessages(d.masterMessages);
    renderDriveControls(d);
  }

  function renderMasterMessages(list) {
    const box = $('steer-messages');
    if (!box) return;
    const items = Array.isArray(list) ? list.slice(-4) : [];
    box.hidden = items.length === 0;
    const html = items.map((item) => {
      const text = String((item && item.text) || '');
      // Формулировки — те, которыми сервер реально пишет об отказе:
      // «команда «w 90.0» не выполнена: <причина>», «не доставлена — порт
      // колеса не на связи», «нет связи». Без них главное сообщение о
      // непоехавшем колесе висело бы в списке обычным текстом.
      const warn = /ОШИБКА|нет связи|не доставлена|не выполнена|нет команды|отклонена|заблокирован|битый/.test(text);
      const time = item && item.ts ? clockText(new Date(item.ts * 1000)) : '';
      return '<li' + (warn ? ' data-warn="true"' : '') + '><time>' + escapeHtml(time) + '</time> ' + escapeHtml(text) + '</li>';
    }).join('');
    if (box.dataset.html !== html) {
      box.innerHTML = html;
      box.dataset.html = html;
    }
  }

  function escapeHtml(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, (c) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    }[c]));
  }

  const DRIVE_REASON_TEXT = {
    command_expired: 'Команда устарела в сети. Повторите после восстановления связи.',
    navigation_stopping: 'Маршрут останавливается. Повторите после остановки.',
    serial_unavailable: 'Нет связи с USB-мастером.',
    module_offline: 'Нет свежей телеметрии всех четырёх модулей.',
    drive_firmware_required: 'Обновите прошивку всех четырёх плат для тяги и watchdog.',
    servo_firmware_required: 'Прошейте плату актуальным motor_wheel.ino (v15).',
    estop: 'Включён E-STOP: тяга заблокирована.',
    not_homed: 'Сначала откалибруйте колёса по датчику (или задайте 0° вручную).',
    module_busy: 'Дождитесь остановки рулевых модулей.',
    // Старый сервер ещё мог запирать тягу по углам — текст оставлен, чтобы
    // отказ был понятен и после обновления одного экрана без другого.
    steering_misaligned: 'Колёса должны смотреть в одном направлении (±2,5°).',
    control_not_supported: 'Тяга доступна только в режиме master.',
    command_queue_full: 'Очередь команд занята.',
    pin_required: 'PIN истёк; плата остановится по watchdog.'
  };

  /* Тяга: вход «P» драйвера ZS-Z11H ведёт Servo на D3. Бортовой экран
   * отправляет целое значение 0…150 без пересчёта. Обороты — только показание. */

  /* --------------------------------------------------------------- манёвры
   * Пресет здесь — цель, а не время. Старые прошивки с одометрией считают
   * путь по тахометру; SC v16 используется только для показа, поэтому с ним
   * сервер оценивает путь по времени. Числа сохраняются на сервере.
   */
  const MANEUVER_ORDER = ['forward', 'backward', 'left', 'right', 'spin_left', 'spin_right'];
  // Заводские значения — только для первой отрисовки, пока не пришёл
  // манифест с сервера (он же источник истины после правок).
  const MANEUVER_FALLBACK = {
    forward: {title: 'Вперёд', kind: 'distance', distanceM: 1, percent: 12},
    backward: {title: 'Назад', kind: 'distance', distanceM: 0.5, percent: 10},
    left: {title: 'Боком влево', kind: 'distance', distanceM: 0.5, percent: 10},
    right: {title: 'Боком вправо', kind: 'distance', distanceM: 0.5, percent: 10},
    spin_left: {title: 'Разворот влево', kind: 'angle', angleDeg: 90, percent: 10},
    spin_right: {title: 'Разворот вправо', kind: 'angle', angleDeg: 90, percent: 10}
  };
  const MANEUVER_REASON_TEXT = {
    invalid_maneuver: 'Такого манёвра нет — обновите страницу.',
    invalid_value: 'Проверьте числа: цель и процент в допустимых пределах.',
    maneuver_busy: 'Другой манёвр уже идёт — сначала СТОП.',
    module_busy: 'Сначала остановите тягу и дождитесь, пока колёса встанут.',
    module_offline: 'Нужна свежая телеметрия всех четырёх колёс.',
    serial_unavailable: 'Нет связи с USB-платами.',
    not_homed: 'У колеса нет нуля — сначала калибровка по датчику.',
    steering_timeout: 'Колёса не успели встать в углы манёвра.',
    drive_not_ready: 'Тяга сейчас недоступна.',
    pin_required: 'PIN истёк — разблокируйте управление.',
    estop: 'Включён E-STOP: манёвр заблокирован.',
    control_not_supported: 'Манёвры доступны только в режиме master.',
    request_header_required: 'Запрос без заголовка команды.',
    command_failed: 'Сервер не принял команду.'
  };
  let maneuverMessage = '';
  let maneuverMessageUntil = 0;
  let maneuverPending = false;

  function maneuverPreset(id, config) {
    const presets = (config && config.presets) || {};
    return Object.assign({}, MANEUVER_FALLBACK[id] || {}, presets[id] || {});
  }

  function maneuverKind(preset) {
    return preset.kind === 'distance' ? 'distance' : 'angle';
  }

  function maneuverValue(preset) {
    return maneuverKind(preset) === 'distance'
      ? Number(preset.distanceM) : Number(preset.angleDeg);
  }

  function maneuverText(value, kind) {
    const number = Number(value) || 0;
    return kind === 'distance' ? number.toFixed(2) + ' м' : Math.round(number) + '°';
  }

  function maneuverRefusal(result) {
    const reason = (result && result.body && result.body.reason) || 'command_failed';
    if (reason === 'pin_required') steeringAuthorized = false;
    maneuverMessage = MANEUVER_REASON_TEXT[reason] || ('Отказ: ' + reason);
    maneuverMessageUntil = Date.now() + 8000;
  }

  async function postManeuver(url, payload) {
    try {
      const response = await fetch(url, {
        method: 'POST', cache: 'no-store',
        headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'steering'},
        body: JSON.stringify(payload)
      });
      const body = await response.json().catch(() => ({}));
      return {ok: response.ok && body.ok !== false, status: response.status, body};
    } catch (error) {
      return {ok: false, status: 0, body: {reason: 'command_failed'}};
    }
  }

  /** Почему манёвр сейчас не поедет — словами, но кнопку не гасим.
   *
   * Пресеты доступны всегда: оператор нажимает «Ехать», запрос уходит на сервер,
   * и тот отвечает точной причиной («нет нуля», «нет связи с платой», «PIN
   * истёк»). Мёртвая серая кнопка не говорит ничего и заставляет гадать.
   */
  function maneuverReadiness(ctx) {
    const drive = ctx.drive || {};
    if (!ctx.supported) {
      return 'Пресеты ждут настройки от сервера — нажмите «Ехать», он ответит причиной.';
    }
    if (!ctx.available) {
      return 'Манёвры ведёт бортовая программа — нажмите «Ехать», она ответит причиной.';
    }
    if (!ctx.authorized) {
      return 'Управление заблокировано: «Ехать» попросит PIN.';
    }
    if (!drive.canDrive) {
      const reason = DRIVE_REASON_TEXT[drive.reason] || 'тяга не готова';
      return 'Пока не поедет: ' + reason + ' Кнопки работают — «Ехать» покажет точную причину.';
    }
    return '';
  }

  function renderManeuvers(data) {
    const stop = $('maneuver-stop');
    const statusEl = $('maneuver-status');
    if (!stop || !statusEl) return;
    const d = data || {};
    const config = d.maneuverConfig || null;
    const supported = !!(config && config.supported !== false);
    const drive = d.drive || {};
    const state = drive.maneuver || {};
    const active = !!state.active;
    const available = !!(d.steeringControl && d.steeringControl.available);
    const authorized = steeringAuthorized && available;
    const editing = document.activeElement && document.activeElement.id;
    MANEUVER_ORDER.forEach((id) => {
      const preset = maneuverPreset(id, config || {});
      const targetEl = $('maneuver-target-' + id);
      const percentEl = $('maneuver-percent-' + id);
      // Пока оператор набирает число, телеметрия его не перебивает.
      if (targetEl && editing !== ('maneuver-target-' + id)) {
        targetEl.value = String(maneuverValue(preset));
      }
      if (percentEl && editing !== ('maneuver-percent-' + id)) {
        percentEl.value = String(preset.percent);
      }
      const button = $('maneuver-' + id);
      if (!button) return;
      // «Доступны всегда»: гасим только на время полёта запроса (двойной клик).
      // Всё остальное — не повод для серой кнопки: пусть ответит сервер.
      button.disabled = maneuverPending;
      button.textContent = (active && state.name === id) ? 'Едем…' : 'Ехать';
    });
    // СТОП манёвра доступен всегда: он же и просто «снять тягу».
    stop.disabled = maneuverPending;
    const kind = state.kind === 'distance' ? 'distance' : 'angle';
    let text;
    if (maneuverMessage && Date.now() < maneuverMessageUntil) {
      text = maneuverMessage;
    } else if (active) {
      const percent = Math.round((Number(state.progress) || 0) * 100);
      text = 'Манёвр «' + (state.title || state.name || '') + '»: ' + percent + '% · ' +
        maneuverText(state.value, kind) + ' из ' + maneuverText(state.target, kind) +
        ' · ' + (state.mode === 'time' ? 'по времени (без одометрии)' : 'по тахометру');
    } else if (state.state === 'done' || state.state === 'failed' || state.state === 'cancelled') {
      text = (state.message || 'Манёвр завершён.');
    } else {
      const ready = maneuverReadiness({supported: supported, available: available,
                                       authorized: authorized, drive: drive});
      text = ready || 'Готово · выберите манёвр. Числа в строках — метры, градусы и рабочий процент.';
    }
    statusEl.textContent = text;
    statusEl.dataset.active = String(active);
  }

  /** Старт манёвра: сначала сохраняем числа из формы, потом пускаем. */
  async function startManeuver(id) {
    if (maneuverPending) return;
    if (!steeringAuthorized) {
      // Не молчим: пресет доступен, просто сначала нужен PIN.
      maneuverMessage = 'Разблокируйте управление PIN — и манёвр поедет.';
      maneuverMessageUntil = Date.now() + 8000;
      renderManeuvers(lastData || {});
      if (window._openSteeringAuth) window._openSteeringAuth();
      return;
    }
    // Пока манёвр идёт, числа не сохраняем: сервер всё равно откажет (рывок).
    // Зато «Ехать» отвечает «другой манёвр уже идёт — сначала СТОП».
    const running = !!(((lastData || {}).drive || {}).maneuver || {}).active;
    const preset = maneuverPreset(id, (lastData || {}).maneuverConfig || {});
    const kind = maneuverKind(preset);
    const targetEl = $('maneuver-target-' + id);
    const percentEl = $('maneuver-percent-' + id);
    const target = Number(targetEl ? targetEl.value : maneuverValue(preset));
    const percent = Number(percentEl ? percentEl.value : preset.percent);
    if (!(target > 0) || !(percent > 0)) {
      maneuverMessage = 'Проверьте числа: цель и процент должны быть больше нуля.';
      maneuverMessageUntil = Date.now() + 6000;
      renderManeuvers(lastData || {});
      return;
    }
    maneuverPending = true;
    renderManeuvers(lastData || {});
    const values = kind === 'distance'
      ? {distanceM: target, percent: percent}
      : {angleDeg: target, percent: percent};
    if (!running) {
      const saved = await postManeuver('api/kiosk_maneuver_config', {name: id, values: values});
      if (!saved.ok) {
        maneuverPending = false;
        maneuverRefusal(saved);
        renderManeuvers(lastData || {});
        return;
      }
    }
    const started = await postManeuver('api/kiosk_maneuver', {action: 'start', name: id});
    maneuverPending = false;
    if (!started.ok) {
      maneuverRefusal(started);
    } else {
      maneuverMessage = 'Манёвр «' + (preset.title || id) + '» начат — сервер ведёт его сам.';
      maneuverMessageUntil = Date.now() + 4000;
    }
    renderManeuvers(lastData || {});
  }

  async function stopManeuver() {
    if (maneuverPending) return;
    maneuverPending = true;
    const result = await postManeuver('api/kiosk_maneuver', {action: 'stop'});
    maneuverPending = false;
    if (result.ok) {
      maneuverMessage = 'Манёвр остановлен, тяга снята.';
      maneuverMessageUntil = Date.now() + 6000;
    } else {
      maneuverRefusal(result);
    }
    renderManeuvers(lastData || {});
  }

  /* Кнопка включает движение до СТОП; активный ползунок сразу отправляет
   * новое число без пересчёта. Heartbeat нужен только для watchdog платы. */
  function hasServoTargets() {
    return SERVO_IDS.some((id) => servoTargets[id] !== 0);
  }

  function servoLinksReady(data) {
    const drive = (data || {}).drive || {};
    const servo = drive.servo || {};
    const states = servo.perModule || {};
    return !data.estop && !!drive.available && SERVO_IDS.every((id) =>
      !(servoRuns.has('ALL') || servoRuns.has(id) || servoTargets[id]) ||
      !!(states[id] && states[id].canDrive));
  }

  function syncServoSliders() {
    SERVO_IDS.forEach((id) => {
      const slider = $('drive-module-' + id);
      if (slider) slider.value = String(servoSpeeds[id]);
      setText($('drive-module-value-' + id), String(servoSpeeds[id]));
    });
    const common = $('drive-speed');
    if (common) common.value = String(servoCommonSpeed);
    setText($('drive-speed-value'), String(servoCommonSpeed));
  }

  function setDriveMessage(text) {
    driveMessage = {text: text, until: Date.now() + 6000};
    setText($('drive-status'), text);
  }

  function renderServoButtons(id, enabled) {
    const run = servoRuns.get(id);
    const incompatible = id === 'ALL' ? servoRuns.size > 0 && !run : servoRuns.has('ALL');
    ['backward', 'forward'].forEach((direction) => {
      const button = $('direct-drive-' + (direction === 'forward' ? 'forward' : 'back') + '-' + id);
      if (!button) return;
      const pressed = !!run && run.direction === direction;
      button.disabled = !enabled || incompatible || (!!run && !pressed);
      button.classList.toggle('is-active', pressed);
      button.setAttribute('aria-pressed', String(pressed));
    });
  }

  function renderDriveControls(data) {
    const status = $('drive-status');
    const speed = $('drive-speed');
    if (!status || !speed) return;
    const d = data || {};
    const drive = d.drive || {};
    const servo = drive.servo || null;
    const enabled = !!servo && !!drive.available && steeringAuthorized;
    const canDrive = enabled && !!servo.canDrive && !d.estop;
    speed.max = String(SERVO_MAX);
    speed.disabled = !canDrive || (servoRuns.size > 0 && !servoRuns.has('ALL'));
    renderServoButtons('ALL', !!canDrive);
    const stop = $('drive-stop');
    if (stop) stop.disabled = !drive.available || !steeringAuthorized;
    const message = driveMessage && driveMessage.until > Date.now() ? driveMessage.text : '';
    const run = servoRuns.get('ALL');
    if (message) status.textContent = message;
    else if (!servo) status.textContent = 'Прямая тяга доступна при RC_SLAM_SOURCE=master.';
    else if (d.estop) status.textContent = DRIVE_REASON_TEXT.estop;
    else if (!steeringAuthorized) status.textContent = 'Управление временно недоступно.';
    else if (!servo.canDrive) status.textContent = DRIVE_REASON_TEXT[servo.reason] || 'Нет связи с платами.';
    else if (run) status.textContent = (run.direction === 'forward' ? 'Вперёд' : 'Назад') + ' · движение включено, нажмите СТОП для остановки.';
    else if (servoRuns.size) status.textContent = 'Отдельные колёса · движение включено, нажмите СТОП для остановки.';
    else status.textContent = 'Задайте скорость и нажмите «Вперёд» или «Назад».';
    status.dataset.ready = String(!!canDrive);
    syncServoSliders();
    // Это подтверждение значения самой платы, а не измерение оборотов или D3.
    const reports = SERVO_IDS.map((id) => {
      const state = servo && (servo.perModule || {})[id];
      const motor = (d.motors || []).find((item) => item && item.id === id);
      const value = motor && motor.driveServoValue;
      const fresh = state && state.canDrive && motor && motor.online !== false;
      return id + ': ' + (fresh && Number.isInteger(value) ? value : '—');
    });
    setText($('drive-servo-reported'), 'Платы сообщили Servo.write: ' + reports.join(' · ') + ' (не обороты)');
    const rpms = SERVO_IDS.map((id) => {
      const motor = (d.motors || []).find((item) => item && item.id === id);
      return id + ': ' + rpmText(motor);
    });
    setText($('drive-rpm-reported'), 'Обороты SC, об/мин: ' + rpms.join(' · '));
    SERVO_IDS.forEach((id) => renderModuleDriveCard(id, d, !!servo));
    renderManeuvers(d);
  }

  function renderModuleDriveCard(id, data, supported) {
    const stateEl = $('drive-module-state-' + id);
    const outputEl = $('drive-module-output-' + id);
    const slider = $('drive-module-' + id);
    const stop = $('drive-module-stop-' + id);
    if (!stateEl && !slider) return;
    const drive = data.drive || {};
    const servo = drive.servo || {};
    const state = (servo.perModule || {})[id] || null;
    const motor = ((data.motors || []).find((m) => m && m.id === id)) || null;
    const reported = motor && motor.driveServoValue != null ? Number(motor.driveServoValue) : null;
    setText($('drive-module-rpm-' + id), rpmText(motor));
    const canDrive = supported && !!drive.available && steeringAuthorized &&
      !data.estop && !!state && !!state.canDrive;
    if (slider) {
      slider.max = String(SERVO_MAX);
      slider.disabled = !canDrive || servoRuns.has('ALL');
    }
    renderServoButtons(id, !!canDrive);
    if (stop) stop.disabled = !drive.available || !steeringAuthorized;
    const card = stateEl && stateEl.closest ? stateEl.closest('.steer-drive') : null;
    if (card) card.dataset.active = String(!!servoTargets[id] || !!reported);
    const note = moduleDriveNote[id];
    if (note && note.until <= Date.now()) delete moduleDriveNote[id];
    const run = servoRuns.get(id) || servoRuns.get('ALL');
    let text;
    if (note && note.until > Date.now()) text = note.text;
    else if (!supported) text = 'прямая тяга недоступна';
    else if (data.estop) text = DRIVE_REASON_TEXT.estop;
    else if (!steeringAuthorized) text = 'управление недоступно';
    else if (!state) text = 'нет телеметрии';
    else if (!state.canDrive) text = DRIVE_REASON_TEXT[state.reason] || 'не готово';
    else if (run) text = (run.direction === 'forward' ? 'Вперёд' : 'Назад') + ' · движение включено';
    else text = reported ? 'Servo.write(' + reported + ')' : 'стоит';
    if (stateEl) stateEl.textContent = text;
    if (outputEl) {
      outputEl.textContent = reported != null && Number.isFinite(reported)
        ? 'Servo.write(' + reported + ')' : '—';
    }
  }

  function startServoHeartbeat() {
    if (!hasServoTargets()) {
      if (driveTimer) clearInterval(driveTimer);
      driveTimer = null;
      return;
    }
    if (driveTimer) return;
    driveLastTick = Date.now();
    driveTimer = setInterval(() => {
      const now = Date.now();
      if (now - driveLastTick > 600) {
        void stopDrive();
        setDriveMessage('Связь прервалась · нажмите кнопку заново.');
        return;
      }
      driveLastTick = now;
      servoRuns.forEach((run, id) => {
        const value = id === 'ALL' ? servoCommonSpeed : servoSpeeds[id];
        if (value) void queueServo(id, value, run.direction);
      });
    }, 200);
  }

  function setServoTarget(id, value, direction) {
    const wheels = id === 'ALL' ? SERVO_IDS : [id];
    wheels.forEach((wheel) => {
      servoTargets[wheel] = value;
      servoDirections[wheel] = direction;
    });
    void queueServo(id, value, direction);
    startServoHeartbeat();
  }

  function setServoSpeed(id, value) {
    if (!Number.isInteger(value) || value < 0 || value > SERVO_MAX) return;
    driveMessage = null;
    const wheels = id === 'ALL' ? SERVO_IDS : [id];
    if (id === 'ALL') servoCommonSpeed = value;
    wheels.forEach((wheel) => { servoSpeeds[wheel] = value; delete moduleDriveNote[wheel]; });
    const run = servoRuns.get(id);
    if (run) {
      // Ноль снимает тягу и не оставляет скрытого «взведённого» направления:
      // последующее перемещение ползунка само не должно запускать мотор.
      if (value === 0) stopServoRun(id);
      else setServoTarget(id, value, run.direction);
    }
    syncServoSliders();
    renderDriveControls(lastData || {});
  }

  function startServoRun(id, direction) {
    const data = lastData || {};
    const drive = data.drive || {};
    const servo = drive.servo || {};
    const state = id === 'ALL' ? servo : (servo.perModule || {})[id];
    if (!steeringAuthorized || document.hidden || data.estop || !drive.available || !state || !state.canDrive || servoRuns.has(id)) return;
    if ((id === 'ALL' && servoRuns.size) || (id !== 'ALL' && servoRuns.has('ALL'))) return;
    const value = id === 'ALL' ? servoCommonSpeed : servoSpeeds[id];
    if (value === 0) {
      const text = 'Задайте скорость больше нуля и нажмите направление.';
      if (id === 'ALL') setDriveMessage(text);
      else moduleDriveNote[id] = {text: text, until: Date.now() + 6000};
      renderDriveControls(data);
      return;
    }
    driveMessage = null;
    if (id === 'ALL') SERVO_IDS.forEach((wheel) => { servoSpeeds[wheel] = value; });
    servoRuns.set(id, {direction: direction});
    setServoTarget(id, value, direction);
    renderDriveControls(data);
  }

  function stopServoRun(id, force) {
    // При общей кнопке останов любого отдельного колеса безопасно гасит все.
    if (id !== 'ALL' && servoRuns.has('ALL')) { void stopDrive(true); return; }
    const run = servoRuns.get(id);
    if (!run && !force) return;
    servoRuns.delete(id);
    setServoTarget(id, 0, run ? run.direction : 'forward');
    renderDriveControls(lastData || {});
  }

  function releaseDrive() {
    if (servoRuns.size || hasServoTargets() || driveTimer) void stopDrive();
  }

  async function stopDrive(force) {
    if (window.RCRemoteControls && window.RCRemoteLegacyStop) window.RCRemoteLegacyStop();
    const wasDriving = servoRuns.size || hasServoTargets() || !!driveTimer;
    servoRuns.clear();
    SERVO_IDS.forEach((id) => { servoTargets[id] = 0; });
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    syncServoSliders();
    renderDriveControls(lastData || {});
    if (!wasDriving && !force) return;
    // Ноль вытесняет ожидающие ненулевые команды, а b принимают и старые платы.
    await queueServo('ALL', 0, 'forward');
  }

  function stopModuleDrive(id) {
    stopServoRun(id, true);
  }

  function queueServo(id, value, direction) {
    const target = {value: value, direction: direction || 'forward'};
    if (id === 'ALL') {
      servoPending.clear();
      servoPending.set(id, target);
    } else {
      // Не дать ожидающей общей команде перезаписать новое состояние колеса.
      if (servoPending.has('ALL')) {
        servoPending.delete('ALL');
        SERVO_IDS.forEach((wheel) => servoPending.set(wheel, {
          value: servoTargets[wheel], direction: servoDirections[wheel]
        }));
      }
      servoPending.delete(id);
      if (value === 0) {
        const rest = Array.from(servoPending.entries());
        servoPending.clear();
        servoPending.set(id, target);
        rest.forEach(([wheel, pending]) => servoPending.set(wheel, pending));
      } else servoPending.set(id, target);
    }
    return flushServoQueue();
  }

  function flushServoQueue() {
    if (driveSending) return driveFlush;
    driveSending = true;
    driveFlush = (async () => {
      while (servoPending.size) {
        const [id, target] = servoPending.entries().next().value;
        servoPending.delete(id);
        const payload = id === 'ALL' ? {value: target.value} : {module: id, value: target.value};
        if (target.value) payload.direction = target.direction;
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 550);
        const headers = {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'};
        // После таймаута или СТОП запоздавшая команда не должна снова
        // включать мотор. Экран и сервер на одном бортовом компьютере.
        if (target.value) headers['X-Robot-Command-Deadline'] = String(Date.now() / 1000 + 0.5);
        try {
          const response = await fetch('api/kiosk_drive', {
            method: 'POST', cache: 'no-store', signal: controller.signal,
            headers: headers,
            body: JSON.stringify(payload)
          });
          const body = await response.json().catch(() => ({}));
          if (!response.ok || !body.ok) {
            const reason = body.reason || (response.status === 401 ? 'pin_required' : 'command_failed');
            if (response.status === 401 || reason === 'pin_required') steeringAuthorized = false;
            failServoCommand(id, target.value, DRIVE_REASON_TEXT[reason] || 'Команда тяги отклонена.');
          }
        } catch (error) {
          failServoCommand(id, target.value, 'Связь потеряна · watchdog остановит колесо.');
        } finally {
          clearTimeout(timeout);
        }
      }
    })().finally(() => {
      driveSending = false;
      if (servoPending.size) void flushServoQueue();
    });
    return driveFlush;
  }

  function failServoCommand(id, value, text) {
    if (id === 'ALL') setDriveMessage(text);
    else moduleDriveNote[id] = {text: text, until: Date.now() + 6000};
    if (value !== 0) {
      const stopAll = id === 'ALL' || servoRuns.has('ALL');
      if (stopAll) {
        servoRuns.clear();
        SERVO_IDS.forEach((wheel) => { servoTargets[wheel] = 0; });
      } else {
        servoRuns.delete(id);
        servoTargets[id] = 0;
      }
      void queueServo(stopAll ? 'ALL' : id, 0, 'forward');
      syncServoSliders();
      startServoHeartbeat();
    }
    renderDriveControls(lastData || {});
  }

  function bindServoButton(id, direction) {
    const button = $('direct-drive-' + (direction === 'forward' ? 'forward' : 'back') + '-' + id);
    if (!button) return;
    // Обычный click одинаково работает с мышью, сенсорным экраном и клавиатурой.
    // Повторное нажатие текущего направления не останавливает движение.
    button.addEventListener('click', () => {
      if (!button.disabled) startServoRun(id, direction);
    });
  }

  function initSteeringControls() {
    const panel = $('swipe-motors-view');
    if (!panel) return;
    const ids = ['FL', 'FR', 'RL', 'RR'];
    const driveSpeed = $('drive-speed');
    if (driveSpeed) driveSpeed.addEventListener('input', () => {
      if (!driveSpeed.disabled) setServoSpeed('ALL', Number(driveSpeed.value));
    });
    bindServoButton('ALL', 'forward');
    bindServoButton('ALL', 'backward');
    const driveStopButton = $('drive-stop');
    if (driveStopButton) driveStopButton.addEventListener('click', () => { void stopDrive(true); });

    // У каждого колеса свои скорость и кнопки направления.
    ids.forEach((id) => {
      const slider = $('drive-module-' + id);
      if (slider) slider.addEventListener('input', () => {
        if (!slider.disabled) setServoSpeed(id, Number(slider.value));
      });
      bindServoButton(id, 'forward');
      bindServoButton(id, 'backward');
      const stopButton = $('drive-module-stop-' + id);
      if (stopButton) stopButton.addEventListener('click', () => stopModuleDrive(id));
    });

    // Манёвры остаются отдельными от прямого управления Servo.
    MANEUVER_ORDER.forEach((id) => {
      const button = $('maneuver-' + id);
      if (button) button.addEventListener('click', () => { void startManeuver(id); });
    });
    const maneuverStop = $('maneuver-stop');
    if (maneuverStop) maneuverStop.addEventListener('click', () => { void stopManeuver(); });
    // Отпускание больше не останавливает движение. При уходе со страницы,
    // потере фокуса или связи тяга снимается; watchdog страхует обрыв.
    window.addEventListener('blur', releaseDrive);
    window.addEventListener('pagehide', releaseDrive);
    document.addEventListener('visibilitychange', () => { if (document.hidden) releaseDrive(); });

    // Не даём горизонтальному движению по слайдеру закрыть swipe-панель.
    ['pointerdown', 'pointermove', 'pointerup', 'mousedown', 'mouseup'].forEach((name) => {
      panel.addEventListener(name, (event) => {
        if (event.target.closest && event.target.closest('input, button, select, textarea, .steer-dial, .cam-viewport, .map-canvas-wrap')) event.stopPropagation();
      });
    });
    ['touchstart', 'touchmove', 'touchend'].forEach((name) => {
      panel.addEventListener(name, (event) => {
        if (event.target.closest && event.target.closest('input, button, select, textarea, .steer-dial, .cam-viewport, .map-canvas-wrap')) event.stopPropagation();
      }, {passive: true});
    });

    function markDirty(id, value) {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      const output = $('steer-target-' + id);
      if (!card || !slider) return;
      const angle = steerClamp(value);
      if (angle === null) return;
      slider.value = String(angle);
      card.dataset.steerDirty = 'true';
      delete steerResult[id];
      if (output) output.textContent = angle + '°';
      steerPreviewAngles[id] = angle;
      steerPreviewMode = 'manual';
      // Без этого схема «замерзала» на ручном углу навсегда: после одного
      // касания ползунка экран переставал показывать настоящие углы колёс.
      scheduleSteerPreviewLive();
      updateSteerDiagram();
      renderSteeringControls(lastData || {});
    }

    async function sendSteer(id) {
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const slider = $('steer-slider-' + id);
      if (!card || !slider) return;
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const angle = steerClamp(slider.value);
      if (angle === null) return;
      // Колесо уже стоит там, куда просит ползунок, — команда не нужна (иначе
      // каждый щелчок по дорожке ползунка уходил в плату и возвращал «модуль
      // занят»). Если фактический угол расходится с целью — отправляем: это
      // рабочий способ докрутить колесо, которое не доехало.
      const motor = ((lastData && lastData.motors) || []).find((m) => m && m.id === id) || null;
      const currentAngle = motor && Number.isFinite(+motor.angle) ? +motor.angle : null;
      if (motor && currentAngle !== null && !motor.moving && Math.abs(currentAngle - angle) <= 1) {
        card.dataset.steerDirty = 'false';
        steerResult[id] = { text: 'колесо уже на ' + angle + '°', error: false, until: Date.now() + 4000 };
        renderSteeringControls(lastData || {});
        return;
      }
      const sendButton = panel.querySelector('[data-steer-send="' + id + '"]');
      if (sendButton) sendButton.disabled = true;
      try {
        const response = await fetch('api/kiosk_steer', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id, angle })
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) {
            setSteeringAuthorized(false);
          }
          steerResult[id] = { text: errorText(result && result.reason, result), error: true, until: Date.now() + 6000 };
          renderSteeringControls(lastData || {});
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        pendingSteer[id] = { angle: Number(result.angle), sentAt: Date.now() };
        card.dataset.steerDirty = 'false';
        steerResult[id] = { text: 'команда передана · без тяги', error: false, until: Date.now() + 6000 };
        renderSteeringControls(lastData || {});
      } catch (error) {
        steerResult[id] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
        renderSteeringControls(lastData || {});
      }
    }

    async function setSoftwareZero(id) {
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const confirmed = window.confirm(
        'Запасной вариант без датчика: сначала физически выставьте колесо ' + id + ' прямо. Команда не двигает мотор — она назначит текущую позицию за 0°. Ноль потеряется после перезапуска Arduino. Продолжить?'
      );
      if (!confirmed) return;
      const button = panel.querySelector('[data-steer-origin="' + id + '"]');
      if (button) button.disabled = true;
      try {
        const response = await fetch('api/kiosk_zero', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id })
        });
        const result = await response.json();
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) setSteeringAuthorized(false);
          steerResult[id] = { text: errorText(result && result.reason, result), error: true, until: Date.now() + 6000 };
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        pendingZero[id] = { sentAt: Date.now() };
        steerResult[id] = { text: 'программный ноль отправлен', error: false, until: Date.now() + 6000 };
      } catch (error) {
        steerResult[id] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
      } finally {
        renderSteeringControls(lastData || {});
      }
    }

    async function startCalibration(id) {
      if (!steeringAuthorized) {
        if (window._openSteeringAuth) window._openSteeringAuth();
        return;
      }
      const all = id === 'ALL';
      const confirmed = window.confirm(
        (all ? 'Все четыре колеса' : 'Колесо ' + id) +
        ' будет искать оптодатчик D18 в пределах ±135° от текущего положения (сначала вправо, затем влево) ' +
        'и встанет в ноль по среднему трёх подходов к каждой кромке. После включения Arduino колёса должны ' +
        'стоять примерно прямо (±30°). Колёса не должны упираться, рядом не должно быть людей. ' +
        'Тяга во время калибровки запрещена. Начать?'
      );
      if (!confirmed) return;
      const targets = all ? ids : [id];
      const button = all ? $('steer-calib-all') : panel.querySelector('[data-steer-calib="' + id + '"]');
      if (button) button.disabled = true;
      try {
        const response = await fetch('api/kiosk_calibrate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id })
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result || !result.ok) {
          if (response.status === 401 || (result && result.reason === 'pin_required')) setSteeringAuthorized(false);
          targets.forEach((target) => {
            steerResult[target] = { text: errorText(result && result.reason, result), error: true, until: Date.now() + 6000 };
          });
          if (response.status === 401 && window._openSteeringAuth) window._openSteeringAuth();
          return;
        }
        const offline = Array.isArray(result.offline) ? result.offline : [];
        targets.filter((target) => !offline.includes(target)).forEach((target) => {
          pendingCalib[target] = { sentAt: Date.now(), started: false };
          delete steerResult[target];
        });
        offline.forEach((target) => {
          steerResult[target] = { text: 'нет связи · не калибруется', error: true, until: Date.now() + 6000 };
        });
      } catch (error) {
        targets.forEach((target) => {
          steerResult[target] = { text: 'нет связи с сервером', error: true, until: Date.now() + 6000 };
        });
      } finally {
        renderSteeringControls(lastData || {});
      }
    }

    const masterSlider = $('steer-master-slider');
    if (masterSlider) {
      masterSlider.addEventListener('input', () => {
        const angle = steerClamp(masterSlider.value);
        if (angle === null) return;
        const angles = {};
        STEER_IDS.forEach((id) => { angles[id] = angle; });
        previewAllCards(angles);
        scheduleSteerPreviewLive();
      });
    }
    const applyAllButton = $('steer-apply-all');
    if (applyAllButton) applyAllButton.addEventListener('click', () => { void applyMasterAngles(); });
    buildSteerPresetButtons();

    const calibAllButton = $('steer-calib-all');
    if (calibAllButton) calibAllButton.addEventListener('click', () => { void startCalibration('ALL'); });

    ids.forEach((id) => {
      const slider = $('steer-slider-' + id);
      const card = panel.querySelector('[data-steer-card="' + id + '"]');
      const calib = panel.querySelector('[data-steer-calib="' + id + '"]');
      if (calib) calib.addEventListener('click', () => { void startCalibration(id); });
      if (window.RCSteerDial) window.RCSteerDial.attach(slider);
      // Круговой регулятор двигает цель; команда уходит, когда палец отпущен
      // (change), — одна команда на жест, а не поток при каждом шаге.
      if (slider) {
        slider.addEventListener('input', () => markDirty(id, slider.value));
        slider.addEventListener('change', () => { void sendSteer(id); });
      }
      panel.querySelectorAll('[data-steer-adjust="' + id + '"]').forEach((button) => {
        button.addEventListener('click', () => {
          markDirty(id, Number(slider.value) + Number(button.dataset.step));
          void sendSteer(id);
        });
      });
      const zero = panel.querySelector('[data-steer-zero="' + id + '"]');
      if (zero) zero.addEventListener('click', () => { markDirty(id, 0); void sendSteer(id); });
      const origin = panel.querySelector('[data-steer-origin="' + id + '"]');
      if (origin) origin.addEventListener('click', () => setSoftwareZero(id));
      const send = panel.querySelector('[data-steer-send="' + id + '"]');
      if (send) send.addEventListener('click', () => sendSteer(id));
      if (card) card.dataset.steerDirty = 'false';
    });

    renderSteeringControls(lastData || {});
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
