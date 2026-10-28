/* PIN-защищённая адресная панель поворота для веб-интерфейса оператора. */
(function () {
  'use strict';

  if (window.RCSteeringPanel) return;
  const IDS = ['FL', 'FR', 'RL', 'RR'];
  const WHEEL_NAMES = {FL:'Переднее левое', FR:'Переднее правое', RL:'Заднее левое', RR:'Заднее правое'};
  /* Манёвры: цель в метрах или градусах, а не «держать процент». Сервер сам
   * ставит углы, плавно набирает тягу, меряет путь по тахометру и гасит её на
   * цели. Здесь только кнопки и числа: правки уходят в api/kiosk_maneuver_config
   * и живут на борту, поэтому борт и пульт показывают одни и те же числа. */
  const MANEUVERS = [
    {id:'forward', title:'Вперёд', unit:'м', key:'distanceM', min:0.05, max:20, step:0.05},
    {id:'backward', title:'Назад', unit:'м', key:'distanceM', min:0.05, max:20, step:0.05},
    {id:'left', title:'Боком влево', unit:'м', key:'distanceM', min:0.05, max:20, step:0.05},
    {id:'right', title:'Боком вправо', unit:'м', key:'distanceM', min:0.05, max:20, step:0.05},
    {id:'spin_left', title:'Разворот влево', unit:'°', key:'angleDeg', min:5, max:360, step:5},
    {id:'spin_right', title:'Разворот вправо', unit:'°', key:'angleDeg', min:5, max:360, step:5}
  ];
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
    pin_required: 'PIN истёк — разблокируйте управление снова.',
    estop: 'Включён E-STOP: манёвр заблокирован.',
    control_not_supported: 'Манёвры доступны только в режиме master.',
    request_header_required: 'Запрос без заголовка команды.',
    command_failed: 'Сервер не принял команду.'
  };
  /** Запасной предел угла, пока сервер не ответил; совпадает с прошивкой. */
  const FALLBACK_LIMIT_DEG = 90;
  let limitDeg = FALLBACK_LIMIT_DEG;
  let authorized = false;
  let snapshot = null;
  let pollTimer = null;
  let driveTimer = null;
  let driveDirection = 0;
  let driveQueued = null;
  let driveSending = false;
  let driveFlush = Promise.resolve();
  let presetBusy = false;
  let presetMessage = null;
  const pending = Object.create(null);
  const feedback = Object.create(null);
  const sending = Object.create(null);
  // Сколько даём колесу доехать и остановиться, прежде чем признать команду
  // неисполненной: 90° — это 1,2 с хода плюс доводка по потенциометру.
  const PENDING_SETTLE_MS = 4000;
  // Аварийный срок, если телеметрия от колеса вообще перестала приходить.
  const PENDING_TELEMETRY_TIMEOUT_MS = 15000;
  const $ = (id) => document.getElementById(id);

  function clampAngle(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return null;
    if (window.RCSteerPresets && typeof window.RCSteerPresets.clampAngle === 'function') {
      return window.RCSteerPresets.clampAngle(n);
    }
    return Math.max(-limitDeg, Math.min(limitDeg, Math.round(n)));
  }

  /**
   * Предел хода руля приходит с сервера (steeringControl.limitDeg) и
   * расставляется по ползункам, подписям и пресетам. Зашитое в шаблон число
   * расходилось с прошивкой — экран разрешал угол, который плата отклоняла.
   */
  function syncLimit(control) {
    const presets = window.RCSteerPresets;
    const reported = Number(control && control.limitDeg);
    const next = Number.isFinite(reported) && reported > 0 ? Math.round(reported) : FALLBACK_LIMIT_DEG;
    if (presets && typeof presets.setLimit === 'function') presets.setLimit(next);
    const changed = next !== limitDeg;
    limitDeg = next;
    const text = presets && typeof presets.limitText === 'function' ? presets.limitText() : ('±' + next + '°');
    document.querySelectorAll('[data-rc-steer-limit]').forEach((element) => { element.textContent = text; });
    const ranges = [$('rc-preset-angle')].concat(
      IDS.map((id) => document.querySelector('[data-rc-steering="' + id + '"] [data-range]')));
    ranges.forEach((range) => {
      if (!range) return;
      range.min = String(-next);
      range.max = String(next);
      const value = clampAngle(range.value);
      if (value !== null && Number(range.value) !== value) range.value = String(value);
    });
    if (changed) buildPresetButtons();
    return changed;
  }

  function addUi() {
    if ($('rc-steering-launch')) return;
    const launch = document.createElement('button');
    launch.id = 'rc-steering-launch';
    launch.className = 'rc-steering-launch';
    launch.type = 'button';
    launch.textContent = 'Руление и тяга';
    launch.title = 'Открыть вкладку управления через бортовой сервер';

    const overlay = document.createElement('div');
    overlay.id = 'rc-steering-overlay';
    overlay.className = 'rc-steering-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-steering-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-steering-title">
        <header class="rc-steering-head">
          <div><h2 id="rc-steering-title">Управление приводами</h2>
            <p>Рулевые шаговые моторы; кнопки движения — в самом низу окна</p></div>
          <button type="button" class="rc-steering-close" aria-label="Закрыть">×</button>
        </header>
        <div class="rc-steering-safety" role="note"><b>Ноль руля — по оптодатчику D18:</b> «Калибровка» ищет оптодатчик в пределах ±135° (ход руля 270°) и ставит ноль в середину флажка по среднему трёх подходов к каждой кромке. После включения Arduino выставьте колёса прямо руками (±30°). «Задать 0°» — запасной ручной вариант без датчика. Ноль теряется после перезапуска Arduino.</div>
        <div class="rc-steering-toolbar">
          <span id="rc-steering-auth">Нужен PIN</span>
          <button type="button" class="rc-steering-btn" id="rc-steering-unlock">Разблокировать</button>
          <button type="button" class="rc-steering-btn" id="rc-steering-lock" hidden>Заблокировать</button>
          <span id="rc-steering-source" role="status">Проверка связи…</span>
        </div>
        <section class="rc-presets" aria-label="Пресеты руления">
          <div class="rc-presets-head">
            <div><b>Быстрые режимы</b><small>Пресет выставляет углы всех четырёх колёс в пределах <span data-rc-steer-limit>±100°</span>; тяга — кнопками «Движение» в самом низу окна.</small></div>
            <span id="rc-preset-status" role="status">Проверка связи…</span>
          </div>
          <div id="rc-preset-grid" class="rc-preset-grid"></div>
          <div class="rc-preset-all">
            <label for="rc-preset-angle">Угол всех колёс <b><span id="rc-preset-angle-value">0</span>°</b>
              <input id="rc-preset-angle" type="range" min="-100" max="100" step="1" value="0" disabled /></label>
            <button type="button" class="rc-steering-btn" id="rc-preset-apply" disabled>Задать всем</button>
            <span class="rc-preset-scale"><span>← влево</span><span>прямо</span><span>вправо →</span></span>
          </div>
        </section>
        <div class="rc-steering-grid">${IDS.map((id) => `
          <article class="rc-steering-card" data-rc-steering="${id}">
            <header><b>${id}<small>${WHEEL_NAMES[id]}</small></b><span data-status>нет связи</span></header>
            <div class="rc-steering-values"><span>Сейчас <b data-current>—</b></span><span>Цель <b data-target>0°</b></span></div>
            <input data-range type="range" min="-100" max="100" step="1" value="0" aria-label="${id}, целевой угол" disabled>
            <div class="rc-steering-actions">
              <button type="button" data-calib disabled>Калибровка</button>
              <button type="button" data-origin disabled>Задать 0°</button>
              <button type="button" data-step="-5" disabled>−5°</button>
              <button type="button" data-center disabled>0°</button>
              <button type="button" data-step="5" disabled>+5°</button>
              <button type="button" class="primary" data-send disabled>Повернуть</button>
            </div>
            <small data-message aria-live="polite"></small>
          </article>`).join('')}
        </div>
        <section class="rc-drive-panel" aria-label="Тяговые моторы">
          <div class="rc-drive-top"><div><b>Движение</b><small>Зажмите направление; отпустите для торможения</small></div>
            <span id="rc-drive-status" role="status">Нужен ноль всех колёс (калибровка)</span></div>
          <label class="rc-drive-speed">Ограничение скорости <b><span id="rc-drive-speed-value">8</span>%</b>
            <input id="rc-drive-speed" type="range" min="1" max="100" value="8" disabled></label>
          <div class="rc-drive-actions">
            <button type="button" data-drive-direction="-1" disabled>Зажать: назад</button>
            <button type="button" data-drive-direction="1" disabled>Зажать: вперёд</button>
            <button type="button" id="rc-drive-stop" class="stop" disabled>СТОП</button>
          </div>
          <small id="rc-drive-mode" class="rc-drive-mode">Привод: —</small>
          <section class="rc-maneuver-panel" aria-label="Манёвры тяги">
            <div class="rc-maneuver-top"><b>Манёвры</b><small>цель в метрах или градусах — сервер ведёт сам</small></div>
            <div class="rc-maneuver-list">${MANEUVERS.map((item) => `
              <div class="rc-maneuver-row" data-rc-maneuver="${item.id}">
                <b>${item.title}</b>
                <label>${item.unit}<input type="number" data-maneuver-target min="${item.min}" max="${item.max}" step="${item.step}" value="${item.key === 'distanceM' ? 1 : 90}" aria-label="${item.title}: цель"></label>
                <label>%<input type="number" data-maneuver-percent min="1" max="100" step="1" value="10" aria-label="${item.title}: рабочий процент"></label>
                <button type="button" data-maneuver-start disabled>Ехать</button>
              </div>`).join('')}
            </div>
            <div class="rc-maneuver-actions">
              <button type="button" id="rc-maneuver-stop" class="stop" disabled>СТОП манёвра</button>
              <small id="rc-maneuver-status" role="status">манёвры недоступны</small>
            </div>
          </section>
          <small class="rc-drive-warning">Нейтраль и размах серво-ШИМ, тахометр, тормозная линия и линия STOP настраиваются на экране «Руление и тяга» (вкладка /controls): настройка живёт в EEPROM платы. Программный стоп и watchdog (650 мс), не аварийный выключатель. Перед запуском вывесьте колёса и установите аппаратную аварийную кнопку в силовую цепь.</small>
        </section>
      </section>`;
    const header = document.querySelector('.top-status, .head-actions');
    if (header) header.append(launch);
    else document.body.append(launch);
    document.body.append(overlay);

    launch.addEventListener('click', () => {
      const url = new URL('/controls', window.location.href);
      const token = new URLSearchParams(window.location.search).get('st');
      if (token) url.searchParams.set('st', token);
      window.location.assign(url.toString());
    });
    overlay.querySelector('.rc-steering-close').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    overlay.querySelector('#rc-steering-unlock').addEventListener('click', unlock);
    overlay.querySelector('#rc-steering-lock').addEventListener('click', lock);
    IDS.forEach((id) => bindCard(id));
    bindDriveControls();
    buildPresetButtons();
    const presetAngle = $('rc-preset-angle');
    if (presetAngle) presetAngle.addEventListener('input', () => {
      const label = $('rc-preset-angle-value');
      if (label) label.textContent = String(presetAngle.value);
    });
    const presetApply = $('rc-preset-apply');
    if (presetApply) presetApply.addEventListener('click', () => { void applyPresetAngle(); });
    window.addEventListener('pointerup', releaseDrive);
    window.addEventListener('pointercancel', releaseDrive);
    window.addEventListener('blur', releaseDrive);
    document.addEventListener('visibilitychange', () => { if (document.hidden) releaseDrive(); });
  }

  function close() {
    const overlay = $('rc-steering-overlay');
    if (overlay) overlay.hidden = true;
    if (pollTimer) clearInterval(pollTimer);
    pollTimer = null;
    presetBusy = false;
    releaseDrive();
    void lock();
  }

  async function json(url, init) {
    const response = await fetch(url, { cache: 'no-store', ...init });
    const body = await response.json().catch(() => ({}));
    return { response, body };
  }

  async function refresh() {
    const overlay = $('rc-steering-overlay');
    if (!overlay || overlay.hidden) return;
    try {
      const [{ body: state }, { body: auth }] = await Promise.all([
        json('api/kiosk_state'), json('api/kiosk_lock')
      ]);
      if (!state.ok || !state.data) throw new Error('Нет телеметрии');
      snapshot = state.data;
      authorized = !!auth.steeringAuthorized;
      $('rc-steering-auth').textContent = authorized ? 'PIN подтверждён' : 'Нужен PIN';
      $('rc-steering-unlock').hidden = authorized;
      $('rc-steering-lock').hidden = !authorized;
      const control = snapshot.steeringControl || {};
      syncLimit(control);
      const source = $('rc-steering-source');
      source.textContent = !control.available
        ? 'Управление доступно при RC_SLAM_SOURCE=master и подключённом USB мастера.'
        : snapshot.linkOk === false
          ? 'Мастер выбран, но свежей телеметрии пока нет.'
          : control.calibration
            ? 'Мастер на связи · откалибруйте колёса по датчику перед рулением и тягой.'
            : 'Мастер на связи · задайте ручной ноль перед рулением и тягой.';
      if (window.RCPotMode) window.RCPotMode.render(document.querySelector('.rc-steering-toolbar'), snapshot, authorized);
      IDS.forEach((id) => renderCard(id));
      renderDrive();
      renderPresets();
    } catch (error) {
      $('rc-steering-source').textContent = error && error.message ? error.message : 'Нет связи с сервером.';
      IDS.forEach((id) => renderCard(id));
      renderDrive();
    }
  }

  function presetList() {
    if (!window.RCSteerPresets) return [];
    if (typeof window.RCSteerPresets.list === 'function') return window.RCSteerPresets.list();
    return Array.isArray(window.RCSteerPresets.PRESETS) ? window.RCSteerPresets.PRESETS : [];
  }

  function presetById(id) {
    if (window.RCSteerPresets && typeof window.RCSteerPresets.get === 'function') {
      return window.RCSteerPresets.get(id);
    }
    return presetList().find((preset) => preset.id === id) || null;
  }

  function setPresetMessage(text, error) {
    presetMessage = { text: String(text || ''), error: !!error, until: Date.now() + 10000 };
    renderPresets();
  }

  function presetUniformAngle() {
    const motors = snapshot && Array.isArray(snapshot.motors) ? snapshot.motors : [];
    let common = null;
    for (const id of IDS) {
      const motor = motors.find((item) => item && item.id === id);
      const raw = motor && motor.targetAngle != null && Number.isFinite(+motor.targetAngle)
        ? +motor.targetAngle
        : (motor && motor.angle != null && Number.isFinite(+motor.angle) ? +motor.angle : null);
      if (raw === null) return null;
      const value = clampAngle(raw);
      if (value === null) return null;
      if (common === null) common = value;
      else if (Math.abs(value - common) > 1) return null;
    }
    return common;
  }

  function presetBlockedReason() {
    const control = (snapshot && snapshot.steeringControl) || {};
    if (!control.available) return 'Управление доступно только в режиме master.';
    if (!authorized) return 'Сначала разблокируйте PIN.';
    const drive = (snapshot && snapshot.drive) || {};
    if (drive.active || Number(drive.commandedPercent || 0) !== 0) {
      return 'Сначала остановите тягу кнопкой СТОП.';
    }
    const motors = snapshot && Array.isArray(snapshot.motors) ? snapshot.motors : [];
    const find = (id) => motors.find((item) => item && item.id === id) || null;
    const offline = IDS.filter((id) => !(find(id) && find(id).online === true));
    if (offline.length) return 'Нет связи: ' + offline.join(', ') + '.';
    const unhomed = IDS.filter((id) => !(find(id) && find(id).homed === true));
    if (unhomed.length) return 'Не откалиброваны: ' + unhomed.join(', ') + '.';
    const moving = IDS.filter((id) => {
      const motor = find(id);
      return !!(motor && (motor.moving || motor.calibrating));
    });
    if (moving.length) return 'Дождитесь остановки: ' + moving.join(', ') + '.';
    if (presetBusy) return 'Применяю пресет…';
    if (Object.keys(pending).length) return 'Предыдущая команда ещё выполняется.';
    return '';
  }

  function buildPresetButtons() {
    const grid = $('rc-preset-grid');
    if (!grid) return;
    grid.innerHTML = '';
    const list = presetList();
    if (!list.length) {
      const note = document.createElement('p');
      note.className = 'rc-preset-note';
      note.textContent = 'Пресеты не загрузились — перезагрузите страницу.';
      grid.appendChild(note);
      return;
    }
    list.forEach((preset) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'rc-preset';
      button.dataset.rcPreset = preset.id;
      button.setAttribute('aria-pressed', 'false');
      if (!preset.available) {
        button.classList.add('is-unavailable');
        button.title = preset.reason || 'Недоступно';
      }
      const icon = document.createElement('span');
      icon.className = 'rc-preset-icon';
      icon.setAttribute('aria-hidden', 'true');
      const iconAngles = preset.available
        ? [preset.angle, preset.angle, preset.angle, preset.angle]
        : [limitDeg, -limitDeg, limitDeg, -limitDeg];
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
      button.addEventListener('click', () => { void applyPreset(presetById(preset.id) || preset); });
      grid.appendChild(button);
    });
  }

  function renderPresets() {
    const slider = $('rc-preset-angle');
    const apply = $('rc-preset-apply');
    const uniform = presetUniformAngle();
    if (slider) {
      if (!presetBusy && uniform !== null && document.activeElement !== slider) slider.value = String(uniform);
      slider.disabled = presetBusy || !authorized || !(snapshot && snapshot.steeringControl && snapshot.steeringControl.available);
      const label = $('rc-preset-angle-value');
      if (label) label.textContent = String(slider.value);
    }
    const reason = presetBlockedReason();
    if (apply) apply.disabled = presetBusy || !!reason;
    document.querySelectorAll('[data-rc-preset]').forEach((button) => {
      const preset = presetById(button.dataset.rcPreset);
      if (!preset) return;
      const active = preset.available && uniform !== null && Math.abs(uniform - preset.angle) <= 1;
      button.classList.toggle('is-active', active);
      button.setAttribute('aria-pressed', String(active));
      // Недоступный пресет остаётся нажимаемым — по нажатию показываем причину.
      button.disabled = presetBusy || (preset.available && !!reason);
      if (preset.reason) button.title = preset.reason;
    });
    const status = $('rc-preset-status');
    if (!status) return;
    if (presetMessage && presetMessage.until <= Date.now()) presetMessage = null;
    if (presetMessage) {
      status.textContent = presetMessage.text;
      status.dataset.error = String(!!presetMessage.error);
    } else if (!window.RCSteerPresets) {
      status.textContent = 'Пресеты не загрузились — перезагрузите страницу.';
      status.dataset.error = 'true';
    } else {
      status.textContent = reason || 'Колёса готовы. Выберите режим.';
      status.dataset.error = String(!!reason);
    }
  }

  async function applyAngles(angles, label) {
    if (presetBusy) return false;
    const reason = presetBlockedReason();
    if (reason) {
      setPresetMessage(reason, true);
      return false;
    }
    const targets = {};
    for (const id of IDS) {
      const angle = clampAngle(angles[id]);
      if (angle === null) {
        setPresetMessage('Не задан угол для ' + id + '.', true);
        return false;
      }
      targets[id] = angle;
    }
    presetBusy = true;
    presetMessage = { text: 'Применяю «' + label + '»…', error: false, until: Date.now() + 20000 };
    renderPresets();
    for (const id of IDS) {
      try {
        const { response, body } = await json('api/kiosk_steer', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
          body: JSON.stringify({ module: id, angle: targets[id] })
        });
        if (!response.ok || !body.ok) {
          if (response.status === 401 || body.reason === 'pin_required') authorized = false;
          const message = reasonText(body.reason, body);
          feedback[id] = { text: 'пресет: ' + message, error: true, until: Date.now() + 8000 };
          presetBusy = false;
          setPresetMessage('Остановлено на ' + id + ': ' + message + '. Остальные колёса не трогаем.', true);
          await refresh();
          return false;
        }
        pending[id] = { at: Date.now(), angle: body.angle };
        feedback[id] = { text: 'пресет · ' + targets[id] + '°', error: false, until: Date.now() + 6000 };
      } catch (error) {
        presetBusy = false;
        setPresetMessage('Нет связи с сервером на ' + id + '. Проверьте сеть.', true);
        await refresh();
        return false;
      }
    }
    presetBusy = false;
    presetMessage = {
      text: '«' + label + '»: угол ' + targets.FL + '° на всех колёсах. Тяга — кнопками «Движение» в самом низу окна.',
      error: false, until: Date.now() + 10000
    };
    await refresh();
    return true;
  }

  async function applyPreset(preset) {
    if (!preset || presetBusy) return;
    if (!preset.available) {
      setPresetMessage(preset.reason || 'Этот режим недоступен.', true);
      return;
    }
    const angles = {};
    IDS.forEach((id) => { angles[id] = preset.angle; });
    if (!authorized) {
      await unlock();
      if (!authorized) {
        setPresetMessage('Без PIN пресет не применить: нажмите «Разблокировать» и введите PIN.', true);
        return;
      }
    }
    await applyAngles(angles, preset.title);
  }

  async function applyPresetAngle() {
    const slider = $('rc-preset-angle');
    if (!slider) return;
    const angle = clampAngle(slider.value);
    if (angle === null) return;
    const angles = {};
    IDS.forEach((id) => { angles[id] = angle; });
    await applyAngles(angles, 'Угол всех колёс ' + angle + '°');
  }

  function renderCard(id) {
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    if (!card) return;
    const motor = snapshot && Array.isArray(snapshot.motors)
      ? snapshot.motors.find((item) => item && item.id === id) : null;
    const control = (snapshot && snapshot.steeringControl) || {};
    const online = !!(motor && motor.online === true && Number.isFinite(+motor.angle));
    const zeroed = !!(motor && motor.homed === true);
    const calibrating = !!(motor && motor.calibrating);
    const moving = !!(motor && (motor.moving || motor.calibrating));
    const homeSource = motor && motor.homeSource ? String(motor.homeSource) : (zeroed ? 'manual' : 'none');
    const drive = (snapshot && snapshot.drive) || {};
    const tractionActive = !!drive.active || Number(drive.commandedPercent || 0) !== 0;
    const current = online ? Number(motor.angle) : null;
    const target = online && Number.isFinite(+motor.targetAngle) ? Number(motor.targetAngle) : current;
    const entry = pending[id];
    if (entry && entry.calib && calibrating) entry.started = true;
    const steerEntry = entry && !entry.zero && !entry.calib;
    if (steerEntry && !moving) {
      // Колесо остановилось. Цель достигнута — либо команда дошла до платы и
      // колесо встало там, где смогло (упор, остаток после доводки по
      // потенциометру). Держать карточку заблокированной дальше незачем:
      // раньше она отвисала лишь через 15 с, и всё это время колесо нельзя
      // было ни тронуть, ни понять, что с ним случилось.
      const want = Number(entry.angle);
      const arrived = current != null && Math.abs(current - want) <= 1;
      const delivered = Math.abs(target - want) <= 1;
      const limit = current == null ? PENDING_TELEMETRY_TIMEOUT_MS : PENDING_SETTLE_MS;
      if (arrived || delivered || Date.now() - entry.at > limit) {
        delete pending[id];
        if (!arrived && current != null) {
          // Не доехало: колесо разблокировано, но оператор должен видеть, что
          // угол не тот — иначе тяга встанет в «руль не выровнен».
          feedback[id] = {
            text: 'не доехало до ' + want + '°: стоит на ' + current.toFixed(1) + '°',
            error: true,
            until: Date.now() + 10000,
          };
        }
      }
    } else if (entry && ((entry.zero && zeroed) || (entry.calib && entry.started && !calibrating))) {
      delete pending[id];
    }
    const activePending = pending[id] && Date.now() - pending[id].at < PENDING_TELEMETRY_TIMEOUT_MS;
    if (pending[id] && !activePending) delete pending[id];
    // Пока команда в полёте, карточка «занята»: второй запрос по тому же колесу
    // плата отклонила бы как module_busy.
    const ready = !!control.available && authorized && online && zeroed && !moving && !activePending &&
      !tractionActive && !sending[id];
    const canZero = !!control.available && authorized && online && !moving && !activePending && !tractionActive;
    const range = card.querySelector('[data-range]');
    if (!card.dataset.dirty && current != null) range.value = String(Math.round(target));
    card.querySelector('[data-current]').textContent = current == null ? '—' : current.toFixed(1) + '°';
    card.querySelector('[data-target]').textContent = Math.round(+range.value || 0) + '°';
    range.disabled = !ready;
    if (window.RCSteerDial) window.RCSteerDial.sync(range, current);
    card.querySelectorAll('[data-step], [data-center]').forEach((button) => { button.disabled = !ready; });
    card.querySelector('[data-send]').disabled = !ready || !card.dataset.dirty;
    card.querySelector('[data-origin]').disabled = !canZero;
    const calibButton = card.querySelector('[data-calib]');
    calibButton.disabled = !canZero || !control.calibration;
    calibButton.hidden = !!control.available && !control.calibration;
    const status = card.querySelector('[data-status]');
    status.textContent = !control.available ? 'нет master' : !authorized ? 'нужен PIN'
      : !online ? 'нет связи' : calibrating ? 'калибровка по датчику…' : moving ? 'движется'
        : tractionActive ? 'тяга включена · стоп' : sending[id] ? 'отправляю команду…'
          : activePending ? 'команда в очереди'
          : !zeroed ? (motor && motor.calibErrorText ? 'ошибка калибровки: ' + motor.calibErrorText
            : control.calibration ? 'нужна калибровка' : 'нужен ручной ноль')
            : homeSource === 'opto' ? 'готов · ноль по датчику' : 'готов · ноль вручную';
    const msg = feedback[id];
    card.querySelector('[data-message]').textContent = msg && msg.until > Date.now() ? msg.text : '';
    card.classList.toggle('error', !!(msg && msg.error && msg.until > Date.now()));
  }

  const DRIVE_REASON_TEXT = {
    serial_unavailable: 'Нет связи с USB-мастером.',
    module_offline: 'Нет свежей телеметрии всех четырёх модулей.',
    drive_firmware_required: 'Обновите прошивку всех четырёх плат для тяги и watchdog.',
    not_homed: 'Сначала откалибруйте колёса по датчику (или задайте 0° вручную).',
    module_busy: 'Дождитесь остановки рулевых модулей.',
    steering_misaligned: 'Для тяги колёса должны смотреть в одном направлении (±2,5°).',
    drive_limit: 'Скорость выше предела привода (см. настройку привода).',
    control_not_supported: 'Тяга доступна только в режиме master.',
    command_queue_full: 'Очередь команд занята.',
    pin_required: 'PIN истёк — разблокируйте управление снова.'
  };

  function maneuverConfigFor(id) {
    const config = (snapshot && snapshot.maneuverConfig) || null;
    const presets = (config && config.presets) || {};
    return presets[id] || null;
  }

  function renderManeuvers() {
    const stop = $('rc-maneuver-stop');
    const statusEl = $('rc-maneuver-status');
    if (!stop || !statusEl) return;
    const config = (snapshot && snapshot.maneuverConfig) || null;
    const supported = !!(config && config.supported !== false);
    const drive = (snapshot && snapshot.drive) || {};
    const state = drive.maneuver || {};
    const active = !!state.active;
    const control = (snapshot && snapshot.steeringControl) || {};
    const enabled = supported && authorized && !!control.available && !!drive.canDrive;
    const focused = document.activeElement;
    MANEUVERS.forEach((item) => {
      const row = document.querySelector('[data-rc-maneuver="' + item.id + '"]');
      if (!row) return;
      const preset = maneuverConfigFor(item.id) || {};
      const targetEl = row.querySelector('[data-maneuver-target]');
      const percentEl = row.querySelector('[data-maneuver-percent]');
      // Пока оператор набирает число, телеметрия его не перебивает.
      if (targetEl && focused !== targetEl && preset[item.key] != null) {
        targetEl.value = String(preset[item.key]);
      }
      if (percentEl && focused !== percentEl && preset.percent != null) {
        percentEl.value = String(preset.percent);
      }
      const button = row.querySelector('[data-maneuver-start]');
      if (button) {
        button.disabled = !enabled || active;
        button.textContent = (active && state.name === item.id) ? 'Едем…' : 'Ехать';
      }
    });
    stop.disabled = !authorized || !active;
    const kind = state.kind === 'distance' ? 'distance' : 'angle';
    const format = (value) => kind === 'distance'
      ? (Number(value) || 0).toFixed(2) + ' м' : Math.round(Number(value) || 0) + '°';
    if (!supported) {
      statusEl.textContent = 'Пресеты манёвров недоступны: сервер не отдал их настройки.';
    } else if (!control.available) {
      statusEl.textContent = 'Манёвры — только в режиме master.';
    } else if (!authorized) {
      statusEl.textContent = 'Манёвры требуют разблокированного управления.';
    } else if (active) {
      statusEl.textContent = 'Манёвр «' + (state.title || state.name || '') + '»: ' +
        Math.round((Number(state.progress) || 0) * 100) + '% · ' +
        format(state.value) + ' из ' + format(state.target) +
        ' · ' + (state.mode === 'time' ? 'по времени (тахометра нет)' : 'по тахометру');
    } else if (state.state === 'done' || state.state === 'failed' || state.state === 'cancelled') {
      statusEl.textContent = state.message || 'Манёвр завершён.';
    } else if (!drive.canDrive) {
      statusEl.textContent = 'Манёвры недоступны: ' +
        (DRIVE_REASON_TEXT[drive.reason] || 'тяга не готова') + '.';
    } else {
      statusEl.textContent = 'Готово · выберите манёвр.';
    }
    statusEl.dataset.active = String(active);
  }

  /** Старт манёвра: сначала сохраняем числа из формы, потом пускаем. */
  async function startManeuver(id) {
    const item = MANEUVERS.find((entry) => entry.id === id);
    const row = document.querySelector('[data-rc-maneuver="' + id + '"]');
    if (!item || !row) return;
    if (!authorized) {
      $('rc-maneuver-status').textContent = 'Сначала разблокируйте PIN.';
      void unlock();
      return;
    }
    const targetEl = row.querySelector('[data-maneuver-target]');
    const percentEl = row.querySelector('[data-maneuver-percent]');
    const target = Number(targetEl && targetEl.value);
    const percent = Number(percentEl && percentEl.value);
    if (!(target > 0) || !(percent > 0)) {
      $('rc-maneuver-status').textContent = 'Проверьте числа: цель и процент больше нуля.';
      return;
    }
    const values = {percent: percent};
    values[item.key] = target;
    const saved = await json('api/kiosk_maneuver_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
      body: JSON.stringify({ name: id, values: values })
    });
    if (!saved.response.ok || saved.body.ok === false) {
      const reason = saved.body.reason || 'command_failed';
      if (reason === 'pin_required') authorized = false;
      $('rc-maneuver-status').textContent = MANEUVER_REASON_TEXT[reason] || 'Отказ сервера.';
      return;
    }
    const started = await json('api/kiosk_maneuver', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
      body: JSON.stringify({ action: 'start', name: id })
    });
    if (!started.response.ok || started.body.ok === false) {
      const reason = started.body.reason || 'command_failed';
      if (reason === 'pin_required') authorized = false;
      $('rc-maneuver-status').textContent = MANEUVER_REASON_TEXT[reason] || 'Отказ сервера.';
      return;
    }
    $('rc-maneuver-status').textContent = 'Манёвр «' + item.title + '» начат — сервер ведёт его сам.';
    renderManeuvers();
  }

  async function stopManeuver() {
    const result = await json('api/kiosk_maneuver', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
      body: JSON.stringify({ action: 'stop' })
    });
    if (!result.response.ok || result.body.ok === false) {
      const reason = result.body.reason || 'command_failed';
      $('rc-maneuver-status').textContent = MANEUVER_REASON_TEXT[reason] || 'Отказ сервера.';
      return;
    }
    $('rc-maneuver-status').textContent = 'Манёвр остановлен, тяга снята.';
    renderManeuvers();
  }

  function bindDriveControls() {
    const speed = $('rc-drive-speed');
    if (speed) speed.addEventListener('input', () => {
      $('rc-drive-speed-value').textContent = String(speed.value);
      if (driveDirection) void queueDrive(driveDirection * Number(speed.value));
    });
    document.querySelectorAll('[data-drive-direction]').forEach((button) => {
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        const direction = Number(button.dataset.driveDirection) < 0 ? -1 : 1;
        startDrive(direction);
        try { button.setPointerCapture(event.pointerId); } catch (error) {}
      });
      button.addEventListener('click', (event) => event.preventDefault());
      button.addEventListener('contextmenu', (event) => event.preventDefault());
    });
    const stop = $('rc-drive-stop');
    if (stop) stop.addEventListener('click', () => { void stopDrive(); });
    // Манёвры: «Ехать» в строке и общий СТОП манёвра.
    document.querySelectorAll('[data-rc-maneuver]').forEach((row) => {
      const button = row.querySelector('[data-maneuver-start]');
      if (button) button.addEventListener('click', () => {
        void startManeuver(row.dataset.rcManeuver);
      });
    });
    const maneuverStop = $('rc-maneuver-stop');
    if (maneuverStop) maneuverStop.addEventListener('click', () => { void stopManeuver(); });
  }

  function renderDrive() {
    const statusEl = $('rc-drive-status');
    const slider = $('rc-drive-speed');
    if (!statusEl || !slider) return;
    const drive = (snapshot && snapshot.drive) || {};
    // Предел — из телеметрии плат (dmax=), он задан прошивкой и не настраивается.
    const max = Math.max(1, Math.min(100, Number(drive.maxPercent) || 100));
    slider.max = String(max);
    if (Number(slider.value) > max) slider.value = String(max);
    $('rc-drive-speed-value').textContent = String(slider.value);
    const modeEl = $('rc-drive-mode');
    if (modeEl) {
      // Драйвер ZS-Z11H: ШИМ на входе «P», обороты с «SC». Настроек нет —
      // экран называет предел, который прислала плата.
      modeEl.textContent = drive.available
        ? 'Привод: ШИМ на входе «P» · тахометр SC (90 пер/об) · предел ' + max + '%'
        : 'Привод: нет связи с платами колёс.';
    }
    const supported = !!(snapshot && snapshot.steeringControl && snapshot.steeringControl.available);
    const authorizedForDrive = authorized && supported && !!drive.available;
    slider.disabled = !authorizedForDrive;
    document.querySelectorAll('[data-drive-direction]').forEach((button) => {
      button.disabled = !authorizedForDrive || !drive.canDrive;
    });
    $('rc-drive-stop').disabled = !authorizedForDrive;
    const reason = drive.reason && DRIVE_REASON_TEXT[drive.reason];
    if (!supported) statusEl.textContent = 'Нет master · установите RC_SLAM_SOURCE=master.';
    else if (!authorized) statusEl.textContent = 'Нужен PIN для движения.';
    else if (reason) statusEl.textContent = reason;
    else if (drive.active) {
      statusEl.textContent = 'Движение включено · ' + slider.value + '%' +
        ' · отпустите кнопку для торможения.';
    }
    else {
      // Угол колёс тягу не запирает (краб, боковой ход) — только подпись.
      let angleText = 'колёса выровнены';
      const spread = Number(drive.spreadDeg);
      if (drive.aligned === false && Number.isFinite(spread)) {
        angleText = 'колёса под углом (разброс ' + spread.toFixed(1).replace('.', ',') +
          '°) — поедет под этим углом';
      }
      statusEl.textContent = 'Готово · ' + slider.value + '% · все модули на связи, ' + angleText + '.';
    }
    statusEl.dataset.ready = String(!!drive.canDrive && authorizedForDrive);
    renderManeuvers();
  }

  function startDrive(direction) {
    if (!authorized) {
      $('rc-drive-status').textContent = 'Сначала разблокируйте PIN.';
      void unlock();
      return;
    }
    const drive = (snapshot && snapshot.drive) || {};
    if (!drive.canDrive) {
      renderDrive();
      return;
    }
    driveDirection = direction;
    if (driveTimer) clearInterval(driveTimer);
    const sendHeartbeat = () => {
      if (!driveDirection) return;
      void queueDrive(driveDirection * Number($('rc-drive-speed').value || 8));
    };
    sendHeartbeat();
    driveTimer = setInterval(sendHeartbeat, 200);
  }

  function releaseDrive() {
    const wasDriving = driveDirection !== 0 || driveTimer !== null;
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (wasDriving && authorized) void queueDrive(0);
  }

  async function stopDrive() {
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (!authorized) {
      $('rc-drive-status').textContent = 'PIN истёк — плата остановится по watchdog; разблокируйте для команды СТОП.';
      return;
    }
    await queueDrive(0);
    $('rc-drive-status').textContent = 'Стоп отправлен · PWM 0 · тормоз включён.';
  }

  function queueDrive(percent) {
    driveQueued = percent;
    if (driveSending) return driveFlush;
    driveSending = true;
    driveFlush = (async () => {
      while (driveQueued !== null) {
        const target = driveQueued;
        driveQueued = null;
        const controller = new AbortController();
        const timer = setTimeout(() => controller.abort(), 1200);
        try {
          const response = await fetch('api/drive', {
            method: 'POST', cache: 'no-store', signal: controller.signal,
            headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'drive' },
            body: JSON.stringify({ percent: target })
          });
          const body = await response.json().catch(() => ({}));
          if (!response.ok || !body.ok) {
            const reason = body.reason || (response.status === 401 ? 'pin_required' : 'command_failed');
            $('rc-drive-status').textContent = DRIVE_REASON_TEXT[reason] || 'Команда тяги отклонена.';
            if (response.status === 401 || reason === 'pin_required') authorized = false;
            if (target !== 0) {
              driveDirection = 0;
              if (driveTimer) clearInterval(driveTimer);
              driveTimer = null;
              driveQueued = 0;
            }
          } else if (target === 0) {
            $('rc-drive-status').textContent = 'PWM 0 · тормоз включён.';
          } else {
            $('rc-drive-status').textContent = 'Тяга ' + body.percent + '% · удерживайте кнопку.';
          }
        } catch (error) {
          $('rc-drive-status').textContent = 'Связь потеряна · аппаратный watchdog остановит моторы не позднее 650 мс.';
          if (target !== 0) {
            driveDirection = 0;
            if (driveTimer) clearInterval(driveTimer);
            driveTimer = null;
            driveQueued = 0;
          }
        } finally {
          clearTimeout(timer);
        }
      }
    })().finally(() => { driveSending = false; });
    return driveFlush;
  }

  /** Передвинуть цель колеса на экране; вернуть угол или null, если он не задан. */
  function markCardDirty(id, value) {
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    if (!card) return null;
    const range = card.querySelector('[data-range]');
    const angle = clampAngle(value);
    if (!range || angle === null) return null;
    range.value = String(angle);
    card.dataset.dirty = 'true';
    card.querySelector('[data-target]').textContent = angle + '°';
    renderCard(id);
    return angle;
  }

  function bindCard(id) {
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    const range = card.querySelector('[data-range]');
    if (window.RCSteerDial) window.RCSteerDial.attach(range);
    range.addEventListener('input', () => { markCardDirty(id, range.value); });
    // Как на бортовом экране: отпустил ползунок — команда ушла. Раньше здесь
    // требовалось ещё нажать «Повернуть», а на экране робота — нет: одно и то
    // же действие на двух пультах вело себя по-разному.
    range.addEventListener('change', () => { void sendAngle(id); });
    card.querySelectorAll('[data-step]').forEach((button) => button.addEventListener('click', () => {
      if (markCardDirty(id, Number(range.value) + Number(button.dataset.step)) !== null) void sendAngle(id);
    }));
    card.querySelector('[data-center]').addEventListener('click', () => {
      if (markCardDirty(id, 0) !== null) void sendAngle(id);
    });
    card.querySelector('[data-send]').addEventListener('click', () => void sendAngle(id));
    card.querySelector('[data-origin]').addEventListener('click', () => void setZero(id));
    card.querySelector('[data-calib]').addEventListener('click', () => void calibrate(id));
  }

  async function unlock() {
    const pin = window.prompt('Введите PIN для управления рулевыми и тяговыми моторами');
    if (pin === null) return;
    try {
      const { response, body } = await json('api/kiosk_lock/verify', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pin })
      });
      authorized = response.ok && !!body.steeringAuthorized;
      $('rc-steering-auth').textContent = authorized ? 'PIN подтверждён' : 'Неверный PIN';
      await refresh();
    } catch (error) {
      $('rc-steering-auth').textContent = 'Сервер не отвечает';
    }
  }

  async function lock() {
    driveDirection = 0;
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    if (authorized) await queueDrive(0);
    authorized = false;
    try { await json('api/kiosk_lock/session/end', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }); }
    catch (error) { /* closing the panel must still lock local controls */ }
    IDS.forEach((id) => { delete pending[id]; renderCard(id); });
    const auth = $('rc-steering-auth');
    if (auth) auth.textContent = 'Нужен PIN';
    const unlockButton = $('rc-steering-unlock');
    const lockButton = $('rc-steering-lock');
    if (unlockButton) unlockButton.hidden = false;
    if (lockButton) lockButton.hidden = true;
  }

  async function sendAngle(id) {
    if (!authorized) return unlock();
    // Двойное нажатие отправляло две команды: вторая возвращала «модуль занят»
    // и пугала оператора, хотя колесо уже ехало куда надо.
    if (sending[id]) return;
    const card = document.querySelector('[data-rc-steering="' + id + '"]');
    if (!card) return;
    const angle = clampAngle(card.querySelector('[data-range]').value);
    if (angle === null) return;
    const motor = snapshot && Array.isArray(snapshot.motors)
      ? snapshot.motors.find((item) => item && item.id === id) : null;
    const current = motor && Number.isFinite(+motor.angle) ? +motor.angle : null;
    if (motor && current !== null && !motor.moving && Math.abs(current - angle) <= 1) {
      // Колесо уже стоит там: команда не нужна. Если угол не совпал (не доехало)
      // — отправляем, это рабочий способ докрутить.
      card.dataset.dirty = '';
      feedback[id] = { text: 'колесо уже на ' + angle + '°', error: false, until: Date.now() + 4000 };
      await refresh();
      return;
    }
    const sendButton = card.querySelector('[data-send]');
    sending[id] = true;
    if (sendButton) sendButton.disabled = true;
    try {
      const { response, body } = await json('api/kiosk_steer', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id, angle })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason, body), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), angle: body.angle };
        card.dataset.dirty = '';
        feedback[id] = { text: 'команда отправлена · ' + angle + '°', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    } finally {
      sending[id] = false;
    }
    await refresh();
  }

  async function setZero(id) {
    if (!authorized) return unlock();
    if (!window.confirm('Запасной вариант без датчика. Физически выставьте колесо ' + id + ' прямо. Команда не двигает мотор; она назначает текущую позицию за программный 0°. После перезапуска Arduino ноль нужно задать заново. Продолжить?')) return;
    try {
      const { response, body } = await json('api/kiosk_zero', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), zero: true };
        feedback[id] = { text: 'программный ноль отправлен', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    }
    await refresh();
  }

  async function calibrate(id) {
    if (!authorized) return unlock();
    if (!window.confirm('Колесо ' + id + ' будет искать оптодатчик D18 в пределах ±135° от текущего положения (сначала вправо, затем влево) и встанет в ноль. После включения Arduino колесо должно стоять примерно прямо (±30°). Колесо не должно упираться, рядом не должно быть людей. Начать?')) return;
    try {
      const { response, body } = await json('api/kiosk_calibrate', {
        method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' },
        body: JSON.stringify({ module: id })
      });
      if (!response.ok || !body.ok) {
        if (response.status === 401) authorized = false;
        feedback[id] = { text: reasonText(body.reason), error: true, until: Date.now() + 6000 };
      } else {
        pending[id] = { at: Date.now(), calib: true, started: false };
        feedback[id] = { text: 'калибровка запущена', error: false, until: Date.now() + 6000 };
      }
    } catch (error) {
      feedback[id] = { text: 'сервер не отвечает', error: true, until: Date.now() + 6000 };
    }
    await refresh();
  }

  function reasonText(reason, detail) {
    const reported = Number(detail && detail.limit);
    const limit = Number.isFinite(reported) && reported > 0
      ? '±' + Math.round(reported) + '°'
      : (window.RCSteerPresets && typeof window.RCSteerPresets.limitText === 'function'
        ? window.RCSteerPresets.limitText() : '±' + limitDeg + '°');
    return ({
      pin_required: 'PIN истёк · разблокируйте', module_offline: 'модуль не отвечает',
      serial_unavailable: 'USB-порт мастера недоступен', not_homed: 'сначала калибровка или 0° вручную',
      module_busy: 'дождитесь остановки модуля', angle_limit: 'предел ' + limit,
      command_queue_full: 'очередь команд занята', control_not_supported: 'источник не поддерживает поворот',
      estop: 'E-STOP взведён · снимите защёлку', payload: 'неверный формат команды',
      command_failed: 'команда не выполнена'
    })[reason] || 'команда не принята';
  }

  function start() {
    addUi();
    // Нужный модуль включается только на страницах оператора/веб-панели.
    document.documentElement.classList.add('rc-steering-ready');
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
  window.RCSteeringPanel = { open: () => { const button = $('rc-steering-launch'); if (button) button.click(); } };
})();
