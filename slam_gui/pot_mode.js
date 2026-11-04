/* Режим подтверждается только свежей телеметрией всех четырёх плат. */
(function () {
  'use strict';
  let pending = false;
  let desired = null;
  let deadline = 0;
  let host;
  const errors = {
    firmware_update_required: 'Обновите прошивки мастера и всех трёх слейвов.',
    module_offline: 'Нужна связь со всеми четырьмя платами.',
    module_busy: 'Сначала остановите тягу, повороты и калибровку.',
    zero_required: 'Включите режим с потенциометром и задайте ноль всех колёс.',
    pin_required: 'Разблокируйте управление PIN-кодом.',
    estop: 'Включён E-STOP.',
    control_not_supported: 'Режим недоступен для этого источника.'
  };
  window.RCPotMode = {
    render(parent, state, authorized) {
      if (!parent) return;
      if (!host) {
        host = document.createElement('section');
        host.className = 'calibration-card pot-mode';
        host.setAttribute('aria-labelledby', 'pot-mode-title');
        host.innerHTML = '<div class="calibration-card__head">' +
          '<span class="calibration-card__icon" aria-hidden="true">◔</span>' +
          '<div><span class="calibration-card__eyebrow">ДАТЧИК ПОЛОЖЕНИЯ</span><h5 id="pot-mode-title">Потенциометр</h5></div></div>' +
          '<p class="calibration-card__description">Выберите, как измерять угол поворота колёс.</p>' +
          '<label class="pot-mode__field"><span>Режим датчика</span><select aria-label="Режим потенциометра" disabled>' +
          '<option value="unknown">Нет подтверждения</option><option value="on">С потенциометром</option><option value="off">Без потенциометра</option></select></label>' +
          '<button class="calibration-secondary" type="button" disabled>Калибровать потенциометры (±60°)</button>' +
          '<small class="calibration-card__footnote">Без датчика угол считается по шагам, без доводки; ноль руля обязателен. После перезапуска платы — режим с датчиком, активная доводка выключена.</small>' +
          '<span class="calibration-status" role="status"></span><span class="calibration-alert" role="alert"></span>';
        parent.appendChild(host);
        host.querySelector('select').addEventListener('change', e => { const action = e.target.value; e.target.value = 'unknown'; void send(action); });
        host.querySelector('button').addEventListener('click', () => {
          if (window.confirm('Все колёса повернутся на −60° и +60°. Освободите пространство вокруг колёс. Начать калибровку потенциометров?')) void send('calibrate');
        });
      }
      const motors = (state && state.motors) || [];
      const ready = ['FL', 'FR', 'RL', 'RR'].every(id => motors.some(m => m.id === id && m.online && typeof m.potMode === 'boolean'));
      const mode = ready && motors.every(m => m.potMode === true) ? 'on' : ready && motors.every(m => m.potMode === false) ? 'off' : 'unknown';
      if (desired && mode === desired) { desired = null; host.querySelector('[role="alert"]').textContent = ''; }
      if (desired && Date.now() > deadline) { desired = null; host.querySelector('[role="alert"]').textContent = 'Платы не подтвердили режим. Проверьте связь и повторите.'; }
      const busy = motors.some(m => m.moving || m.calibrating || m.cycling || m.drivePercent);
      const disabled = !authorized || !ready || busy || pending || !!desired ||
        !!(state && state.neutral && state.neutral.locked) || !!(state && state.estop && (state.estop.latched || state.estop.unknown));
      host.querySelector('select').disabled = disabled;
      host.querySelector('select').value = mode;
      host.querySelector('button').disabled = disabled || mode !== 'on' || !motors.every(m => m.homed);
      const status = host.querySelector('[role="status"]');
      const statusText = desired ? 'Ожидание подтверждения плат…'
        : pending ? 'Отправка команды…'
        : !authorized ? 'Разблокируйте управление PIN-кодом.'
        : state && state.estop && (state.estop.latched || state.estop.unknown) ? 'E-STOP: настройка недоступна.'
        : state && state.neutral && state.neutral.locked ? 'Сначала верните и подтвердите тормоз.'
        : !ready ? 'Нужны связь со всеми платами и прошивка с поддержкой режимов.'
        : busy ? 'Идёт движение или калибровка — сначала остановите.'
        : mode === 'unknown' ? 'Режимы плат различаются — выберите общий режим.'
        : mode === 'off' ? 'Подтверждено: без потенциометра.'
        : !motors.every(m => m.homed) ? 'Сначала задайте ноль всех колёс для калибровки датчиков.'
        : motors.every(m => m.pot) ? 'С потенциометром: датчики откалиброваны.'
        : 'С потенциометром: нужна калибровка датчиков.';
      // Live regions should announce changes, not the same text on every poll.
      if (status.textContent !== statusText) status.textContent = statusText;
    }
  };
  async function send(action) {
    pending = true;
    host.querySelector('select').disabled = true;
    host.querySelector('button').disabled = true;
    const message = host.querySelector('[role="alert"]');
    try {
      const response = await fetch('api/kiosk_pot_mode', { method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Robot-Control': 'steering' }, body: JSON.stringify({ action }) });
      const result = await response.json();
      if (!response.ok || !result.ok) throw new Error(errors[result.reason] || 'Команда отклонена: ' + result.reason);
      message.textContent = 'Команда отправлена; проверьте телеметрию и журнал плат.';
      if (action !== 'calibrate') { desired = action; deadline = Date.now() + 5000; }
    } catch (error) { message.textContent = error.message || 'Ошибка связи'; }
    finally { pending = false; }
  }
})();
