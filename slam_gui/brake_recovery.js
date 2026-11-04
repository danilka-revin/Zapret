/* Recovery only: this component can request a brake, never release it. */
(function () {
  'use strict';

  function create(options) {
    const panel = document.getElementById('brake-recovery');
    const button = document.getElementById('brake-restore');
    const status = document.getElementById('brake-recovery-status');
    if (!panel || !button || !status) return null;
    const messages = {
      brake_unconfirmed: 'Не все платы подтвердили тормоз. Проверьте связь и повторите.',
      neutral_firmware_required: 'Платы не передают подтверждение тормоза. Проверьте прошивки.',
      module_offline: 'Нет свежей связи со всеми четырьмя платами.',
      serial_unavailable: 'Нет связи с платами колёс.',
      module_busy: 'Дождитесь остановки колёс и рулевых модулей.',
      sc_invalid: 'Нет корректных показаний SC на всех колёсах.',
      command_failed: 'Не удалось запросить тормоз. Проверьте связь и повторите.'
    };
    let lastLocked = false, pending = false, waitingUntil = 0, note = '';
    const info = () => (options.getState() || {}).neutral || null;
    function render() {
      const state = info();
      const fresh = options.isFresh();
      // A lost connection must not erase the last known fault from the screen.
      if (fresh && state && typeof state.locked === 'boolean') lastLocked = state.locked;
      if (fresh && state && (state.phase === 'restoring' || !state.locked)) waitingUntil = 0;
      if (waitingUntil && Date.now() > waitingUntil) {
        waitingUntil = 0;
        note = 'Нет подтверждения тормоза. Проверьте связь и повторите.';
      }
      panel.hidden = !lastLocked;
      const keyboard = document.getElementById('keyboard-enable');
      if (keyboard) keyboard.disabled = lastLocked;
      if (!lastLocked) { note = ''; return; }
      button.disabled = !fresh || !state || pending || !!waitingUntil || state.phase === 'restoring';
      const text = !fresh || !state ? 'Нет связи: состояние тормоза неизвестно.'
        : pending || waitingUntil || state.phase === 'restoring' ? 'Запрошено торможение · ждём подтверждение всех четырёх плат…'
        : messages[state.reason] || note || 'Обнаружено неподтверждённое состояние тормоза. Верните тормоз.';
      if (status.textContent !== text) status.textContent = text;
    }
    async function restore() {
      if (button.disabled) return;
      pending = true;
      note = '';
      render();
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), 1000);
      try {
        const response = await fetch('api/kiosk_neutral', {
          method: 'POST', cache: 'no-store', signal: controller.signal,
          headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'},
          body: JSON.stringify({action: 'restore'})
        });
        const result = await response.json().catch(() => ({}));
        if (!response.ok || !result.ok) throw new Error(result.reason || 'command_failed');
        // A fresh state poll, not the HTTP acceptance, confirms the brake.
        waitingUntil = result.phase === 'restoring' ? Date.now() + 3000 : 0;
      } catch (error) {
        note = messages[error && error.message] || messages.command_failed;
      } finally {
        clearTimeout(timer);
        pending = false;
        render();
      }
    }
    button.addEventListener('click', () => { void restore(); });
    render();
    return {stateChanged: render};
  }
  window.RCBrakeRecovery = {create};
}());
