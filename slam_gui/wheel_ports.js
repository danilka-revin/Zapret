/* USB wheel identity is supplied by the onboard owner; never infer roles from tty order. */
(function () {
  'use strict';
  if (window.RCWheelPorts) return;
  const IDS = ['FL', 'FR', 'RL', 'RR'];

  function render(element, motor, fresh) {
    if (!element) return;
    const port = fresh && motor && motor.online === true && typeof motor.port === 'string'
      ? motor.port.trim() : '';
    const path = port && typeof motor.portPath === 'string' ? motor.portPath.trim() : '';
    element.textContent = port || '—';
    element.title = port
      ? 'Подтверждённый USB-порт: ' + port + (path && path !== port ? '\nПуть открытия: ' + path : '')
      : 'Нет свежего подтверждения USB-порта этого колеса';
  }

  function status(snapshot) {
    const info = snapshot && snapshot.masterPort;
    if (!info || info.auto !== true) return '';
    const modules = info.modules && typeof info.modules === 'object' ? info.modules : {};
    const missing = IDS.filter(id => typeof modules[id] !== 'string' || !modules[id]);
    const legacy = Array.isArray(info.legacy) ? info.legacy.filter(Boolean) : [];
    const unknown = Array.isArray(info.unclassified) ? info.unclassified.filter(Boolean) : [];
    const excluded = Array.isArray(info.excluded) ? info.excluded.filter(Boolean) : [];
    const candidates = Array.isArray(info.candidates) ? info.candidates.filter(Boolean) : [];
    const parts = ['Автопоиск USB: ' + (IDS.length - missing.length) + '/4'];
    if (info.error) parts.push('Последняя ошибка: ' + String(info.error));
    if (legacy.length) parts.push('Старая прошивка (< fw22): ' + legacy.join(', '));
    if (unknown.length) parts.push('USB без подтверждённого типа: ' + unknown.join(', '));
    if (excluded.length && (missing.length || info.ready !== true)) {
      parts.push('Не проверялись (исключение или активный лидар): ' + excluded.join(', '));
    }
    if (missing.length) parts.push('Не опознаны: ' + missing.join(', '));
    if (!candidates.length && !excluded.length) parts.push('USB-порты не найдены: проверьте питание и кабели');
    if (info.ready !== true) parts.push('Команды заблокированы до завершения USB-проверки');
    return parts.join(' · ');
  }

  window.RCWheelPorts = { render, status };
})();
