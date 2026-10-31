/* Same-origin authenticated transport for the shared onboard motor UI.
 * The browser never connects to localhost:8090 or directly to a USB device. */
(function () {
  'use strict';
  window.RCRemoteControls = true;
  const nativeFetch = window.fetch.bind(window);
  let csrfToken = '';
  let driveMayBeActive = false;
  const session = nativeFetch('/api/control/session' + window.location.search, {cache: 'no-store'})
    .then(async response => {
      if (!response.ok) throw new Error('Нужен вход оператора');
      const data = await response.json();
      if (!data.ok || !data.csrfToken) throw new Error('Не удалось открыть управление');
      csrfToken = data.csrfToken;
      return csrfToken;
    });
  // Keep a rejected bootstrap promise handled even before the first UI request.
  session.catch(() => {});
  const routes = {
    '/api/kiosk_state': '/api/control/state',
    '/api/kiosk_wheel_state': '/api/control/wheel_state',
    '/api/kiosk_logs': '/api/control/logs',
    '/api/battery': '/api/control/battery',
    '/api/kiosk_lock': '/api/control/lock',
    '/api/kiosk_lock/open': '/api/control/lock/open',
    '/api/kiosk_lock/close': '/api/control/lock/close',
    '/api/kiosk_steer': '/api/control/steer',
    '/api/kiosk_zero': '/api/control/zero',
    '/api/kiosk_calibrate': '/api/control/calibrate',
    '/api/kiosk_full_zero': '/api/control/full_zero',
    '/api/kiosk_pot_mode': '/api/control/pot_mode',
    '/api/kiosk_maneuver': '/api/control/maneuver',
    '/api/kiosk_maneuver_config': '/api/control/maneuver_config',
    '/api/kiosk_keyboard': '/api/control/keyboard',
    '/api/kiosk_neutral': '/api/control/neutral',
    '/api/kiosk_drive': '/api/control/drive',
    '/api/estop': '/api/control/estop',
    '/api/estop/release': '/api/control/estop/release',
    '/api/wheels/settings': '/api/control/wheels/settings'
  };
  window.fetch = async function (input, init) {
    const base = input instanceof Request ? input : null;
    const url = new URL(base ? base.url : input, window.location.href);
    if (url.origin !== window.location.origin) return nativeFetch(input, init);
    if (routes[url.pathname]) url.pathname = routes[url.pathname];
    const options = {...init, headers: new Headers((init && init.headers) || (base && base.headers) || {})};
    const method = String((init && init.method) || (base && base.method) || 'GET').toUpperCase();
    if (url.pathname === '/api/control/drive' && method === 'POST') {
      // Stay conservative after a drive attempt, even if a subsequent stop fails.
      driveMayBeActive = true;
    }
    if (method !== 'GET' && method !== 'HEAD') {
      options.headers.set('X-CSRF-Token', await session);
    }
    // Preserve a string body for keepalive: turning it into a Request stream
    // first makes the browser reject unload-time stop requests.
    return nativeFetch(base ? new Request(url, base) : url.toString(), options);
  };
  window.addEventListener('pagehide', () => {
    if (!csrfToken || !driveMayBeActive) return;
    // Best effort on navigation. Firmware's 650 ms watchdog remains the backstop.
    void window.fetch('/api/control/drive', {
      method: 'POST', keepalive: true,
      headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'},
      body: JSON.stringify({percent: 0})
    }).catch(() => {});
  });
  document.addEventListener('DOMContentLoaded', () => {
    const bind = (id, release) => {
      const button = document.getElementById(id);
      if (!button) return;
      button.addEventListener('click', async () => {
        const pin = release ? window.prompt('PIN для снятия E-STOP:') : '';
        if (release && pin === null) return;
        const message = document.getElementById('remote-control-message');
        try {
          const response = await window.fetch('/api/control/estop' + (release ? '/release' : ''), {
            method: 'POST', headers: {'Content-Type': 'application/json', 'X-Robot-Control': 'drive'},
            body: JSON.stringify(release ? {pin} : {})
          });
          const data = await response.json();
          message.textContent = response.ok && data.ok
            ? (release ? 'E-STOP снят. Тяга сама не включается.' : 'E-STOP включён.')
            : 'Команда не выполнена: ' + (data.reason || data.error || response.status);
        } catch (error) { message.textContent = 'Нет подтверждения от борта. Проверьте связь.'; }
      });
    };
    bind('remote-estop', false);
    bind('remote-estop-release', true);
  });
  document.addEventListener('click', event => {
    const link = event.target.closest && event.target.closest('a[href]');
    if (!link) return;
    const token = new URLSearchParams(window.location.search).get('st');
    const url = new URL(link.href, window.location.href);
    if (token && url.origin === window.location.origin) {
      url.searchParams.set('st', token);
      link.href = url.toString();
    }
  });
}());
