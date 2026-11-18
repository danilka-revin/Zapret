/* Кнопка обновляет одним fast-forward весь репозиторий: Flask-сервер,
   веб-пульт и статические файлы бортового экрана. */
(function () {
  'use strict';

  if (window.RCUpdater) return;
  const $ = (id) => document.getElementById(id);
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  let csrfToken = '';
  let currentCheck = null;
  let opened = false;
  let following = false;
  let starting = false;
  let uploading = false;
  let checkId = 0;

  async function getJson(url, init, timeout = 30000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(url, { cache: 'no-store', ...init, signal: controller.signal });
      const body = await response.json().catch(() => null);
      return { response, body };
    } finally {
      clearTimeout(timer);
    }
  }

  function mount() {
    if ($('rc-update-launch')) return;
    const button = document.createElement('button');
    button.id = 'rc-update-launch';
    button.className = 'rc-update-launch';
    button.type = 'button';
    button.textContent = 'Обновить с GitHub';
    button.title = 'Обновить программу через GitHub, с флешки или с другого ПК';

    const overlay = document.createElement('div');
    overlay.id = 'rc-update-overlay';
    overlay.className = 'rc-update-overlay';
    overlay.hidden = true;
    overlay.innerHTML = `
      <section class="rc-update-dialog" role="dialog" aria-modal="true" aria-labelledby="rc-update-title">
        <header class="rc-update-head"><div><h2 id="rc-update-title">Обновление программы</h2>
          <p>Обновляются веб-пульт и бортовой экран; локальные настройки сохраняются. Платы Arduino не прошиваются.</p></div>
          <button type="button" class="rc-update-close" aria-label="Закрыть">×</button></header>
        <label class="rc-update-source">Источник
          <select id="rc-update-source" class="rc-update-btn">
            <option value="github">GitHub (через интернет)</option>
            <option value="usb">С флешки (без интернета)</option>
            <option value="upload">Загрузить с ПК</option>
          </select>
        </label>
        <div class="rc-update-upload" id="rc-update-upload-panel" hidden>
          <label for="rc-update-file">Подписанный <code>robot-update.zip</code> из Releases или <code>robot-update.bulka</code> (до 512 МиБ).</label>
          <div class="rc-update-file-actions">
            <input type="file" id="rc-update-file" accept=".zip,.bulka" />
            <button type="button" class="rc-update-btn" id="rc-update-send" disabled>Загрузить и проверить</button>
          </div>
        </div>
        <div class="rc-update-facts">
          <div><span>Репозиторий</span><b id="rc-update-repo">—</b></div>
          <div><span>Ветка</span><b id="rc-update-branch">—</b></div>
          <div><span>Установлено</span><b id="rc-update-from">—</b></div>
          <div><span>Доступно</span><b id="rc-update-to">—</b></div>
        </div>
        <div class="rc-update-progress-wrap" aria-live="polite">
          <div class="rc-update-progress-label"><b id="rc-update-phase">Нажмите «Проверить».</b><span id="rc-update-percent">0%</span></div>
          <div class="rc-update-track"><i id="rc-update-progress"></i></div>
          <p id="rc-update-message" role="status"></p>
        </div>
        <pre id="rc-update-log" class="rc-update-log" hidden></pre>
        <footer class="rc-update-actions">
          <button type="button" class="rc-update-btn" id="rc-update-check">Проверить</button>
          <button type="button" class="rc-update-btn primary" id="rc-update-run" disabled>Установить обновление</button>
          <button type="button" class="rc-update-btn" id="rc-update-dismiss">Закрыть</button>
        </footer>
      </section>`;
    const header = document.querySelector('.top-status, .head-actions, .top-right');
    if (header) header.append(button);
    else { button.classList.add('rc-update-floating'); document.body.append(button); }
    document.body.append(overlay);

    button.addEventListener('click', () => {
      opened = true;
      overlay.hidden = false;
      void check(false);
    });
    overlay.querySelector('.rc-update-close').addEventListener('click', close);
    $('rc-update-dismiss').addEventListener('click', close);
    overlay.addEventListener('click', (event) => { if (event.target === overlay) close(); });
    $('rc-update-source').addEventListener('change', () => {
      $('rc-update-file').value = '';
      void check(true);
    });
    $('rc-update-file').addEventListener('change', () => {
      if (currentCheck) showCheck(currentCheck);
      else syncUpdateControls();
    });
    $('rc-update-send').addEventListener('click', () => void uploadUpdate());
    $('rc-update-check').addEventListener('click', () => void check(true));
    $('rc-update-run').addEventListener('click', () => void startUpdate());
    window.addEventListener('keydown', (event) => { if (event.key === 'Escape' && opened) close(); });

    // Неблокирующая проверка: подсветить кнопку при появлении новой версии.
    setTimeout(() => { if (!checkId) void check(false); }, 5000);
  }

  function close() {
    opened = false;
    const overlay = $('rc-update-overlay');
    if (overlay) overlay.hidden = true;
  }

  function setButton(label, state) {
    const button = $('rc-update-launch');
    if (!button) return;
    button.textContent = label;
    button.dataset.state = state || '';
  }

  function syncUpdateControls() {
    const input = $('rc-update-file');
    const locked = uploading || starting || following;
    input.disabled = locked;
    $('rc-update-send').disabled = locked || !input.files.length || !csrfToken;
    const awaitingUpload = $('rc-update-source').value === 'upload' && input.files.length > 0;
    $('rc-update-run').disabled = locked || !currentCheck?.canUpdate || !csrfToken || awaitingUpload;
  }

  function showCheck(data) {
    currentCheck = data;
    csrfToken = data.csrfToken || csrfToken;
    $('rc-update-repo').textContent = data.repo || '—';
    $('rc-update-branch').textContent = data.branch || '—';
    $('rc-update-from').textContent = data.local || '—';
    $('rc-update-to').textContent = data.latest || '—';
    $('rc-update-percent').textContent = '0%';
    $('rc-update-progress').style.width = '0%';
    syncUpdateControls();
    if (data.busy) {
      $('rc-update-phase').textContent = 'Обновление уже выполняется…';
      $('rc-update-message').textContent = 'Подключаюсь к его прогрессу.';
      setButton('Обновление…', 'busy');
      void followUpdate();
      return;
    }
    if (data.mode === 'upload' && (data.pending === false || $('rc-update-file').files.length)) {
      $('rc-update-phase').textContent = 'Загрузите файл обновления';
      $('rc-update-message').textContent = 'Выберите пакет и нажмите «Загрузить и проверить». Робот не установит его без подтверждения.';
      setButton('Загрузить с ПК', '');
    } else if (data.dirty) {
      $('rc-update-phase').textContent = 'Есть локальные изменения';
      $('rc-update-message').textContent = 'Автообновление отключено, чтобы не перезаписать файлы. Сохраните изменения в Git и проверьте снова.';
      setButton('Проверить', '');
    } else if (data.updateAvailable && data.needsEstop) {
      $('rc-update-phase').textContent = 'Нужен E-STOP';
      $('rc-update-message').textContent = 'Остановите робота, включите E-STOP и нажмите «Проверить» ещё раз.';
      setButton('Есть обновление', 'available');
    } else if (data.updateAvailable) {
      $('rc-update-phase').textContent = 'Доступна новая версия';
      $('rc-update-message').textContent = 'После установки сервер и отдельный киоск-процесс перезапустятся; страница обновится автоматически.';
      setButton('Есть обновление', 'available');
    } else {
      $('rc-update-phase').textContent = 'Установлена последняя версия';
      $('rc-update-message').textContent = data.latestMsg || 'Обновлений для этой ветки нет.';
      setButton('Актуально', 'current');
    }
  }

  async function check(forceOpen) {
    if (forceOpen) {
      opened = true;
      $('rc-update-overlay').hidden = false;
    }
    if (following || starting || uploading) return;
    const id = ++checkId;
    const mode = $('rc-update-source').value;
    const source = { github: 'Обновить с GitHub', usb: 'С флешки', upload: 'Загрузить с ПК' }[mode];
    $('rc-update-upload-panel').hidden = mode !== 'upload';
    currentCheck = null;
    syncUpdateControls();
    for (const field of ['repo', 'branch', 'from', 'to']) $('rc-update-' + field).textContent = '—';
    $('rc-update-log').hidden = true;
    $('rc-update-percent').textContent = '0%';
    $('rc-update-progress').style.width = '0%';
    $('rc-update-phase').textContent = mode === 'github' ? 'Проверяю Git-ветку…' : 'Проверяю пакет обновления…';
    $('rc-update-message').textContent = '';
    $('rc-update-check').disabled = true;
    try {
      const { response, body } = await getJson('api/update/check?source=' + encodeURIComponent(mode), undefined, 40000);
      if (id !== checkId) return;
      if (body && body.csrfToken) csrfToken = body.csrfToken;
      if (!response.ok || !body || !body.ok) {
        $('rc-update-phase').textContent = 'Проверка не выполнена';
        $('rc-update-message').textContent = (body && body.error) || 'Не удалось проверить обновления.';
        setButton(source, 'error');
        return;
      }
      showCheck(body);
    } catch (error) {
      if (id !== checkId) return;
      $('rc-update-phase').textContent = 'Нет ответа сервера';
      $('rc-update-message').textContent = 'Не удалось проверить обновление. Попробуйте ещё раз.';
      setButton(source, 'error');
    } finally {
      if (id === checkId && !following) $('rc-update-check').disabled = false;
      syncUpdateControls();
    }
  }

  function sendPackage(file) {
    return new Promise((resolve, reject) => {
      const xhr = new XMLHttpRequest();
      xhr.open('POST', 'api/update/upload');
      xhr.responseType = 'json';
      xhr.timeout = 30 * 60 * 1000;
      xhr.setRequestHeader('Content-Type', 'application/octet-stream');
      xhr.setRequestHeader('X-CSRF-Token', csrfToken);
      xhr.upload.onprogress = (event) => {
        if (!event.lengthComputable) return;
        const pct = Math.min(100, Math.round(100 * event.loaded / event.total));
        $('rc-update-percent').textContent = pct + '%';
        $('rc-update-progress').style.width = pct + '%';
      };
      xhr.upload.onload = () => {
        $('rc-update-phase').textContent = 'Проверяю подпись и контрольную сумму…';
      };
      xhr.onload = () => {
        if (xhr.status === 201 && xhr.response && xhr.response.ok) resolve(xhr.response);
        else reject(new Error(xhr.response?.error || 'Не удалось загрузить файл. Проверьте вход и соединение с роботом.'));
      };
      xhr.onerror = xhr.ontimeout = xhr.onabort = () => reject(new Error('Передача прервана. Загрузите файл ещё раз.'));
      xhr.send(file);
    });
  }

  async function uploadUpdate() {
    const input = $('rc-update-file');
    const file = input.files[0];
    if (!file || !csrfToken || following || starting || uploading) return;
    ++checkId;  // Ignore any check started before the upload.
    currentCheck = null;
    uploading = true;
    $('rc-update-source').disabled = true;
    $('rc-update-check').disabled = true;
    syncUpdateControls();
    for (const field of ['repo', 'branch', 'from', 'to']) $('rc-update-' + field).textContent = '—';
    $('rc-update-log').hidden = true;
    $('rc-update-percent').textContent = '0%';
    $('rc-update-progress').style.width = '0%';
    $('rc-update-phase').textContent = 'Загружаю файл на робота…';
    $('rc-update-message').textContent = 'Не закрывайте страницу до окончания проверки.';
    try {
      const offer = await sendPackage(file);
      input.value = '';
      showCheck(offer);
    } catch (error) {
      $('rc-update-phase').textContent = 'Файл не принят';
      $('rc-update-message').textContent = error.message;
    } finally {
      uploading = false;
      $('rc-update-source').disabled = following;
      $('rc-update-check').disabled = following;
      syncUpdateControls();
    }
  }

  async function startUpdate() {
    if (!currentCheck || $('rc-update-run').disabled) return;
    const source = $('rc-update-source').value;
    const fromFile = source !== 'github';
    const prompt = fromFile
      ? `Установить версию ${currentCheck.latest} из проверенного файла? Не снимайте E-STOP до конца обновления.`
      : 'Установить новую версию с GitHub? При перезапуске пульта и экрана связь пропадёт на несколько секунд.';
    if (!window.confirm(prompt)) return;
    const payload = fromFile ? { source, expectedSha: currentCheck.latestSha } : {};
    currentCheck = null;
    starting = true;
    syncUpdateControls();
    $('rc-update-source').disabled = true;
    $('rc-update-check').disabled = true;
    $('rc-update-message').textContent = 'Запускаю обновление…';
    try {
      const { response, body } = await getJson('api/update/run', {
        method: 'POST', headers: { 'X-CSRF-Token': csrfToken, 'Content-Type': 'application/json' }, body: JSON.stringify(payload)
      }, 15000);
      if (!response.ok || !body || !body.ok) {
        $('rc-update-phase').textContent = 'Не удалось запустить';
        $('rc-update-message').textContent = (body && (body.error || body.phase)) || 'Повторите проверку.';
        $('rc-update-check').disabled = false;
        return;
      }
      await followUpdate();
    } catch (error) {
      $('rc-update-phase').textContent = 'Нет ответа сервера';
      $('rc-update-message').textContent = 'Проверяю статус обновления…';
      await followUpdate();
    } finally {
      starting = false;
      syncUpdateControls();
      $('rc-update-source').disabled = false;
      $('rc-update-check').disabled = false;
    }
  }

  async function followUpdate() {
    if (following) return;
    following = true;
    syncUpdateControls();
    $('rc-update-source').disabled = true;
    let rev = -1;
    let failures = 0;
    try {
      while (opened) {
        try {
          const { response, body } = await getJson('api/update/status?since=' + encodeURIComponent(rev), undefined, 30000);
          if (!response.ok || !body) throw new Error('status');
          failures = 0;
          rev = Number(body.rev) || 0;
          renderStatus(body);
          if (body.status === 'done') {
            if (Number(body.changes) > 0 && body.restart) {
              await waitForRestart(body.toSha || '');
            }
            return;
          }
          if (body.status === 'error') return;
          if (body.status === 'idle') {
            await sleep(500);
            continue;
          }
        } catch (error) {
          failures++;
          if (failures > 80) {
            $('rc-update-phase').textContent = 'Сервер долго не отвечает';
            $('rc-update-message').textContent = 'Проверьте запуск сервера и обновите страницу вручную.';
            return;
          }
          await sleep(750);
        }
      }
    } finally {
      following = false;
      syncUpdateControls();
      $('rc-update-source').disabled = starting;
      $('rc-update-check').disabled = starting;
    }
  }

  function renderStatus(state) {
    $('rc-update-phase').textContent = state.phase || 'Обновляю…';
    const pct = Math.max(0, Math.min(100, Number(state.pct) || 0));
    $('rc-update-percent').textContent = pct + '%';
    $('rc-update-progress').style.width = pct + '%';
    if (state.error) $('rc-update-message').textContent = state.error;
    const log = $('rc-update-log');
    const lines = Array.isArray(state.output) ? state.output : [];
    if (lines.length) {
      log.hidden = false;
      log.textContent = lines.slice(-14).join('\n');
      log.scrollTop = log.scrollHeight;
    }
    if (state.status === 'busy') setButton('Обновление…', 'busy');
    if (state.status === 'error') setButton('Повторить', 'error');
    if (state.status === 'done') {
      $('rc-update-run').disabled = true;
      if (Number(state.changes) === 0) setButton('Актуально', 'current');
    }
  }

  async function waitForRestart(expectedSha) {
    $('rc-update-phase').textContent = 'Перезапускаю сервер и экран робота…';
    $('rc-update-message').textContent = 'Жду ответ уже запущенного процесса с новой версией кода.';
    const until = Date.now() + 90000;
    while (opened && Date.now() < until) {
      try {
        const { body } = await getJson('api/update/version', undefined, 3500);
        const buildSha = String(body && body.buildSha || '');
        if (body && body.ok && buildSha && (!expectedSha || buildSha === expectedSha)) {
          window.location.reload();
          return;
        }
      } catch (error) { /* процесс перезапускается */ }
      await sleep(900);
    }
    $('rc-update-phase').textContent = 'Обновление установлено';
    $('rc-update-message').textContent = 'Сервер обновлён, но процесс с новой версией кода не ответил автоматически. Проверьте службы и перезагрузите страницу.';
  }

  function start() {
    mount();
    window.RCUpdater = { check: () => check(true) };
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
