/* Robot cameras only: never claim the operator's webcam or the server's USB device.
 * Independent JPEG requests avoid Image.complete/MJPEG inconsistencies in kiosk Chromium.
 */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const cameras = [];
  let active = false;
  let initialized = false;

  function init() {
    if (initialized) return;
    initialized = true;
    for (const n of [1, 2]) {
      const canvas = $('cam' + n + '-canvas');
      const view = $('cam' + n + '-view');
      if (!canvas || !view) continue;
      const ctx = canvas.getContext('2d');
      if (!ctx) continue;
      const camera = { n, canvas, view, ctx, frame: null, pending: null, timer: null, timeout: null, generation: 0 };
      camera.viewport = window.RCViewport(view, () => draw(camera));
      const bind = (suffix, fn) => { const button = $('cam' + n + '-' + suffix); if (button) button.addEventListener('click', fn); };
      bind('in', () => camera.viewport.zoomTo(camera.viewport.state.zoom * 1.25));
      bind('out', () => camera.viewport.zoomTo(camera.viewport.state.zoom / 1.25));
      bind('reset', () => camera.viewport.reset());
      bind('expand', () => {
        const panel = $('swipe-cameras-view');
        const card = view.closest('.cam-card');
        const expanded = !card.classList.contains('is-expanded');
        panel.classList.toggle('camera-expanded', expanded);
        for (const c of cameras) {
          const chosen = c === camera && expanded;
          c.view.closest('.cam-card').classList.toggle('is-expanded', chosen);
          const button = $('cam' + c.n + '-expand');
          button.setAttribute('aria-pressed', String(chosen));
          button.textContent = chosen ? 'Свернуть' : 'Развернуть';
        }
        cameras.forEach(draw);
      });
      cameras.push(camera);
      draw(camera);
    }
    window.addEventListener('resize', () => cameras.forEach(draw));
    document.addEventListener('visibilitychange', () => {
      for (const camera of cameras) {
        stop(camera);
        if (active && !document.hidden) poll(camera);
      }
    });
  }

  function draw(camera) {
    const { canvas, view, ctx, frame } = camera;
    const r = view.getBoundingClientRect();
    if (!r.width || !r.height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0a0a0b'; ctx.fillRect(0, 0, r.width, r.height);
    const v = camera.viewport.state;
    if (frame) {
      const fit = Math.min(r.width / frame.naturalWidth, r.height / frame.naturalHeight);
      const w = frame.naturalWidth * fit * v.zoom, h = frame.naturalHeight * fit * v.zoom;
      // Do not allow dragging the entire image out of view.
      v.x = Math.max(-Math.max(0, (w - r.width) / 2), Math.min(Math.max(0, (w - r.width) / 2), v.x));
      v.y = Math.max(-Math.max(0, (h - r.height) / 2), Math.min(Math.max(0, (h - r.height) / 2), v.y));
      ctx.drawImage(frame, (r.width - w) / 2 + v.x, (r.height - h) / 2 + v.y, w, h);
    }
    const label = $('cam' + camera.n + '-zoom');
    if (label) label.textContent = v.zoom.toFixed(1) + '×';
  }

  function status(camera, live) {
    const n = camera.n;
    const empty = $('cam' + n + '-empty');
    if (empty) { empty.hidden = live; empty.textContent = 'Нет сигнала · переподключение…'; }
    const chip = $('cam' + n + '-chip');
    if (chip) { chip.textContent = live ? '● LIVE' : 'нет сигнала'; chip.classList.toggle('ok', live); }
    const dot = $('cam' + n + '-dot');
    if (dot) dot.classList.toggle('ok', live);
    const meta = $('cam' + n + '-meta');
    if (meta) meta.textContent = live
      ? camera.frame.naturalWidth + '×' + camera.frame.naturalHeight + ' · цифровой зум до 8×'
      : 'ожидание камеры ' + (n === 1 ? 'передней' : 'задней');
  }

  function poll(camera) {
    if (!active || document.hidden) return;
    const generation = camera.generation;
    const image = new Image();
    camera.pending = image;
    let done = false;
    function finish(ok) {
      if (done || camera.generation !== generation) return;
      done = true;
      clearTimeout(camera.timeout);
      image.onload = image.onerror = null;
      camera.pending = null;
      const live = ok && image.naturalWidth > 0;
      camera.frame = live ? image : null;
      status(camera, live); draw(camera);
      camera.timer = setTimeout(() => poll(camera), live ? 80 : 1000);
    }
    image.onload = () => finish(true);
    image.onerror = () => finish(false);
    camera.timeout = setTimeout(() => { finish(false); image.src = ''; }, 5000);
    // IDs are fixed: rear camera still works when it is the only connected one.
    image.src = 'api/camera/' + (camera.n - 1) + '/frame?t=' + Date.now();
  }

  function stop(camera) {
    camera.generation++;
    clearTimeout(camera.timer); clearTimeout(camera.timeout);
    if (camera.pending) {
      camera.pending.onload = camera.pending.onerror = null;
      camera.pending.src = '';
      camera.pending = null;
    }
  }

  window.RCCameraView = {
    setActive(value) {
      init();
      if (active === !!value) return;
      active = !!value;
      for (const camera of cameras) {
        stop(camera);
        if (active) { camera.frame = null; status(camera, false); draw(camera); poll(camera); }
      }
    },
  };
})();
