/* Bounded still-frame polling keeps camera feeds from monopolising the
 * browser's per-origin HTTP connections needed for wheel telemetry/commands. */
(function () {
  'use strict';
  if (window.RCCameraFrames) return;

  function create(onFrame) {
    const cameras = [0, 1, 2, 3].map(id => ({id, timer: null, timeout: null, image: null, queued: false}));
    const queue = [];
    let active = false, generation = 0, inFlight = 0;
    const MAX_IN_FLIGHT = 2;

    function enqueue(camera) {
      if (!active || camera.queued || camera.image) return;
      camera.queued = true;
      queue.push(camera);
      pump();
    }

    function pump() {
      while (active && inFlight < MAX_IN_FLIGHT && queue.length) {
        const camera = queue.shift();
        camera.queued = false;
        const image = new Image();
        camera.image = image;
        const epoch = generation;
        let done = false;
        inFlight += 1;
        function finish(ok, abort) {
          if (done || !active || epoch !== generation) return;
          done = true;
          clearTimeout(camera.timeout);
          camera.timeout = null;
          image.onload = image.onerror = null;
          if (abort) image.src = '';
          camera.image = null;
          inFlight -= 1;
          const live = !!ok && image.naturalWidth > 0;
          onFrame(camera.id, live ? image : null);
          camera.timer = setTimeout(() => { camera.timer = null; enqueue(camera); }, live ? 250 : 1000);
          pump();
        }
        image.onload = () => finish(true, false);
        image.onerror = () => finish(false, false);
        camera.timeout = setTimeout(() => finish(false, true), 1500);
        const url = 'api/camera/' + camera.id + '/frame?t=' + Date.now();
        image.src = window.RCSessionUrl ? window.RCSessionUrl(url) : url;
      }
    }

    function start() {
      if (active) return;
      active = true;
      generation += 1;
      cameras.forEach(enqueue);
    }

    function stop() {
      active = false;
      generation += 1;
      queue.length = 0;
      inFlight = 0;
      cameras.forEach(camera => {
        clearTimeout(camera.timer);
        clearTimeout(camera.timeout);
        camera.timer = camera.timeout = null;
        camera.queued = false;
        if (camera.image) {
          camera.image.onload = camera.image.onerror = null;
          camera.image.src = '';
          camera.image = null;
        }
        onFrame(camera.id, null);
      });
    }
    return {start, stop};
  }

  window.RCCameraFrames = {create};
})();
