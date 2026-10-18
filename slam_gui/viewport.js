/* Pan/zoom in CSS pixels. Pointer capture keeps gestures inside the viewport. */
(function () {
  'use strict';
  window.RCViewport = function (element, changed, options) {
    const min = (options && options.min) || 1;
    const max = (options && options.max) || 8;
    const state = { zoom: 1, x: 0, y: 0 };
    const pointers = new Map();
    function emit() { changed(state); }
    function zoomTo(value, x, y) {
      const rect = element.getBoundingClientRect();
      const px = (x == null ? rect.width / 2 : x) - rect.width / 2;
      const py = (y == null ? rect.height / 2 : y) - rect.height / 2;
      const next = Math.max(min, Math.min(max, value));
      const ratio = next / state.zoom;
      state.x = px - (px - state.x) * ratio;
      state.y = py - (py - state.y) * ratio;
      state.zoom = next;
      if (min === 1 && next === 1) state.x = state.y = 0;
      emit();
    }
    function reset() { state.zoom = 1; state.x = state.y = 0; emit(); }
    element.addEventListener('wheel', (e) => {
      e.preventDefault(); e.stopPropagation();
      const r = element.getBoundingClientRect();
      zoomTo(state.zoom * Math.exp(-e.deltaY * 0.002), e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    element.addEventListener('pointerdown', (e) => {
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      if (e.target.closest && e.target.closest('button')) return;
      e.stopPropagation();
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (element.setPointerCapture) element.setPointerCapture(e.pointerId);
    });
    element.addEventListener('pointermove', (e) => {
      const old = pointers.get(e.pointerId);
      if (!old) return;
      e.preventDefault(); e.stopPropagation();
      const before = Array.from(pointers.values());
      pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
      const after = Array.from(pointers.values());
      if (after.length === 2) {
        const distance = (a) => Math.hypot(a[0].x - a[1].x, a[0].y - a[1].y);
        const d = distance(before);
        const r = element.getBoundingClientRect();
        if (d > 1) zoomTo(state.zoom * distance(after) / d,
          (before[0].x + before[1].x) / 2 - r.left, (before[0].y + before[1].y) / 2 - r.top);
        state.x += (e.clientX - old.x) / 2;
        state.y += (e.clientY - old.y) / 2;
      } else if (after.length === 1) {
        state.x += e.clientX - old.x; state.y += e.clientY - old.y;
      }
      emit();
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      element.addEventListener(name, (e) => { pointers.delete(e.pointerId); });
    }
    element.addEventListener('keydown', (e) => {
      if (e.key === '+' || e.key === '=') { e.preventDefault(); zoomTo(state.zoom * 1.25); }
      if (e.key === '-') { e.preventDefault(); zoomTo(state.zoom / 1.25); }
      if (e.key === '0') { e.preventDefault(); reset(); }
    });
    return { state, zoomTo, reset };
  };
})();
