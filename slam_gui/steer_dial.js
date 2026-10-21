/* One circular wheel-angle control for the onboard screen and web console.
 * The existing range remains the source of value/limits/permission and events.
 * This component never sends commands: input previews, change commits once.
 */
(function () {
  'use strict';
  const controls = new WeakMap();
  const NS = 'http://www.w3.org/2000/svg';
  const rad = Math.PI / 180;
  function point(angle, radius) {
    return [100 + Math.sin(angle * rad) * radius, 100 - Math.cos(angle * rad) * radius];
  }
  function arc(from, to) {
    const a = point(from, 80), b = point(to, 80);
    return 'M ' + a.join(' ') + ' A 80 80 0 ' + (Math.abs(to - from) > 180 ? 1 : 0) +
      ' ' + (to >= from ? 1 : 0) + ' ' + b.join(' ');
  }
  function svg(tag, attrs, parent) {
    const el = document.createElementNS(NS, tag);
    Object.entries(attrs).forEach(([key, value]) => el.setAttribute(key, String(value)));
    if (parent) parent.appendChild(el);
    return el;
  }
  function html(tag, className, text, parent) {
    const el = document.createElement(tag);
    el.className = className;
    if (text) el.textContent = text;
    if (parent) parent.appendChild(el);
    return el;
  }
  function signed(angle) { return (angle > 0 ? '+' : '') + Math.round(angle) + '°'; }

  function attach(input) {
    if (!input) return null;
    if (controls.has(input)) return controls.get(input);
    const wrapper = html('div', 'steer-dial-wrap');
    html('div', 'steer-dial-north', '↑ Вперёд · 0°', wrapper);
    const dial = html('div', 'steer-dial', '', wrapper);
    dial.setAttribute('role', 'slider');
    dial.setAttribute('aria-label', input.getAttribute('aria-label') || 'Целевой угол колеса');
    dial.setAttribute('aria-orientation', 'horizontal');
    dial.setAttribute('aria-description', 'Вид сверху. Отрицательный угол — влево, положительный — вправо. Стрелки — 1 градус, Page Up и Page Down — 5 градусов. Отпустите для поворота. Escape — отмена.');
    const image = svg('svg', {viewBox:'0 0 200 200', 'aria-hidden':'true'}, dial);
    svg('circle', {cx:100, cy:100, r:80, class:'steer-dial-ring'}, image);
    const allowed = svg('path', {class:'steer-dial-allowed'}, image);
    const selected = svg('path', {class:'steer-dial-selected'}, image);
    [-90, 0, 90].forEach(angle => {
      const a = point(angle, 71), b = point(angle, 88);
      svg('line', {x1:a[0], y1:a[1], x2:b[0], y2:b[1], class:'steer-dial-tick'}, image);
    });
    svg('path', {d:'M 100 43 V 157 M 43 100 H 157', class:'steer-dial-axis'}, image);
    const actual = svg('g', {class:'steer-dial-actual'}, image);
    svg('rect', {x:86, y:60, width:28, height:80, rx:8}, actual);
    svg('path', {d:'M 94 53 L 100 44 L 106 53'}, actual);
    const target = svg('g', {class:'steer-dial-target'}, image);
    svg('rect', {x:91, y:68, width:18, height:64, rx:5}, target);
    svg('path', {d:'M 100 43 L 91 57 H 109 Z'}, target);
    svg('path', {d:'M 94 78 H 106 M 94 87 H 106 M 94 113 H 106 M 94 122 H 106', class:'steer-dial-tread'}, target);
    svg('circle', {cx:100, cy:100, r:4, class:'steer-dial-hub'}, image);
    const knob = svg('circle', {r:7, class:'steer-dial-knob'}, image);
    const value = html('div', 'steer-dial-value', '', wrapper);
    const scale = html('div', 'steer-dial-scale', '', wrapper);
    const left = html('span', '', '', scale);
    html('span', '', 'Вид сверху', scale);
    const right = html('span', '', '', scale);
    html('small', 'steer-dial-legend', 'Сплошное колесо — выбрано · пунктир — сейчас', wrapper);
    html('small', 'steer-dial-help', 'Выберите угол по кругу и отпустите — колесо повернётся', wrapper);
    input.insertAdjacentElement('afterend', wrapper);
    input.hidden = true;
    let current = null;
    let pointer = null;
    let keys = null;

    function limits() {
      const min = Number(input.min), max = Number(input.max);
      return [Number.isFinite(min) ? min : -90, Number.isFinite(max) ? max : 90];
    }
    function clamp(n) { const [min, max] = limits(); return Math.max(min, Math.min(max, Math.round(n))); }
    function emit(type) { input.dispatchEvent(new Event(type, {bubbles:true})); }
    function preview(n) {
      if (input.disabled) return;
      input.value = String(clamp(n));
      sync(current);
      emit('input');
    }
    function releaseCapture(id) {
      try { if (dial.hasPointerCapture(id)) dial.releasePointerCapture(id); } catch (error) {}
    }
    function cancel() {
      const active = pointer || keys;
      const id = pointer && pointer.id;
      pointer = null; keys = null;
      if (id !== null && id !== undefined) releaseCapture(id);
      if (active) { input.value = active.start; sync(current); emit('input'); }
    }
    function sync(actualAngle) {
      current = actualAngle != null && Number.isFinite(Number(actualAngle)) ? Number(actualAngle) : null;
      if (input.disabled && (pointer || keys)) cancel();
      const [min, max] = limits();
      const angle = clamp(Number(input.value) || 0);
      allowed.setAttribute('d', arc(Math.max(-179.9, min), Math.min(179.9, max)));
      selected.setAttribute('d', angle === 0 ? '' : arc(0, angle));
      target.setAttribute('transform', 'rotate(' + angle + ' 100 100)');
      actual.setAttribute('visibility', current === null ? 'hidden' : 'visible');
      if (current !== null) actual.setAttribute('transform', 'rotate(' + current + ' 100 100)');
      const p = point(angle, 80);
      knob.setAttribute('cx', p[0]); knob.setAttribute('cy', p[1]);
      value.textContent = 'Выбрано ' + signed(angle) + ' · сейчас ' + (current === null ? '—' : signed(current));
      left.textContent = signed(min) + ' ← влево'; right.textContent = 'вправо → ' + signed(max);
      dial.setAttribute('aria-valuemin', min); dial.setAttribute('aria-valuemax', max);
      dial.setAttribute('aria-valuenow', angle);
      dial.setAttribute('aria-valuetext', signed(angle) + ', сейчас ' + (current === null ? 'нет данных' : signed(current)));
      dial.setAttribute('aria-disabled', String(input.disabled));
      dial.tabIndex = input.disabled ? -1 : 0;
    }
    function position(event) {
      const box = dial.getBoundingClientRect();
      const x = event.clientX - box.left - box.width / 2;
      const y = event.clientY - box.top - box.height / 2;
      if (!Number.isFinite(x + y) || Math.hypot(x, y) < box.width * 0.12) return null;
      return Math.atan2(x, -y) / rad;
    }
    dial.addEventListener('pointerdown', event => {
      event.stopPropagation();
      if (input.disabled || pointer || keys || (event.button != null && event.button !== 0) || event.isPrimary === false) return;
      const angle = position(event);
      if (angle === null) return;
      event.preventDefault();
      dial.focus();
      pointer = {id:event.pointerId, start:input.value, raw:angle, unwrapped:angle};
      try { dial.setPointerCapture(event.pointerId); } catch (error) { pointer = null; return; }
      preview(angle);
    });
    dial.addEventListener('pointermove', event => {
      event.stopPropagation();
      if (!pointer || pointer.id !== event.pointerId) return;
      event.preventDefault();
      const angle = position(event);
      if (angle === null) return;
      let delta = angle - pointer.raw;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      pointer.raw = angle; pointer.unwrapped += delta;
      // Unwrap the rear seam: crossing ±180° must never jump to the other stop.
      preview(pointer.unwrapped);
    });
    dial.addEventListener('pointerup', event => {
      event.stopPropagation();
      if (!pointer || pointer.id !== event.pointerId) return;
      event.preventDefault();
      pointer = null;
      releaseCapture(event.pointerId);
      if (!input.disabled) emit('change');
    });
    ['pointercancel', 'lostpointercapture'].forEach(name => dial.addEventListener(name, event => {
      if (pointer && pointer.id === event.pointerId) cancel();
    }));
    const steps = {ArrowRight:1, ArrowUp:1, ArrowLeft:-1, ArrowDown:-1, PageUp:5, PageDown:-5};
    dial.addEventListener('keydown', event => {
      if (event.key === 'Escape') { event.preventDefault(); event.stopPropagation(); cancel(); return; }
      if (!(event.key in steps) && event.key !== 'Home' && event.key !== 'End') return;
      event.preventDefault(); event.stopPropagation();
      if (input.disabled || pointer) return;
      if (!keys) keys = {start:input.value};
      const [min, max] = limits();
      preview(event.key === 'Home' ? min : event.key === 'End' ? max : Number(input.value) + steps[event.key]);
    });
    dial.addEventListener('keyup', event => {
      if (!(event.key in steps) && event.key !== 'Home' && event.key !== 'End') return;
      event.preventDefault(); event.stopPropagation();
      if (keys) { keys = null; if (!input.disabled) emit('change'); }
    });
    dial.addEventListener('blur', cancel);
    window.addEventListener('blur', cancel);
    window.addEventListener('pagehide', cancel);
    document.addEventListener('visibilitychange', () => { if (document.hidden) cancel(); });
    input.addEventListener('input', () => sync(current));
    input.addEventListener('change', () => sync(current));
    const control = {element:dial, sync};
    controls.set(input, control);
    sync(null);
    return control;
  }
  window.RCSteerDial = {attach, sync(input, current) {
    const control = controls.get(input);
    if (control) control.sync(current);
  }};
})();
