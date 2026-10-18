/* Shared read-only chassis model. Coordinates: +X forward, +Y left (ROS).
 * Actual angle and targetAngle are distinct telemetry channels, never substituted.
 * No commands are sent from this module. */
(function () {
  'use strict';
  if (window.RCRobotModel) return;
  const IDS = ['FL', 'FR', 'RL', 'RR'];
  const STALE_MS = 2500;
  const COLORS_KEY = 'rc_robot_wheel_colors_v1';
  const DEFAULT_COLORS = { target: '#36c275', actual: '#e35d6a' };
  const listeners = new Set();
  function sanitizeColors(raw) {
    const result = {};
    for (const key of ['target', 'actual']) {
      result[key] = raw && /^#[0-9a-f]{6}$/i.test(raw[key]) ? raw[key].toLowerCase() : DEFAULT_COLORS[key];
    }
    return result;
  }
  function loadColors() {
    try { return sanitizeColors(JSON.parse(localStorage.getItem(COLORS_KEY) || 'null')); }
    catch (err) { return { ...DEFAULT_COLORS }; }
  }
  let colors = loadColors();
  const getColors = () => ({ ...colors });
  function setColors(patch) {
    colors = sanitizeColors({ ...colors, ...patch });
    try { localStorage.setItem(COLORS_KEY, JSON.stringify(colors)); } catch (err) {}
    listeners.forEach(listener => listener());
    return getColors();
  }
  function subscribe(listener) { listeners.add(listener); return () => listeners.delete(listener); }
  function bindColors(targetId, actualId, resetId) {
    const target = document.getElementById(targetId), actual = document.getElementById(actualId);
    if (!target || !actual) return;
    const populate = () => { target.value = colors.target; actual.value = colors.actual; };
    populate(); subscribe(populate);
    target.addEventListener('input', () => setColors({ target: target.value }));
    actual.addEventListener('input', () => setColors({ actual: actual.value }));
    const reset = document.getElementById(resetId);
    if (reset) reset.addEventListener('click', () => setColors(DEFAULT_COLORS));
  }
  if (window.addEventListener) window.addEventListener('storage', event => {
    if (event.key !== COLORS_KEY && event.key !== null) return;
    colors = loadColors(); listeners.forEach(listener => listener());
  });
  let data = null;
  let receivedAt = 0;
  const finite = value => value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value));

  function update(snapshot, now = Date.now()) {
    data = snapshot && typeof snapshot === 'object' ? snapshot : null;
    receivedAt = now;
  }

  function state(now = Date.now()) {
    const fresh = !!data && now - receivedAt <= STALE_MS && data.linkOk === true;
    const motors = data && Array.isArray(data.motors) ? data.motors : [];
    const wheels = IDS.map(id => {
      const motor = motors.find(m => m && m.id === id) || {};
      const valid = fresh && motor.online === true && finite(motor.angle) && Math.abs(Number(motor.angle)) <= 180;
      const angle = valid ? Number(motor.angle) : null;
      const targetAngle = fresh && motor.online === true && finite(motor.targetAngle) && Math.abs(Number(motor.targetAngle)) <= 180
        ? Number(motor.targetAngle) : null;
      return { id, angle, targetAngle, targetRadians: targetAngle === null ? null : targetAngle * Math.PI / 180, radians: angle === null ? null : angle * Math.PI / 180,
        fault: valid && !!(motor.fault || motor.calibError),
        homed: valid && motor.homed === true, calibrating: valid && motor.calibrating === true,
        moving: valid && motor.moving === true, rpm: valid && finite(motor.rpm) ? Number(motor.rpm) : null };
    });
    const estop = !!(data && data.estop && data.estop.latched === true);
    return { fresh, wheels, estop, demo: !!(data && /demo|sim/i.test(String(data.source || ''))),
      speed: fresh && finite(data.speedMps) ? Number(data.speedMps) : null,
      driving: fresh && !estop && !!(data.drive && data.drive.active),
      source: data ? String(data.source || '') : '' };
  }

  function palette(light) {
    return light ? { body: '#f2e6e8', edge: '#75444c', wheel: '#3a3035', text: '#30262b', panel: 'rgba(255,255,255,.92)' }
      : { body: '#302128', edge: '#e1b5bd', wheel: '#151216', text: '#f5f2f4', panel: 'rgba(17,16,19,.92)' };
  }

  // Local +X is screen-right. Rotation by -yaw makes ROS positive angles left.
  function draw(ctx, x, y, yaw, scale, options = {}) {
    const live = options.state || state();
    const pal = palette(options.light);
    const k = Math.max(scale, options.minScale || 42);
    ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw);
    ctx.fillStyle = pal.body; ctx.strokeStyle = live.estop ? '#c0505a' : pal.edge; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(.375 * k, -.22 * k); ctx.lineTo(.295 * k, -.30 * k);
    ctx.lineTo(-.375 * k, -.30 * k); ctx.lineTo(-.375 * k, .30 * k);
    ctx.lineTo(.295 * k, .30 * k); ctx.lineTo(.375 * k, .22 * k); ctx.closePath(); ctx.fill(); ctx.stroke();
    // Per-wheel telemetry: FL/FR at +X; left is +Y, hence screen -Y.
    live.wheels.forEach(wheel => {
      const front = wheel.id[0] === 'F' ? 1 : -1;
      const side = wheel.id[1] === 'L' ? 1 : -1;
      ctx.save(); ctx.translate(front * .275 * k, -side * .25 * k);
      // Target is an unfilled, dashed outline; actual is a smaller solid wheel.
      // Distinct shapes remain readable when colors coincide or angles overlap.
      if (Number.isFinite(wheel.targetRadians)) {
        ctx.save(); ctx.rotate(-wheel.targetRadians);
        ctx.strokeStyle = colors.target; ctx.lineWidth = 2; ctx.setLineDash([4, 2]);
        ctx.beginPath(); ctx.rect(-.115 * k, -.065 * k, .23 * k, .13 * k); ctx.stroke();
        ctx.setLineDash([]); ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(.15 * k, 0); ctx.stroke();
        ctx.restore();
      }
      ctx.rotate(-(wheel.radians === null ? 0 : wheel.radians));
      ctx.fillStyle = wheel.angle === null ? pal.wheel : colors.actual;
      ctx.strokeStyle = wheel.angle === null ? '#82757c' : colors.actual;
      ctx.setLineDash(wheel.angle === null ? [3, 2] : []);
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.rect(-.09 * k, -.045 * k, .18 * k, .09 * k); ctx.fill(); ctx.stroke();
      ctx.setLineDash([]);
      if (wheel.angle !== null) {
        ctx.beginPath(); ctx.moveTo(-.045 * k, 0); ctx.lineTo(.07 * k, 0); ctx.stroke();
      }
      ctx.restore();
    });
    // Forward arrow and front-mounted lidar share the main restrained accent.
    ctx.fillStyle = pal.text;
    ctx.beginPath(); ctx.moveTo(.20 * k, 0); ctx.lineTo(-.08 * k, -.10 * k);
    ctx.lineTo(-.04 * k, 0); ctx.lineTo(-.08 * k, .10 * k); ctx.closePath(); ctx.fill();
    ctx.fillStyle = live.estop ? '#c0505a' : pal.edge;
    ctx.beginPath(); ctx.arc(.35 * k, 0, Math.max(3, .045 * k), 0, Math.PI * 2); ctx.fill();
    ctx.restore();
    return live;
  }

  function summary(live = state()) {
    const status = live.estop ? 'E-STOP' : !live.fresh ? 'нет свежей телеметрии' : live.driving ? 'тяга активна' : 'тяга выключена';
    const angles = live.wheels.map(w => w.id + ' ' + (w.angle === null ? '—' : w.angle.toFixed(1) + '°' + (w.fault ? ' ошибка' : w.calibrating ? ' калибр.' : !w.homed ? ' без нуля' : '')));
    return { status: (live.demo ? 'ДЕМО · ' : '') + status,
      rows: [angles.slice(0, 2).join('   '), angles.slice(2).join('   ')],
      targetRows: [0, 2].map(start => live.wheels.slice(start, start + 2).map(w => w.id + ' ' + (w.targetAngle == null ? '—' : w.targetAngle.toFixed(1) + '°')).join('   ')),
      speed: live.speed === null ? 'скорость —' : 'скорость ' + live.speed.toFixed(2) + ' м/с' };
  }

  function drawStatus(ctx, width, height, options = {}) {
    const info = summary();
    const pal = palette(options.light);
    const x = 8, y = Math.max(8, height - (options.bottom || 8) - 110);
    const w = Math.min(360, width - 16);
    ctx.save(); ctx.fillStyle = pal.panel; ctx.strokeStyle = options.light ? '#e2dcde' : '#3a3035';
    ctx.beginPath(); ctx.rect(x, y, w, 110); ctx.fill(); ctx.stroke();
    ctx.fillStyle = pal.text; ctx.font = '11px system-ui, sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(info.status + ' · ' + info.speed, x + 8, y + 7, w - 16);
    [colors.actual, colors.target].forEach((color, index) => {
      ctx.fillStyle = color;
      const rows = index === 0 ? info.rows : info.targetRows;
      ctx.fillText((index === 0 ? 'Факт: ' : 'Цель: ') + rows[0], x + 8, y + 24 + index * 32, w - 16);
      ctx.fillText(rows[1], x + 8, y + 40 + index * 32, w - 16);
    });
    ctx.fillStyle = pal.text;
    ctx.fillText('Контур — цель · заливка — факт · — нет данных', x + 8, y + 91, w - 16);
    ctx.restore();
  }

  window.RCRobotModel = { update, state, draw, drawStatus, summary, STALE_MS,
    getColors, setColors, bindColors, subscribe, DEFAULT_COLORS: { ...DEFAULT_COLORS } };
})();
