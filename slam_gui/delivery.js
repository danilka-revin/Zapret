/* Доставка по векторной карте: старт + курс, цель, маршрут и сухой прогон.
 * Реального привода здесь нет: все команды уходят в delivery_service, который
 * не имеет доступа к мосту колёс. Карта читается один раз, препятствия и треки
 * в постоянный файл не пишутся.
 */
(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const state = {
    map: null, summary: null, referenceUrl: '', image: null, imageReady: false,
    background: true, pickStart: true, start: null, goalNodeId: '', scenario: 'clear',
    plan: null, simulation: null, status: null, csrfToken: '',
    view: { scale: 1, ox: 0, oy: 0 }, bounds: null,
    headingDrag: false, playback: null, message: ''
  };

  function text(id, value) { const el = $(id); if (el && el.textContent !== String(value)) el.textContent = String(value); }
  function tone(id, value) { const el = $(id); if (el) el.dataset.tone = value || ''; }
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
  function normDeg(v) { let x = Number(v); if (!Number.isFinite(x)) return 0; x = ((x + 180) % 360 + 360) % 360 - 180; return x; }

  async function api(path, options) {
    const init = Object.assign({ headers: {} }, options || {});
    init.headers = Object.assign({ 'Accept': 'application/json' }, init.headers || {});
    if (init.body && typeof init.body !== 'string') {
      init.body = JSON.stringify(init.body);
      init.headers['Content-Type'] = 'application/json';
    }
    if (state.csrfToken) init.headers['X-CSRF-Token'] = state.csrfToken;
    const response = await fetch(path, init);
    let payload = null;
    try { payload = await response.json(); } catch (e) { payload = null; }
    if (!response.ok || !payload || payload.ok === false) {
      const message = payload && (payload.error || payload.code) ? (payload.error || payload.code) : 'Ошибка запроса ' + response.status;
      throw new Error(message);
    }
    return payload;
  }

  function control(path, body) {
    return api(path, { method: 'POST', headers: { 'X-Robot-Control': 'delivery' }, body: body || {} });
  }

  function mapToScreen(x, y) { return [state.view.ox + x * state.view.scale, state.view.oy - y * state.view.scale]; }
  function screenToMap(sx, sy) { return [(sx - state.view.ox) / state.view.scale, (state.view.oy - sy) / state.view.scale]; }

  function computeBounds() {
    const points = [];
    (state.map.nodes || []).forEach((n) => points.push(n.xy));
    (state.map.edges || []).forEach((e) => (e.geometry || []).forEach((p) => points.push(p)));
    const ref = state.map.reference || {};
    if (ref.imageWidth && ref.imageHeight) points.push([0, 0], [Number(ref.imageWidth), Number(ref.imageHeight)]);
    if (!points.length) return { minX: 0, maxX: 100, minY: 0, maxY: 100 };
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    points.forEach((p) => { minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]); minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]); });
    const pad = Math.max(10, (maxX - minX + maxY - minY) * 0.04);
    return { minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad };
  }

  function fit() {
    const canvas = $('delivery-canvas'); if (!canvas || !state.bounds) return;
    const w = canvas.clientWidth || 900, h = canvas.clientHeight || 600;
    const b = state.bounds;
    const scale = Math.min(w / Math.max(1e-6, b.maxX - b.minX), h / Math.max(1e-6, b.maxY - b.minY));
    state.view.scale = scale;
    state.view.ox = (w - (b.minX + b.maxX) * scale) / 2;
    state.view.oy = (h + (b.minY + b.maxY) * scale) / 2;
    draw();
  }

  function zoom(factor) {
    const canvas = $('delivery-canvas'); if (!canvas) return;
    const w = canvas.clientWidth || 900, h = canvas.clientHeight || 600;
    const cx = w / 2, cy = h / 2;
    const before = screenToMap(cx, cy);
    state.view.scale = clamp(state.view.scale * factor, 0.05, 50);
    const after = mapToScreen(before[0], before[1]);
    state.view.ox += cx - after[0]; state.view.oy += cy - after[1];
    draw();
  }

  function polylineDistance(geometry, distance) {
    let travelled = 0;
    for (let i = 0; i + 1 < geometry.length; i += 1) {
      const a = geometry[i], b = geometry[i + 1];
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (travelled + len >= distance) {
        const t = len <= 1e-9 ? 0 : (distance - travelled) / len;
        return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, Math.atan2(b[1] - a[1], b[0] - a[0])];
      }
      travelled += len;
    }
    const last = geometry[geometry.length - 1], prev = geometry[geometry.length - 2] || last;
    return [last[0], last[1], Math.atan2(last[1] - prev[1], last[0] - prev[0])];
  }

  function drawArrow(ctx, x, y, yaw, color, size) {
    const sx = x, sy = y, angle = -yaw;
    ctx.save(); ctx.translate(sx, sy); ctx.rotate(angle);
    ctx.beginPath(); ctx.moveTo(size, 0); ctx.lineTo(-size * 0.7, size * 0.6); ctx.lineTo(-size * 0.35, 0); ctx.lineTo(-size * 0.7, -size * 0.6); ctx.closePath();
    ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke(); ctx.restore();
  }

  function draw() {
    const canvas = $('delivery-canvas'); if (!canvas || !state.map) return;
    const ctx = canvas.getContext('2d'); if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 900, h = canvas.clientHeight || 600;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const light = document.documentElement.getAttribute('data-map-theme') === 'light';
    ctx.fillStyle = light ? '#f3f1f2' : '#08070a'; ctx.fillRect(0, 0, w, h);
    const ref = state.map.reference || {};
    if (state.background && state.imageReady && state.image && ref.imageWidth && ref.imageHeight) {
      const topLeft = mapToScreen(0, Number(ref.imageHeight));
      ctx.globalAlpha = light ? 0.55 : 0.72;
      ctx.drawImage(state.image, topLeft[0], topLeft[1], Number(ref.imageWidth) * state.view.scale, Number(ref.imageHeight) * state.view.scale);
      ctx.globalAlpha = 1;
    }
    (state.map.edges || []).forEach((edge) => {
      const geom = edge.geometry || []; if (geom.length < 2) return;
      ctx.beginPath();
      geom.forEach((p, i) => { const s = mapToScreen(p[0], p[1]); if (i === 0) ctx.moveTo(s[0], s[1]); else ctx.lineTo(s[0], s[1]); });
      ctx.strokeStyle = edge.closed ? '#e35d6a' : (light ? '#5d5560' : '#b9adb5');
      ctx.lineWidth = edge.closed ? 2 : 3; ctx.setLineDash(edge.closed ? [6, 5] : []); ctx.stroke(); ctx.setLineDash([]);
    });
    if (state.plan && state.plan.geometry && state.plan.geometry.length >= 2) {
      ctx.beginPath();
      state.plan.geometry.forEach((p, i) => { const s = mapToScreen(p[0], p[1]); if (i === 0) ctx.moveTo(s[0], s[1]); else ctx.lineTo(s[0], s[1]); });
      ctx.strokeStyle = '#c0505a'; ctx.lineWidth = 5; ctx.lineJoin = 'round'; ctx.stroke();
    }
    (state.map.nodes || []).forEach((node) => {
      const s = mapToScreen(node.xy[0], node.xy[1]);
      const delivery = node.kind === 'delivery' || node.kind === 'dock';
      ctx.beginPath(); ctx.arc(s[0], s[1], delivery ? 7 : 4, 0, Math.PI * 2);
      ctx.fillStyle = node.kind === 'dock' ? '#4fc3f7' : delivery ? '#7ee2a8' : (light ? '#6d6065' : '#a2949a');
      ctx.fill(); ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke();
      if (delivery) { ctx.fillStyle = light ? '#141013' : '#f5f2f4'; ctx.font = '11px system-ui, sans-serif'; ctx.fillText(node.label || node.id, s[0] + 9, s[1] - 7); }
      if (state.goalNodeId === node.id) { ctx.beginPath(); ctx.arc(s[0], s[1], 12, 0, Math.PI * 2); ctx.strokeStyle = '#7ee2a8'; ctx.lineWidth = 2; ctx.stroke(); }
    });
    // ПДД: светофоры, стоп-знаки, переходы вдоль маршрута или всей карты
    const controls = (state.plan && state.plan.trafficControls) || [];
    const mapLights = (state.map.trafficLights || []);
    const mapStops = (state.map.stopSigns || []);
    const mapCross = (state.map.crosswalks || []);
    // Если маршрут ещё не построен, показываем все объекты карты
    const toDraw = controls.length ? controls : [].concat(
      mapLights.map(l => ({type: 'traffic_light', id: l.id, xy: null, nodeId: l.nodeId})),
      mapStops.map(s => ({type: 'stop_sign', id: s.id, xy: null, nodeId: s.nodeId})),
      mapCross.map(c => ({type: 'crosswalk', id: c.id, xy: null, edgeId: c.edgeId}))
    );
    // Для объектов без xy (из карты) берём координату узла/середины ребра
    const nodeById = {};
    (state.map.nodes || []).forEach(n => nodeById[n.id] = n);
    toDraw.forEach(ctrl => {
      let xy = ctrl.xy;
      if (!xy) {
        if (ctrl.nodeId && nodeById[ctrl.nodeId]) xy = nodeById[ctrl.nodeId].xy;
        else if (ctrl.edgeId) {
          const edge = (state.map.edges || []).find(e => e.id === ctrl.edgeId);
          if (edge && edge.geometry && edge.geometry.length) {
            const mid = edge.geometry[Math.floor(edge.geometry.length/2)];
            xy = mid;
          }
        }
      }
      if (!xy) return;
      const s = mapToScreen(xy[0], xy[1]);
      if (ctrl.type === 'traffic_light') {
        // светофор: корпус + три огня, текущий — красный (по умолчанию показываем красный как требование остановиться)
        ctx.fillStyle = '#222'; ctx.fillRect(s[0]-8, s[1]-14, 16, 28);
        ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.strokeRect(s[0]-8, s[1]-14, 16, 28);
        const colors = ['#e74c3c', '#f1c40f', '#2ecc71'];
        colors.forEach((c, i) => {
          ctx.beginPath(); ctx.arc(s[0], s[1]-9+i*9, 4, 0, Math.PI*2);
          ctx.fillStyle = c; ctx.globalAlpha = i===0 ? 1 : 0.25; ctx.fill(); ctx.globalAlpha = 1; ctx.stroke();
        });
        ctx.fillStyle = light ? '#141013' : '#f5f2f4'; ctx.font = '9px system-ui, sans-serif'; ctx.fillText(ctrl.id, s[0]+10, s[1]+4);
      } else if (ctrl.type === 'stop_sign') {
        ctx.beginPath();
        for (let i=0;i<8;i++) { const a = (Math.PI*2*i/8) - Math.PI/8; const r=9; const x=s[0]+Math.cos(a)*r, y=s[1]+Math.sin(a)*r; if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
        ctx.closePath(); ctx.fillStyle = '#e74c3c'; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
        ctx.fillStyle = '#fff'; ctx.font = 'bold 7px system-ui, sans-serif'; ctx.textAlign='center'; ctx.fillText('STOP', s[0], s[1]+2); ctx.textAlign='left';
      } else if (ctrl.type === 'crosswalk') {
        // зебра: поперечные полосы
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 3;
        for (let k=-2;k<=2;k++) { ctx.beginPath(); ctx.moveTo(s[0]-10, s[1]+k*4); ctx.lineTo(s[0]+10, s[1]+k*4); ctx.stroke(); }
        ctx.fillStyle = light ? '#141013' : '#f5f2f4'; ctx.font = '9px system-ui, sans-serif'; ctx.fillText(ctrl.id, s[0]+12, s[1]+4);
      }
    });
    if (state.start) {
      const s = mapToScreen(state.start.x, state.start.y);
      drawArrow(ctx, s[0], s[1], state.start.yawDeg * Math.PI / 180, '#ffd166', 13);
      ctx.fillStyle = light ? '#141013' : '#f5f2f4'; ctx.font = '11px system-ui, sans-serif'; ctx.fillText('старт', s[0] + 14, s[1] + 14);
    }
    if (state.playback && state.plan && state.plan.geometry) {
      const pose = polylineDistance(state.plan.geometry, state.playback.distance);
      const s = mapToScreen(pose[0], pose[1]);
      drawArrow(ctx, s[0], s[1], pose[2], state.playback.stop ? '#ff8b83' : '#4fc3f7', 11);
    }
  }

  function renderChecks() {
    const status = state.status || {}; const checks = status.checks || {}; const labels = status.checkLabels || {};
    const list = $('delivery-check-list'); if (!list) return;
    list.replaceChildren();
    let ok = true;
    Object.keys(labels).forEach((key) => {
      const passed = Boolean(checks[key]); ok = ok && passed;
      const li = document.createElement('li'); li.className = 'delivery-check ' + (passed ? 'is-ok' : 'is-bad');
      const mark = document.createElement('span'); mark.className = 'delivery-check-mark'; mark.textContent = passed ? '✓' : '✕';
      const label = document.createElement('span'); label.textContent = labels[key];
      li.appendChild(mark); li.appendChild(label); list.appendChild(li);
    });
    text('delivery-checks-state', ok ? 'готово к сухому прогону' : 'не готово');
    const blockers = (status.driveBlockers || []).map((key) => (status.driveBlockerLabels || {})[key] || key);
    text('delivery-blockers', blockers.length ? 'Реальное движение запрещено: ' + blockers.join('; ') + '.' : 'Реальное движение запрещено.');
    const simulate = $('delivery-simulate'); if (simulate) simulate.disabled = !status.canSimulate;
  }

  function renderRoute() {
    const plan = state.plan;
    const list = $('delivery-maneuvers'); if (list) list.replaceChildren();
    if (!plan) { text('delivery-route-summary', 'Маршрут не построен.'); return; }
    const units = plan.units === 'm' ? 'м' : 'ед. карты';
    text('delivery-route-summary', `${plan.totalDistanceUnits} ${units} · участков: ${(plan.legs || []).length} · ПДД: ${(plan.trafficControls || []).length} объектов · тяга отключена`);
    (plan.maneuvers || []).forEach((m) => {
      const li = document.createElement('li');
      const instruction = { left: 'налево', right: 'направо', straight: 'прямо', u_turn: 'разворот', alignment: 'выравнивание у цели' }[m.instruction] || m.instruction;
      li.textContent = `${m.label || m.nodeId}: ${instruction}, ${m.turnDeg}° · ${m.atDistanceUnits} ${units}`;
      if (list) list.appendChild(li);
    });
    (plan.trafficControls || []).forEach((c) => {
      const names = {traffic_light: 'Светофор', stop_sign: 'СТОП', crosswalk: 'Переход'};
      const li = document.createElement('li'); li.dataset.tone = 'warn';
      li.textContent = `${names[c.type] || c.type} ${c.id}: ${c.atDistanceUnits} ${units}`;
      if (list) list.appendChild(li);
    });
    (plan.warnings || []).forEach((w) => { const li = document.createElement('li'); li.textContent = w; li.dataset.tone = 'warn'; if (list) list.appendChild(li); });
  }

  function renderEvents(upToDistance) {
    const list = $('delivery-events'); if (!list) return;
    list.replaceChildren();
    const sim = state.simulation; if (!sim) return;
    (sim.events || []).forEach((event) => {
      if (upToDistance !== undefined && event.distance > upToDistance + 1e-6) return;
      const li = document.createElement('li');
      li.dataset.tone = /STOP|YIELD|AVOID|SLOW|WAIT|SPEED/.test(event.state) ? (/STOP/.test(event.state) ? 'stop' : 'warn') : (/DONE|DOCK/.test(event.state) ? 'ok' : '');
      li.textContent = `${event.distance} · ${event.message}`;
      list.appendChild(li);
    });
  }

  function renderSimulation() {
    const sim = state.simulation;
    if (!sim) { text('delivery-sim-state', 'ожидание'); tone('delivery-sim-state', 'idle'); text('delivery-sim-distance', '—'); const p = $('delivery-progress'); if (p) p.value = 0; renderEvents(); return; }
    const ok = Boolean(sim.ok);
    text('delivery-sim-state', ok ? 'завершён' : sim.finalState);
    tone('delivery-sim-state', ok ? 'ok' : 'stop');
    text('delivery-sim-distance', `${sim.distanceUnits} / ${sim.totalDistanceUnits} ед. карты`);
    const p = $('delivery-progress'); if (p) p.value = sim.totalDistanceUnits ? 100 * sim.distanceUnits / sim.totalDistanceUnits : 0;
    renderEvents();
  }

  function applyStatus(status) {
    state.status = status; state.csrfToken = status.csrfToken || state.csrfToken;
    if (status.setup) {
      state.start = Object.assign({}, status.setup.start);
      state.goalNodeId = status.setup.goalNodeId || '';
      state.scenario = status.setup.scenario || 'clear';
      const ack = $('delivery-ack'); if (ack) ack.checked = Boolean(status.setup.draftAcknowledged);
    }
    if (status.lastPlan) state.plan = status.lastPlan;
    if (status.lastSimulation) state.simulation = status.lastSimulation;
    syncInputs(); renderChecks(); renderRoute(); renderSimulation(); draw();
  }

  function syncInputs() {
    if (state.start) {
      if ($('delivery-x')) $('delivery-x').value = String(Math.round(state.start.x * 1000) / 1000);
      if ($('delivery-y')) $('delivery-y').value = String(Math.round(state.start.y * 1000) / 1000);
      if ($('delivery-yaw')) $('delivery-yaw').value = String(Math.round(normDeg(state.start.yawDeg)));
      if ($('delivery-yaw-range')) $('delivery-yaw-range').value = String(Math.round(normDeg(state.start.yawDeg)));
      text('delivery-start-note', `Старт: X ${state.start.x.toFixed(2)}, Y ${state.start.y.toFixed(2)}, курс ${normDeg(state.start.yawDeg).toFixed(0)}°. Это ручная настройка до рейса.`);
    }
    if ($('delivery-goal')) $('delivery-goal').value = state.goalNodeId;
    if ($('delivery-scenario')) $('delivery-scenario').value = state.scenario;
    const node = state.map && (state.map.nodes || []).find((n) => n.id === state.goalNodeId);
    text('delivery-goal-note', node ? `Цель: ${node.label || node.id}.` : 'Цель не выбрана.');
  }

  function setupBody() {
    return { start: state.start, goalNodeId: state.goalNodeId, scenario: state.scenario,
             draftAcknowledged: Boolean($('delivery-ack') && $('delivery-ack').checked) };
  }

  async function refreshStatus() { const status = await api('/api/delivery/status'); applyStatus(status); }

  async function loadMap() {
    const payload = await api('/api/delivery/map');
    state.map = payload.map; state.summary = payload.summary; state.referenceUrl = payload.referenceImageUrl || '';
    state.bounds = computeBounds();
    const goal = $('delivery-goal');
    if (goal) {
      goal.replaceChildren();
      const empty = document.createElement('option'); empty.value = ''; empty.textContent = 'Выберите пункт доставки'; goal.appendChild(empty);
      (state.map.nodes || []).filter((n) => n.kind === 'delivery' || n.kind === 'dock').forEach((n) => {
        const option = document.createElement('option'); option.value = n.id; option.textContent = n.label || n.id; goal.appendChild(option);
      });
    }
    const scenario = $('delivery-scenario');
    if (scenario) {
      scenario.replaceChildren();
      (payload.scenarios || []).forEach((s) => { const option = document.createElement('option'); option.value = s.id; option.textContent = s.label; scenario.appendChild(option); });
    }
    text('delivery-map-badge', state.map.status === 'calibrated' ? 'КАЛИБРОВАНА' : 'ЧЕРНОВИК');
    tone('delivery-map-badge', state.map.status === 'calibrated' ? 'live' : 'idle');
    text('delivery-map-note', state.map.status === 'calibrated' ? 'Карта калибрована, но реальное движение на этом этапе всё равно отключено.' : 'Координаты приближённые. Масштаб не подтверждён: расстояния показываются в единицах карты, не в метрах.');
    if (state.referenceUrl && typeof Image !== 'undefined') {
      const image = new Image();
      image.onload = function () { state.image = image; state.imageReady = true; draw(); };
      image.onerror = function () { state.imageReady = false; draw(); };
      image.src = typeof window.RCSessionUrl === 'function' ? window.RCSessionUrl(state.referenceUrl) : state.referenceUrl;
    }
    fit();
  }

  function canvasPoint(event) {
    const canvas = $('delivery-canvas'); const rect = canvas.getBoundingClientRect();
    return [event.clientX - rect.left, event.clientY - rect.top];
  }

  function onPointerDown(event) {
    if (!state.map) return;
    const point = canvasPoint(event);
    if (state.start) {
      const s = mapToScreen(state.start.x, state.start.y);
      if (Math.hypot(point[0] - s[0], point[1] - s[1]) <= 24) {
        state.headingDrag = true; updateHeading(point); event.preventDefault(); return;
      }
    }
    if (state.pickStart) {
      const mapPoint = screenToMap(point[0], point[1]);
      state.start = { x: mapPoint[0], y: mapPoint[1], yawDeg: state.start ? normDeg(state.start.yawDeg) : 0 };
      state.plan = null; state.simulation = null; syncInputs(); renderRoute(); renderSimulation(); draw();
    }
  }

  function updateHeading(point) {
    if (!state.start) return;
    const s = mapToScreen(state.start.x, state.start.y);
    const yaw = Math.atan2(-(point[1] - s[1]), point[0] - s[0]) * 180 / Math.PI;
    state.start.yawDeg = normDeg(yaw); state.plan = null; state.simulation = null;
    syncInputs(); renderRoute(); renderSimulation(); draw();
  }

  function onPointerMove(event) { if (state.headingDrag) { updateHeading(canvasPoint(event)); event.preventDefault(); } }
  function onPointerUp() { state.headingDrag = false; }

  function startPlayback() {
    if (!state.simulation || !state.plan) return;
    const sim = state.simulation;
    const duration = clamp((sim.elapsedS || 1) * 120, 1500, 12000);
    const started = performance.now();
    state.playback = { distance: 0, stop: !sim.ok };
    function frame(now) {
      const t = clamp((now - started) / duration, 0, 1);
      state.playback.distance = sim.totalDistanceUnits * t;
      state.playback.stop = !sim.ok && t >= 0.98;
      const p = $('delivery-progress'); if (p) p.value = 100 * t * (sim.distanceUnits / Math.max(1e-6, sim.totalDistanceUnits));
      text('delivery-sim-distance', `${state.playback.distance.toFixed(1)} / ${sim.totalDistanceUnits} ед. карты`);
      renderEvents(state.playback.distance); draw();
      if (t < 1) state.playback.raf = requestAnimationFrame(frame);
      else { state.playback = null; renderSimulation(); draw(); }
    }
    state.playback.raf = requestAnimationFrame(frame);
  }

  async function runSimulation() {
    try {
      text('delivery-sim-note', 'Сухой прогон выполняется…');
      const payload = await control('/api/delivery/simulate', setupBody());
      state.plan = payload.plan; state.simulation = payload.simulation; applyStatus(payload.status);
      text('delivery-sim-note', payload.simulation.ok ? 'Сухой прогон завершён успешно. Тяга не включалась.' : 'Сухой прогон остановился по сценарию. Тяга не включалась.');
      startPlayback();
    } catch (error) { text('delivery-sim-note', error.message); tone('delivery-sim-state', 'stop'); }
  }

  function bind() {
    const canvas = $('delivery-canvas');
    if (canvas) {
      canvas.addEventListener('pointerdown', onPointerDown);
      canvas.addEventListener('pointermove', onPointerMove);
      canvas.addEventListener('pointerup', onPointerUp);
      canvas.addEventListener('pointerleave', onPointerUp);
    }
    window.addEventListener('resize', fit);
    [['delivery-zoom-in', () => zoom(1.2)], ['delivery-zoom-out', () => zoom(1 / 1.2)], ['delivery-fit', fit]].forEach(([id, fn]) => { const el = $(id); if (el) el.addEventListener('click', fn); });
    const pick = $('delivery-pick-start'); if (pick) pick.addEventListener('click', () => { state.pickStart = !state.pickStart; pick.setAttribute('aria-pressed', String(state.pickStart)); });
    const background = $('delivery-background'); if (background) background.addEventListener('change', () => { state.background = Boolean(background.checked); draw(); });
    const theme = $('delivery-theme'); if (theme) theme.addEventListener('click', () => {
      const next = document.documentElement.getAttribute('data-map-theme') === 'light' ? 'dark' : 'light';
      document.documentElement.setAttribute('data-map-theme', next); theme.setAttribute('aria-pressed', String(next === 'light'));
      theme.textContent = next === 'light' ? 'Тёмная карта' : 'Светлая карта';
      try { localStorage.setItem('kiosk_map_theme', next); } catch (e) {} draw();
    });
    ['delivery-x', 'delivery-y', 'delivery-yaw'].forEach((id) => { const el = $(id); if (el) el.addEventListener('change', () => {
      const x = Number($('delivery-x').value), y = Number($('delivery-y').value), yaw = Number($('delivery-yaw').value);
      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(yaw)) { state.start = { x, y, yawDeg: normDeg(yaw) }; state.plan = null; state.simulation = null; syncInputs(); renderRoute(); renderSimulation(); draw(); }
    }); });
    const yawRange = $('delivery-yaw-range'); if (yawRange) yawRange.addEventListener('input', () => {
      if (!state.start) state.start = { x: 0, y: 0, yawDeg: 0 };
      state.start.yawDeg = normDeg(yawRange.value); state.plan = null; state.simulation = null; syncInputs(); renderRoute(); renderSimulation(); draw();
    });
    const goal = $('delivery-goal'); if (goal) goal.addEventListener('change', () => { state.goalNodeId = goal.value; state.plan = null; state.simulation = null; syncInputs(); renderRoute(); renderSimulation(); draw(); });
    const scenario = $('delivery-scenario'); if (scenario) scenario.addEventListener('change', () => { state.scenario = scenario.value; });
    const ack = $('delivery-ack'); if (ack) ack.addEventListener('change', () => { if (state.status) { state.status.checks = Object.assign({}, state.status.checks, { draftAcknowledged: ack.checked }); state.status.canSimulate = Object.values(state.status.checks).every(Boolean); renderChecks(); } });
    const save = $('delivery-save'); if (save) save.addEventListener('click', async () => {
      try { const payload = await control('/api/delivery/setup', setupBody()); state.plan = payload.plan; applyStatus(payload.status); text('delivery-goal-note', 'Настройка сохранена. Маршрут построен.'); }
      catch (error) { text('delivery-goal-note', error.message); }
    });
    const plan = $('delivery-plan'); if (plan) plan.addEventListener('click', async () => {
      try { const payload = await api('/api/delivery/plan', { method: 'POST', body: setupBody() }); state.plan = payload.plan; applyStatus(payload.status); text('delivery-goal-note', 'Маршрут построен по графу дорог.'); }
      catch (error) { text('delivery-goal-note', error.message); }
    });
    const simulate = $('delivery-simulate'); if (simulate) simulate.addEventListener('click', runSimulation);
    const reset = $('delivery-reset'); if (reset) reset.addEventListener('click', async () => {
      try { const payload = await control('/api/delivery/reset', {}); state.plan = null; state.simulation = null; applyStatus(payload.status); text('delivery-sim-note', 'Результат сброшен. Настройка старта и цели сохранена.'); }
      catch (error) { text('delivery-sim-note', error.message); }
    });
  }

  async function init() {
    bind();
    try { await loadMap(); await refreshStatus(); }
    catch (error) {
      const empty = $('delivery-map-empty'); if (empty) { empty.hidden = false; empty.textContent = error.message; }
      text('delivery-map-note', error.message); text('delivery-sim-note', 'Карта или сервер недоступны. Реальное движение не включается.');
    }
  }

  window.RCDelivery = { init, state, fit, draw, api, control, setupBody, renderChecks, renderRoute, renderSimulation, startPlayback };
})();
