/* Экран «Навигация по карте»: три шага без сюрпризов.
 *
 * Маршрут и навигацию целиком делает map_view.js — он же рисует карту здесь и
 * сам обновляет кнопки `nav-*` по ответу сервера. Этот файл добавляет только
 * то, чего на экране карты нет:
 *   • список проверок готовности с понятным «что мешает» (тот же ответ сервера,
 *     что решает, можно ли ехать — без второй, «своей» логики);
 *   • подпись «здесь стоит робот» и строка состояния карты;
 *   • подсказки к шагам.
 *
 * Данные приходят не запросом, а подпиской: map_view.js зовёт
 * RCNavigationScreen.onStatus(status) после каждого опроса /api/navigation/status.
 * Поэтому на экране ровно один источник правды и один поток запросов.
 */
(function () {
  'use strict';
  const $ = (id) => (typeof document === 'undefined' ? null : document.getElementById(id));

  function text(id, value) {
    const el = $(id);
    if (el && el.textContent !== value) el.textContent = value;
  }

  function message(value, tone) {
    const el = $('nav-msg');
    if (!el) return;
    el.textContent = value;
    el.dataset.tone = tone || '';
  }

  function setTone(el, tone) {
    if (el) el.dataset.tone = tone || '';
  }

  // Ключи — diagnostics.checks из navigation.status(): подпись и что делать.
  const CHECK_LABELS = {
    enabled: ['Автономная тяга разрешена', 'включите RC_NAV_ENABLE=1 после проверки системы'],
    speedCalibrated: ['Скорость измерена', 'нужен RC_NAV_MPS_PER_PERCENT: значение не угадывается'],
    speedWithinLimit: ['Скорость в пределах тяги', 'уставка скорости больше предела тяги'],
    masterReady: ['Шасси готово', 'нужен RC_SLAM_SOURCE=master и свободная тяга'],
    driveStopped: ['Тяга стоит', 'кто-то держит привод — сначала остановите его'],
    mapFresh: ['Скан свежий', 'лидар не присылает кадры'],
    mapReady: ['Карта собрана', 'нужно минимум два скана лидара'],
    mapTrusted: ['Карта с физического лидара', 'карта пришла не с датчика робота'],
    mapIntegrity: ['Карта без забывания препятствий', 'в настройках карты включён decay'],
    scanReady: ['Скан полный', 'в скане есть пропуски — часть пространства не видна'],
    poseReady: ['Локализация уверенная', 'сопоставление сканов ниже порога'],
    mapDecayDisabled: ['Забывание препятствий выключено', 'в настройках карты включён decay'],
    footprintSufficient: ['Габарит с запасом', 'RC_NAV_FOOTPRINT_M меньше половины диагонали корпуса'],
    estopClear: ['E-STOP снят', 'на борту взведена аварийная остановка'],
  };

  function renderChecks(status) {
    const list = $('navscreen-check-list');
    const checks = (status && status.diagnostics && status.diagnostics.checks) || {};
    const keys = Object.keys(CHECK_LABELS);
    if (list) {
      list.replaceChildren();
      keys.forEach((key) => {
        const [label, why] = CHECK_LABELS[key];
        const ok = checks[key] === true;
        const row = document.createElement('li');
        row.className = 'navscreen-check ' + (ok ? 'is-ok' : 'is-bad');
        const mark = document.createElement('span');
        mark.className = 'navscreen-check-mark';
        mark.textContent = ok ? '✓' : '✕';
        mark.setAttribute('aria-hidden', 'true');
        const body = document.createElement('span');
        body.className = 'navscreen-check-text';
        const title = document.createElement('b');
        title.textContent = label;
        body.appendChild(title);
        if (!ok && why) {
          const hint = document.createElement('small');
          hint.textContent = ' — ' + why;
          body.appendChild(hint);
        }
        row.appendChild(mark);
        row.appendChild(body);
        list.appendChild(row);
      });
    }
    const bad = keys.filter((key) => checks[key] !== true);
    text('navscreen-checks-state', bad.length ? 'мешают: ' + bad.length : 'всё готово');
    const blockers = $('navscreen-blockers');
    const reason = status && status.reason && !status.canStart ? String(status.reason) : '';
    if (blockers && blockers.textContent !== reason) blockers.textContent = reason;
    setTone(blockers, reason ? 'warn' : '');
  }

  function goalFromInputs() {
    const x = $('nav-goal-x');
    const y = $('nav-goal-y');
    if (!x || !y) return null;
    if (!String(x.value).trim() || !String(y.value).trim()) return null;
    const xv = Number(String(x.value).replace(',', '.'));
    const yv = Number(String(y.value).replace(',', '.'));
    if (!Number.isFinite(xv) || !Number.isFinite(yv)) return null;
    return {x: xv, y: yv};
  }

  function renderGoalNote() {
    const note = $('navscreen-goal-note');
    if (!note) return;
    const goal = goalFromInputs();
    note.textContent = goal
      ? 'цель: X ' + goal.x.toFixed(2) + ' м, Y ' + goal.y.toFixed(2) + ' м'
      : 'Цель не выбрана.';
    setTone(note, goal ? 'ok' : '');
  }

  function renderMapInfo() {
    const map = window.RCMapView && window.RCMapView.getSnapshot
      ? window.RCMapView.getSnapshot() : null;
    if (!map) return;
    const stats = map.stats || {};
    const settings = map.settings || {};
    text('navscreen-mapinfo',
      'карта ' + Number(map.width || stats.width || 0) + ' × '
      + Number(map.height || stats.height || 0) + ' клеток · '
      + Number(settings.resolution || map.resolution || 0).toFixed(2) + ' м/клетка · изучено '
      + Math.round(Number(stats.coverage || 0) * 100) + '%');
    const pose = map.pose || {x: 0, y: 0, yaw: 0};
    text('navscreen-posemeta',
      'поза: ' + Number(pose.x).toFixed(2) + ' / ' + Number(pose.y).toFixed(2) + ' м · '
      + Math.round(Number(pose.yaw || 0) * 180 / Math.PI) + '° · сканов: '
      + Number(map.scans || 0));
  }

  /* Отметка пройденных шагов: видно, где оператор остановился. */
  function renderSteps(status, currentPlan) {
    const goal = goalFromInputs();
    const plan = currentPlan && Array.isArray(currentPlan.path) ? currentPlan : null;
    const executing = status && ['starting', 'executing', 'stopping'].includes(String(status.phase || ''));
    const states = {
      goal: goal ? 'done' : 'todo',
      plan: plan ? 'done' : (goal ? 'next' : 'todo'),
      drive: executing ? 'done' : (plan && status && status.canStart ? 'next' : 'todo'),
    };
    for (const [name, value] of Object.entries(states)) {
      const step = document.querySelector('[data-navstep="' + name + '"]');
      if (step) step.dataset.state = value;
    }
    const reason = $('navscreen-reason');
    if (reason) {
      let text = '';
      if (!goal) text = 'Шаг 1: выберите цель на карте.';
      else if (!plan) text = 'Шаг 2: постройте маршрут до цели.';
      else if (status && !status.canStart) text = 'Шаг 3 закрыт: ' + (status.reason || 'не все проверки готовности пройдены.');
      reason.textContent = text;
      setTone(reason, text ? 'warn' : '');
    }
  }

  /* Подписка, которую зовёт map_view.js после каждого опроса статуса. */
  function onStatus(status, currentPlan) {
    if (!status || typeof status !== 'object') return;
    renderChecks(status);
    renderGoalNote();
    renderSteps(status, currentPlan);
  }

  function bind() {
    const route = $('navscreen-route');
    if (route) route.addEventListener('click', () => {
      const token = new URLSearchParams(location.search).get('st');
      route.href = '/route' + (token ? '?st=' + encodeURIComponent(token) : '');
    });

    // «Здесь стоит робот»: цель по текущей позе — привычная кнопка RViz.
    const useRobot = $('nav-use-robot');
    if (useRobot) useRobot.addEventListener('click', () => {
      const map = window.RCMapView && window.RCMapView.getSnapshot
        ? window.RCMapView.getSnapshot() : null;
      const pose = map && map.pose;
      if (!pose || !window.RCMapView.setGoal) {
        // #nav-msg принадлежит map_view.js: он перезапишет его при опросе.
        message('Поза робота неизвестна: нет скана лидара.', 'warn');
        return;
      }
      window.RCMapView.setGoal({x: +pose.x, y: +pose.y});
      renderGoalNote();
    });

    for (const id of ['nav-goal-x', 'nav-goal-y']) {
      const input = $(id);
      if (input) input.addEventListener('input', renderGoalNote);
    }
  }

  let timer = null;

  function init() {
    if (timer) return;
    bind();
    renderGoalNote();
    renderMapInfo();
    renderSteps(null);
    // Только локальные данные карты: сеть уже опрашивает map_view.js.
    timer = setInterval(renderMapInfo, 1000);
  }

  window.RCNavigationScreen = {init, onStatus, renderChecks, renderSteps, renderGoalNote, renderMapInfo};
  if (typeof document !== 'undefined' && document.addEventListener) {
    document.addEventListener('DOMContentLoaded', init);
  }
})();
