/* Пресеты руления для экранов робота и веб-пульта.
 *
 * Система знаков — как у прошивки: положительный угол поворачивает колесо
 * вправо (по часовой стрелке, если смотреть сверху), отрицательный — влево.
 * Поэтому «краб влево» — это −45°, а не +45°.
 *
 * Предел угла — ОДНО число на весь стек (``LIMIT_DEG``): прошивка
 * (``STEER_LIMIT_DEG`` в firmware/motor_wheel), сервер
 * (``MasterSource.MAX_STEER_DEG``) и экраны. Дополнительно экран подхватывает
 * фактический предел из телеметрии (``steeringControl.limitDeg``) через
 * ``setLimit()``: если плата прошита старым вариантом и режет ход до ±32°,
 * пресет на 90° честно помечается недоступным, а не молча уезжает на 32°.
 *
 * Ограничения железа, которые нельзя обойти пресетом:
 *   • тяга одна на все четыре колеса, поэтому все углы должны совпадать;
 *   • ход руля ±100° (механика позволяет ±125°): при 90° получается чистый
 *     боковой ход, при меньших углах — диагональ («краб»), а 91…100° — запас
 *     сверх бокового хода;
 *   • разворот на месте требует противоположных направлений левого и правого
 *     бортов, чего эта прошивка не умеет. Такой пресет показывается, но честно
 *     помечен недоступным: лучше объяснить, чем сделать вид, что он работает.
 *
 * Модуль ничего не отправляет на сервер: он только считает углы. Отправкой
 * занимаются robot.js (бортовой экран) и steering.js (веб-пульт).
 */
(function (global) {
  'use strict';

  const LIMIT_DEG = 100;
  const IDS = ['FL', 'FR', 'RL', 'RR'];
  const VEER_DEG = 15;
  const CRAB_DEG = 45;
  const SIDE_DEG = 90;
  /** С какого угла режим считаем боковым ходом, а не диагональю. */
  const SIDE_MIN_DEG = 80;
  /** Механический упор потенциометра: больше него предел выставить нельзя. */
  const TRAVEL_LIMIT_DEG = 125;

  const PRESETS = [
    {
      id: 'straight',
      title: 'Прямо',
      hint: '0° · вперёд / назад',
      angle: 0,
      available: true,
      icon: 'straight',
      description: 'Все колёса смотрят вперёд: обычное движение по прямой.'
    },
    {
      id: 'veer-left',
      title: 'Подрулить влево',
      hint: '−' + VEER_DEG + '° · плавная дуга',
      angle: -VEER_DEG,
      available: true,
      icon: 'left',
      description: 'Все колёса чуть влево: плавный уход влево без резкой перекладки.'
    },
    {
      id: 'veer-right',
      title: 'Подрулить вправо',
      hint: '+' + VEER_DEG + '° · плавная дуга',
      angle: VEER_DEG,
      available: true,
      icon: 'right',
      description: 'Все колёса чуть вправо: плавный уход вправо без резкой перекладки.'
    },
    {
      id: 'crab-left',
      title: 'Краб влево',
      hint: '−' + CRAB_DEG + '° · диагональ влево',
      angle: -CRAB_DEG,
      available: true,
      icon: 'left',
      description: 'Все колёса повёрнуты влево на 45°. При движении вперёд робот едет по диагонали влево-вперёд.'
    },
    {
      id: 'crab-right',
      title: 'Краб вправо',
      hint: '+' + CRAB_DEG + '° · диагональ вправо',
      angle: CRAB_DEG,
      available: true,
      icon: 'right',
      description: 'Все колёса повёрнуты вправо на 45°. При движении вперёд робот едет по диагонали вправо-вперёд.'
    },
    {
      id: 'side-left',
      title: 'Боком влево',
      hint: '−' + SIDE_DEG + '° · чистый боковой ход',
      angle: -SIDE_DEG,
      available: true,
      icon: 'side-left',
      description: 'Колёса перпендикулярно корпусу: при движении вперёд робот едет строго вбок, влево.'
    },
    {
      id: 'side-right',
      title: 'Боком вправо',
      hint: '+' + SIDE_DEG + '° · чистый боковой ход',
      angle: SIDE_DEG,
      available: true,
      icon: 'side-right',
      description: 'Колёса перпендикулярно корпусу: при движении вперёд робот едет строго вбок, вправо.'
    },
    {
      id: 'pivot',
      title: 'Разворот',
      hint: 'недоступен на этой прошивке',
      angle: null,
      available: false,
      icon: 'pivot',
      reason: 'Разворот на месте недоступен: у прошивки одна тяга на все четыре колеса, ' +
        'а для разворота нужны противоположные направления левого и правого бортов. ' +
        'Доступны прямое, диагональное (краб) и боковое перемещение в пределах ±' + LIMIT_DEG + '°.'
    }
  ];

  const byId = Object.create(null);
  PRESETS.forEach((preset) => { byId[preset.id] = preset; });

  /** Фактический предел: по умолчанию LIMIT_DEG, сервер может его уточнить. */
  let limitDeg = LIMIT_DEG;

  function setLimit(value) {
    const n = Number(value);
    if (!Number.isFinite(n) || n <= 0) return limitDeg;
    // Предел больше механического упора gefährен: колесо упрётся в край.
    limitDeg = Math.min(TRAVEL_LIMIT_DEG, Math.round(n));
    return limitDeg;
  }

  function limit() {
    return limitDeg;
  }

  function clampAngle(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return null;
    return Math.max(-limitDeg, Math.min(limitDeg, Math.round(n)));
  }

  /** Причина, по которой пресет не применить к текущему пределу руля (или null). */
  function limitReason(preset) {
    if (!preset || preset.angle === null || preset.angle === undefined) return null;
    if (Math.abs(Number(preset.angle)) <= limitDeg) return null;
    return 'Режим «' + preset.title + '» требует ±' + Math.abs(Number(preset.angle)) +
      '°, а руль сейчас ограничен ±' + limitDeg + '° (так ответил сервер/прошивка). ' +
      'Обновите прошивку плат — предел задаётся одной константой STEER_LIMIT_DEG.';
  }

  /** Пресет с учётом фактического предела: недоступный честно помечен. */
  function effective(presetOrId) {
    const preset = typeof presetOrId === 'string' ? byId[presetOrId] : presetOrId;
    if (!preset) return null;
    const reason = limitReason(preset);
    if (!reason) return preset;
    const copy = Object.assign({}, preset);
    copy.available = false;
    copy.reason = reason;
    copy.overLimit = true;
    return copy;
  }

  /** Все пресеты для отрисовки — с учётом предела, который сообщил сервер. */
  function list() {
    return PRESETS.map(effective);
  }

  function get(id) {
    return effective(id);
  }

  function anglesFor(presetOrId) {
    const preset = effective(presetOrId);
    if (!preset || !preset.available || preset.angle === null || preset.angle === undefined) return null;
    const angle = clampAngle(preset.angle);
    if (angle === null || Math.abs(angle - Number(preset.angle)) > 0.5) return null;
    const angles = {};
    IDS.forEach((id) => { angles[id] = angle; });
    return angles;
  }

  // Один угол для всех колёс или null, если углы разные.
  function uniformAngle(angles) {
    if (!angles || typeof angles !== 'object') return null;
    let common = null;
    for (const id of IDS) {
      const value = clampAngle(angles[id]);
      if (value === null) return null;
      if (common === null) common = value;
      else if (Math.abs(value - common) > 1) return null;
    }
    return common;
  }

  function sideOf(angle) {
    return angle < 0 ? 'влево' : 'вправо';
  }

  function describe(angle) {
    const value = clampAngle(angle);
    if (value === null) return 'угол не задан';
    if (value === 0) return 'прямо';
    const side = sideOf(value);
    if (Math.abs(value) >= SIDE_MIN_DEG) return 'боком ' + side + ' ' + Math.abs(value) + '°';
    return side === 'влево' ? 'влево ' + Math.abs(value) + '°' : 'вправо ' + value + '°';
  }

  function modeFor(angle) {
    const value = clampAngle(angle);
    if (value === null) return 'Угол не задан';
    if (value === 0) return 'Прямо · движение вперёд/назад';
    const side = sideOf(value);
    if (Math.abs(value) >= SIDE_MIN_DEG) {
      return 'Боком ' + side + ' · чистый боковой ход';
    }
    return 'Краб ' + side + ' · диагональ';
  }

  /** Подпись предела для экрана: «±100°». */
  function limitText() {
    return '±' + limitDeg + '°';
  }

  /**
   * Привести ползунок угла к фактическому пределу: min/max, значение и подпись.
   * Возвращает установленное значение — вызывающий знает, что показывать.
   */
  function syncRange(input) {
    if (!input) return null;
    const min = String(-limitDeg);
    const max = String(limitDeg);
    if (input.min !== min) input.min = min;
    if (input.max !== max) input.max = max;
    const value = clampAngle(input.value);
    if (value !== null && String(value) !== String(input.value)) input.value = String(value);
    return value;
  }

  const api = {
    LIMIT_DEG,
    VEER_DEG,
    CRAB_DEG,
    SIDE_DEG,
    SIDE_MIN_DEG,
    TRAVEL_LIMIT_DEG,
    IDS,
    PRESETS,
    byId,
    setLimit,
    limit,
    limitText,
    limitReason,
    effective,
    list,
    get,
    clampAngle,
    anglesFor,
    uniformAngle,
    describe,
    modeFor,
    syncRange
  };

  global.RCSteerPresets = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
