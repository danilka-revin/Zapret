/* Пресеты руления для экранов робота и веб-пульта.
 *
 * Система знаков — как у прошивки: положительный угол поворачивает колесо
 * вправо (по часовой стрелке, если смотреть сверху), отрицательный — влево.
 * Поэтому «краб влево» — это −28°, а не +28°.
 *
 * Ограничения железа, которые нельзя обойти пресетом:
 *   • тяга одна на все четыре колеса, поэтому все углы должны совпадать;
 *   • ход руля ±32°, чистого бокового хода (90°) нет — «краб» диагональный;
 *   • разворот на месте требует противоположных направлений левого и правого
 *     бортов, чего эта прошивка не умеет. Такой пресет показывается, но честно
 *     помечен недоступным: лучше объяснить, чем сделать вид, что он работает.
 *
 * Модуль ничего не отправляет на сервер: он только считает углы. Отправкой
 * занимаются robot.js (бортовой экран) и steering.js (веб-пульт).
 */
(function (global) {
  'use strict';

  const LIMIT_DEG = 32;
  const IDS = ['FL', 'FR', 'RL', 'RR'];
  const CRAB_DEG = 28;

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
      id: 'crab-left',
      title: 'Краб влево',
      hint: '−28° · диагональ влево',
      angle: -CRAB_DEG,
      available: true,
      icon: 'left',
      description: 'Все колёса повёрнуты влево. При движении вперёд робот едет по диагонали влево-вперёд.'
    },
    {
      id: 'crab-right',
      title: 'Краб вправо',
      hint: '+28° · диагональ вправо',
      angle: CRAB_DEG,
      available: true,
      icon: 'right',
      description: 'Все колёса повёрнуты вправо. При движении вперёд робот едет по диагонали вправо-вперёд.'
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
        'Доступны прямые и крабовые (диагональные) перемещения в пределах ±32°.'
    }
  ];

  const byId = Object.create(null);
  PRESETS.forEach((preset) => { byId[preset.id] = preset; });

  function clampAngle(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return null;
    return Math.max(-LIMIT_DEG, Math.min(LIMIT_DEG, Math.round(n)));
  }

  function anglesFor(presetOrId) {
    const preset = typeof presetOrId === 'string' ? byId[presetOrId] : presetOrId;
    if (!preset || !preset.available || preset.angle === null || preset.angle === undefined) return null;
    const angle = clampAngle(preset.angle);
    if (angle === null) return null;
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

  function describe(angle) {
    const value = clampAngle(angle);
    if (value === null) return 'угол не задан';
    if (value === 0) return 'прямо';
    return value < 0 ? 'влево ' + Math.abs(value) + '°' : 'вправо ' + value + '°';
  }

  function modeFor(angle) {
    const value = clampAngle(angle);
    if (value === null) return 'Угол не задан';
    if (value === 0) return 'Прямо · движение вперёд/назад';
    return (value < 0 ? 'Краб влево' : 'Краб вправо') + ' · диагональ';
  }

  const api = {
    LIMIT_DEG,
    CRAB_DEG,
    IDS,
    PRESETS,
    byId,
    clampAngle,
    anglesFor,
    uniformAngle,
    describe,
    modeFor
  };

  global.RCSteerPresets = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
