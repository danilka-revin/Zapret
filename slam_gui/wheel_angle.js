/* Display-only wheel reading. `angle` remains the controller's step estimate;
 * `potAngle` is a separate, same-frame fw26 ADC measurement. Never feed this
 * selection back into steering, watchdog, full-zero or navigation decisions. */
(function () {
  'use strict';
  if (window.RCWheelAngle) return;
  const number = value => typeof value === 'number' && Number.isFinite(value);
  function read(motor, fresh = true) {
    if (!fresh || !motor || motor.online !== true) {
      return {value: null, source: 'unknown', label: 'нет свежих данных'};
    }
    if (motor.firmware >= 26 && motor.pot === true && motor.potMode === true &&
        motor.homed === true && !motor.calibrating && motor.fullZeroFault === false && number(motor.potAngle) &&
        Math.abs(motor.potAngle) <= 180) {
      return {value: motor.potAngle, source: 'pot', label: 'потенциометр A0'};
    }
    const value = number(motor.angle) ? motor.angle : null;
    if (value === null) return {value: null, source: 'unknown', label: 'нет угла'};
    const reason = motor.potMode === false ? 'датчик выключен'
      : motor.pot === false ? 'датчик не готов'
      : motor.firmware < 26 ? 'для угла A0 нужна fw26'
      : 'нет достоверного угла A0';
    return {value, source: 'steps', label: 'по шагам · ' + reason};
  }
  window.RCWheelAngle = {read};
})();
