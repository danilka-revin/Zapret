/* ============================================================================
 * console-core.js — ядро сервисного пульта RUS SLAM (без DOM).
 *
 * Технический план и описание — docs/SERVICE_CONSOLE.md.
 * Модуль работает и в браузере (window.RS), и в Node (module.exports) —
 * используется узлами пульта и модульными тестами gui/tests/console.test.js.
 *
 * Статистика работы (рейсы, пробег, энергия) — на вкладке «Сводка» основного
 * экрана и считается на сервере: robot_control/stats.py.
 *
 * Состав:
 *   RS.pack   — модель АКБ 12S3P LiFePO4 (SOC по OCV, энергия, пороги);
 *   RS.uart   — кадры протокола обмена (10 Б команда, 16 Б телеметрия, CRC16);
 *   RS.PinVault — PIN-хранилище замка ячейки (SHA-256 + соль, блокировки, аудит);
 *   RS.fmt     — форматирование чисел/времени.
 * ========================================================================== */
(function (global) {
  'use strict';

  const RS = (global.RS = global.RS || {});

  /* ======================================================================
   * 1. Форматирование
   * ==================================================================== */
  const fmt = {
    num(v, d) { return Number(v || 0).toFixed(d === undefined ? 1 : d); },
    int(v) { return String(Math.round(Number(v) || 0)); },
    volts(v) { return fmt.num(v, 2) + ' В'; },
    amps(v) { return fmt.num(v, 1) + ' А'; },
    watts(v) { return fmt.int(v) + ' Вт'; },
    wh(v) { return fmt.int(v) + ' Вт·ч'; },
    kg(v) { return fmt.int(v) + ' кг'; },
    km(m) {
      const km = (Number(m) || 0) / 1000;
      return (km < 1 ? fmt.int(km * 1000) + ' м' : fmt.num(km, 2) + ' км');
    },
    /** Секунды → «чч:мм:сс» */
    dur(sec) {
      sec = Math.max(0, Math.round(Number(sec) || 0));
      const h = Math.floor(sec / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
      return [h, m, s].map((x) => String(x).padStart(2, '0')).join(':');
    },
    /** Секунды → «мм:сс» (компактно) */
    durShort(sec) {
      sec = Math.max(0, Math.round(Number(sec) || 0));
      const m = Math.floor(sec / 60), s = sec % 60;
      return String(m).padStart(2, '0') + ':' + String(s).padStart(2, '0');
    },
    hex(bytes) {
      const a = Array.from(bytes || []);
      return a.map((b) => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');
    },
    date(ts) {
      const d = new Date(ts || Date.now());
      return d.toLocaleString('ru-RU', { hour12: false });
    },
    time(ts) {
      const d = new Date(ts || Date.now());
      return d.toLocaleTimeString('ru-RU', { hour12: false });
    },
  };

  /* ======================================================================
   * 2. Модель АКБ 12S3P LiFePO4 (36 × 32700, 38.4 В, 18.6 А·ч, 714.2 Вт·ч)
   *    Данные: docs/TECHNICAL_SPECIFICATION.md §8, config/safety.yaml
   * ==================================================================== */
  const PACK = {
    series: 12,
    parallel: 3,
    cellAh: 6.2,
    nominalV: 38.4,
    fullV: 43.8,
    emptyV: 30.0,
    capacityAh: 18.6,
    capacityWh: 714.2,
    bmsA: 60,
    fuseA: 25,
    internalR: 0.075,          // эквивалентное сопротивление сборки + проводка, Ом
    thresholds: {
      lowV: 35.5,              // ползучий режим (safety_node battery_low_v)
      criticalV: 33.5,         // аварийный стоп (battery_critical_v)
      tempWarnC: 45,
      tempMaxC: 60,
    },
    // Кривая OCV LiFePO4: [напряжение на элемент, SOC %]
    ocv: [
      [2.50, 0], [2.80, 3], [3.00, 7], [3.10, 10], [3.20, 15], [3.25, 30],
      [3.30, 50], [3.33, 70], [3.35, 85], [3.40, 95], [3.45, 98], [3.65, 100],
    ],
  };

  function _interp(table, x, col) {
    // col = 0: искать по напряжению → SOC; col = 1: искать по SOC → напряжение
    const key = col === 0 ? 0 : 1, val = col === 0 ? 1 : 0;
    if (x <= table[0][key]) return table[0][val];
    const last = table[table.length - 1];
    if (x >= last[key]) return last[val];
    for (let i = 1; i < table.length; i++) {
      const a = table[i - 1], b = table[i];
      if (x <= b[key]) {
        const k = (x - a[key]) / (b[key] - a[key] || 1);
        return a[val] + k * (b[val] - a[val]);
      }
    }
    return last[val];
  }

  /** SOC (%) по напряжению холостого хода пакета (В). */
  function socFromVoltage(packV) {
    return Math.max(0, Math.min(100, _interp(PACK.ocv, (Number(packV) || 0) / PACK.series, 0)));
  }

  /** Напряжение холостого хода пакета (В) по SOC (%). */
  function voltageFromSoc(soc) {
    return _interp(PACK.ocv, Math.max(0, Math.min(100, Number(soc) || 0)), 1) * PACK.series;
  }

  /**
   * Терминальное напряжение пакета: OCV минус просадка под разрядом
   * (или плюс подъём при заряде).
   * @param {number} soc   — SOC, %
   * @param {number} currentA — ток, А (>0 разряд, <0 заряд)
   */
  function terminalVoltage(soc, currentA) {
    const ocv = voltageFromSoc(soc);
    const i = Number(currentA) || 0;
    return Math.max(0, ocv - i * PACK.internalR);
  }

  /**
   * Шаг разряда по энергии: dE = U·I·dt/3600, dSOC = dE / E_max · 100 %.
   * @returns {number} новый SOC, %
   */
  function stepSoc(soc, voltageV, currentA, dtSec) {
    const dE = Math.max(0, Number(voltageV) || 0) * Math.max(0, Number(currentA) || 0)
      * Math.max(0, Number(dtSec) || 0) / 3600;
    return Math.max(0, Math.min(100, (Number(soc) || 0) - dE / PACK.capacityWh * 100));
  }

  function remainingWh(soc) { return PACK.capacityWh * Math.max(0, Math.min(100, Number(soc) || 0)) / 100; }

  /** Запас хода, км, по текущему расходу (Вт·ч/км); при нуле — паспортные 22 км. */
  // Запас хода считается только по реальному расходу (Вт·ч/км из сводки работы).
  // Без него — null, а не выдуманная норма: экран покажет «—».
  function rangeKm(soc, whPerKm) {
    const c = Number(whPerKm);
    return c > 5 ? remainingWh(soc) / c : null;
  }

  /** Уровень по напряжению: 3 — норма, 2 — низкий, 1 — критический, 0 — авария. */
  function packLevel(voltageV) {
    const v = Number(voltageV) || 0;
    if (v >= PACK.thresholds.lowV) return { code: 3, label: 'НОРМА', cls: 'green' };
    if (v >= PACK.thresholds.criticalV) return { code: 2, label: 'НИЗКИЙ', cls: 'amber' };
    if (v >= PACK.emptyV) return { code: 1, label: 'КРИТИЧЕСКИЙ', cls: 'red' };
    return { code: 0, label: 'АВАРИЯ', cls: 'red' };
  }

  /** Снимок состояния пакета для панели «АКБ». */
  function packSnapshot(opts) {
    const o = opts || {};
    const soc = Math.max(0, Math.min(100, Number(o.soc) || 0));
    const voltage = Number(o.voltage) || terminalVoltage(soc, o.current || 0);
    const current = Number(o.current) || 0;
    const cellV = voltage / PACK.series;
    const spread = 0.012 + (1 - soc / 100) * 0.02;
    const level = packLevel(voltage);
    return {
      soc, voltage, ocv: voltageFromSoc(soc), current,
      power: voltage * current,
      charging: !!o.charging,
      remainingWh: remainingWh(soc),
      rangeKm: rangeKm(soc, o.whPerKm),
      level,
      tempC: Number(o.tempC) || 28,
      cycles: o.cycles === null || o.cycles === undefined ? null : (Number(o.cycles) || 0),
      cells: {
        min: +(cellV - spread).toFixed(3),
        max: +(cellV + spread * 1.25).toFixed(3),
        delta: +(spread * 2.25).toFixed(3),
      },
      thresholds: PACK.thresholds,
    };
  }

  RS.pack = {
    PACK,
    socFromVoltage, voltageFromSoc, terminalVoltage, stepSoc,
    remainingWh, rangeKm, packLevel, packSnapshot,
  };

  /* ======================================================================
   * 3. Протокол обмена (зеркало ros2_ws/src/rus_slam_base/.../protocol.py)
   *    См. docs/SERIAL_PROTOCOL.md
   * ==================================================================== */
  const CMD_SYNC0 = 0xAA, CMD_SYNC1 = 0x55;
  const TLM_SYNC0 = 0xBB, TLM_SYNC1 = 0x44;
  const CMD_LEN = 10, TLM_LEN = 16;
  const FLAG_ENABLE = 0x01, FLAG_HOME = 0x02;
  const STATUS_ENABLED = 0x01, STATUS_HOMED = 0x02, STATUS_FAULT = 0x04;
  const MAX_PWM = 1000, MAX_STEER_CDEG = 32767;
  const MODULE_IDS = [1, 2, 3, 4];
  const MODULE_NAMES = { 1: 'FL', 2: 'FR', 3: 'RL', 4: 'RR' };

  /** CRC16/Modbus (полином 0xA001, init 0xFFFF) — эталонный вектор "123456789" → 0x4B37. */
  function crc16(bytes) {
    let crc = 0xFFFF;
    for (const byte of bytes) {
      crc ^= byte & 0xFF;
      for (let i = 0; i < 8; i++) {
        crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) & 0xFFFF : (crc >> 1) & 0xFFFF;
      }
    }
    return crc & 0xFFFF;
  }

  function _i16(v) { return ((Math.round(v) % 65536) + 65536) % 65536; }

  /**
   * Кадр команды ПК → МК (10 байт).
   * @param {{moduleId:number, steerCdeg:number, pwm:number, enable:boolean, home:boolean}} c
   */
  function buildCommand(c) {
    const steer = Math.max(-MAX_STEER_CDEG, Math.min(MAX_STEER_CDEG, Math.round(c.steerCdeg || 0)));
    const pwm = Math.max(-MAX_PWM, Math.min(MAX_PWM, Math.round(c.pwm || 0)));
    let flags = 0;
    if (c.enable) flags |= FLAG_ENABLE;
    if (c.home) flags |= FLAG_HOME;
    const out = new Uint8Array(CMD_LEN);
    out[0] = CMD_SYNC0; out[1] = CMD_SYNC1; out[2] = c.moduleId & 0xFF;
    const s = _i16(steer), p = _i16(pwm);
    out[3] = (s >> 8) & 0xFF; out[4] = s & 0xFF;
    out[5] = (p >> 8) & 0xFF; out[6] = p & 0xFF;
    out[7] = flags;
    const crc = crc16(out.slice(0, 8));
    out[8] = (crc >> 8) & 0xFF; out[9] = crc & 0xFF;
    return out;
  }

  /** Кадр телеметрии МК → ПК (16 байт) — для тестов и модели стенда. */
  function buildTelemetry(t) {
    const out = new Uint8Array(TLM_LEN);
    out[0] = TLM_SYNC0; out[1] = TLM_SYNC1;
    out[2] = t.moduleId & 0xFF;
    out[3] = t.status & 0xFF;
    const s = _i16(t.steerCdeg || 0), e = _i16(t.encDelta || 0), p = _i16(t.pwmActual || 0);
    out[4] = (s >> 8) & 0xFF; out[5] = s & 0xFF;
    out[6] = (e >> 8) & 0xFF; out[7] = e & 0xFF;
    out[8] = (p >> 8) & 0xFF; out[9] = p & 0xFF;
    out[10] = (t.vbatCv >> 8) & 0xFF; out[11] = t.vbatCv & 0xFF;
    out[12] = (t.currentMa >> 8) & 0xFF; out[13] = t.currentMa & 0xFF;
    const crc = crc16(out.slice(0, 14));
    out[14] = (crc >> 8) & 0xFF; out[15] = crc & 0xFF;
    return out;
  }

  /** Разбор кадра телеметрии; null при неверной длине/CRC. */
  function parseTelemetry(bytes) {
    const b = Array.from(bytes || []);
    if (b.length < TLM_LEN) return null;
    if (b[0] !== TLM_SYNC0 || b[1] !== TLM_SYNC1) return null;
    const crc = (b[14] << 8) | b[15];
    if (crc !== crc16(b.slice(0, 14))) return null;
    const i16 = (hi, lo) => (((hi << 8) | lo) << 16) >> 16;
    return {
      moduleId: b[2], status: b[3],
      steerCdeg: i16(b[4], b[5]), encDelta: i16(b[6], b[7]), pwmActual: i16(b[8], b[9]),
      vbatCv: (b[10] << 8) | b[11], currentMa: (b[12] << 8) | b[13],
      enabled: !!(b[3] & STATUS_ENABLED), homed: !!(b[3] & STATUS_HOMED),
      fault: !!(b[3] & STATUS_FAULT),
      get steerRad() { return this.steerCdeg / 100 * Math.PI / 180; },
      get vbatV() { return this.vbatCv / 100; },
      get currentA() { return this.currentMa / 1000; },
    };
  }

  const uart = {
    CMD_LEN, TLM_LEN, FLAG_ENABLE, FLAG_HOME, STATUS_ENABLED, STATUS_HOMED, STATUS_FAULT,
    MAX_PWM, MAX_STEER_CDEG, MODULE_IDS, MODULE_NAMES,
    crc16, buildCommand, buildTelemetry, parseTelemetry, hex: fmt.hex,
    degToCdeg(deg) { return Math.round((Number(deg) || 0) * 100); },
    pwmFromPercent(pct) { return Math.max(-100, Math.min(100, Number(pct) || 0)) / 100 * MAX_PWM; },
  };
  RS.uart = uart;

  /* ======================================================================
   * 4. Хранилище (localStorage с деградацией до памяти)
   * ==================================================================== */
  function memoryStorage() {
    const m = new Map();
    return {
      getItem: (k) => (m.has(k) ? m.get(k) : null),
      setItem: (k, v) => m.set(k, String(v)),
      removeItem: (k) => m.delete(k),
    };
  }

  function safeStorage(name) {
    try {
      const s = global[name];
      if (!s) return memoryStorage();
      const probe = '__rs_probe__';
      s.setItem(probe, '1');
      s.removeItem(probe);
      return s;
    } catch (e) {
      return memoryStorage();
    }
  }

  RS.safeStorage = safeStorage;

  /* ======================================================================
   * 5. Хеширование (SHA-256: WebCrypto → node:crypto → резервный FNV)
   * ==================================================================== */
  /**
   * Кодировщик UTF-8: TextEncoder есть не везде (старые браузеры, jsdom),
   * поэтому держим компактный резервный вариант.
   */
  const utf8 = (function () {
    if (typeof TextEncoder === 'function') {
      const enc = new TextEncoder();
      return { encode: (str) => enc.encode(str) };
    }
    return {
      encode(str) {
        str = String(str);
        const out = [];
        for (let i = 0; i < str.length; i++) {
          const c = str.charCodeAt(i);
          if (c < 0x80) out.push(c);
          else if (c < 0x800) out.push(0xC0 | (c >> 6), 0x80 | (c & 63));
          else if (c >= 0xD800 && c < 0xDC00 && i + 1 < str.length) {
            const cp = 0x10000 + ((c - 0xD800) << 10) + (str.charCodeAt(++i) - 0xDC00);
            out.push(0xF0 | (cp >> 18), 0x80 | ((cp >> 12) & 63), 0x80 | ((cp >> 6) & 63), 0x80 | (cp & 63));
          } else out.push(0xE0 | (c >> 12), 0x80 | ((c >> 6) & 63), 0x80 | (c & 63));
        }
        return new Uint8Array(out);
      },
    };
  })();
  RS.utf8 = utf8;

  async function sha256Hex(text) {
    const data = typeof text === 'string' ? utf8.encode(text) : text;
    try {
      if (global.crypto && global.crypto.subtle) {
        const digest = await global.crypto.subtle.digest('SHA-256', data);
        return Array.from(new Uint8Array(digest)).map((b) => b.toString(16).padStart(2, '0')).join('');
      }
    } catch (e) { /* переходим к резервному пути */ }
    try {
      // Node без global.crypto (старые версии)
      if (typeof require === 'function') {
        const nodeCrypto = require('crypto');
        return nodeCrypto.createHash('sha256').update(text).digest('hex');
      }
    } catch (e) { /* игнорируем */ }
    // Резервный (ослабленный) путь — только для сред без WebCrypto
    let h1 = 0x811c9dc5, h2 = 0x01000193;
    for (let i = 0; i < String(text).length; i++) {
      const c = String(text).charCodeAt(i);
      h1 = ((h1 ^ c) * 0x01000193) >>> 0;
      h2 = ((h2 + c * (i + 1)) ^ (h2 << 5)) >>> 0;
    }
    return 'fnv:' + h1.toString(16).padStart(8, '0') + h2.toString(16).padStart(8, '0');
  }

  function randomHex(nBytes) {
    const a = new Uint8Array(nBytes);
    if (global.crypto && global.crypto.getRandomValues) global.crypto.getRandomValues(a);
    else for (let i = 0; i < a.length; i++) a[i] = Math.floor(Math.random() * 256);
    return Array.from(a).map((b) => b.toString(16).padStart(2, '0')).join('');
  }

  /* ======================================================================
   * 6. Замок ячейки хранения (PIN + блокировка + аудит-цепочка)
   * ==================================================================== */
  const LOCK_KEY = 'rus_slam_lock_v1';
  const DEFAULT_PIN = '7232';

  class PinVault {
    /**
     * @param {Storage} [storage] — localStorage-совместимое хранилище
     * @param {{key?:string, maxAttempts?:number, lockMs?:number, defaultPin?:string}} [opts]
     */
    constructor(storage, opts) {
      const o = opts || {};
      this.storage = storage || safeStorage('localStorage');
      this.key = o.key || LOCK_KEY;
      this.maxAttempts = o.maxAttempts || 5;
      this.lockMs = o.lockMs || 30000;
      this.defaultPin = o.defaultPin || DEFAULT_PIN;
      this.onEvent = typeof o.onEvent === 'function' ? o.onEvent : null;
      this.data = null;
    }

    async init() {
      let raw = null;
      try { raw = JSON.parse(this.storage.getItem(this.key) || 'null'); } catch (e) { raw = null; }
      if (!raw || typeof raw !== 'object') {
        const salt = randomHex(16);
        this.data = {
          version: 1, salt, pinHash: await sha256Hex(salt + ':' + this.defaultPin),
          fails: 0, lockUntil: 0, audit: [], changedAt: Date.now(),
        };
        this._persist();
      } else {
        this.data = raw;
        this.data.audit = Array.isArray(raw.audit) ? raw.audit : [];
        this.data.fails = Number(raw.fails) || 0;
        this.data.lockUntil = Number(raw.lockUntil) || 0;
      }
      return this.data;
    }

    _persist() {
      try { this.storage.setItem(this.key, JSON.stringify(this.data)); } catch (e) { /* нет места */ }
    }

    async _lock(pin) { return sha256Hex(this.data.salt + ':' + pin); }

    lockRemainingMs(now) {
      const t = Number(now) || Date.now();
      return Math.max(0, (this.data ? this.data.lockUntil : 0) - t);
    }

    isLocked(now) { return this.lockRemainingMs(now) > 0; }

    static validatePinFormat(pin) {
      const s = String(pin == null ? '' : pin).trim();
      if (!/^\d{4,8}$/.test(s)) return { ok: false, reason: 'PIN — от 4 до 8 цифр' };
      return { ok: true };
    }

    /**
     * Проверка PIN.
     * @returns {Promise<{ok:boolean, reason:'ok'|'wrong'|'locked'|'format', fails:number, remainingMs:number}>}
     */
    async verify(pin) {
      if (!this.data) await this.init();
      const now = Date.now();
      const rem = this.lockRemainingMs(now);
      if (rem > 0) {
        this._emit('unlock', false);
        await this._audit('unlock', false, 'заблокировано, осталось ' + Math.ceil(rem / 1000) + ' с');
        return { ok: false, reason: 'locked', fails: this.data.fails, remainingMs: rem };
      }
      const chk = PinVault.validatePinFormat(pin);
      if (!chk.ok) return { ok: false, reason: 'format', fails: this.data.fails, remainingMs: 0 };
      const hash = await this._lock(String(pin).trim());
      if (hash === this.data.pinHash) {
        this.data.fails = 0;
        this.data.lockUntil = 0;
        this.data.lastOpenAt = now;
        this._persist();
        this._emit('unlock', true);
        await this._audit('unlock', true, 'доступ разрешён');
        return { ok: true, reason: 'ok', fails: 0, remainingMs: 0 };
      }
      // сбрасываем остывшую блокировку
      if (this.data.lockUntil && this.data.lockUntil <= now) { this.data.fails = 0; this.data.lockUntil = 0; }
      this.data.fails = (this.data.fails || 0) + 1;
      if (this.data.fails >= this.maxAttempts) {
        this.data.lockUntil = now + this.lockMs;
        this.data.fails = 0;
      }
      this._persist();
      this._emit('unlock', false);
      await this._audit('unlock', false, 'неверный PIN');
      return {
        ok: false, reason: 'wrong',
        fails: this.data.fails, remainingMs: this.lockRemainingMs(),
      };
    }

    /** Смена PIN: требуется текущий. */
    async setPin(currentPin, newPin) {
      if (!this.data) await this.init();
      const res = await this.verify(currentPin);
      if (!res.ok) {
        return { ok: false, reason: res.reason === 'locked' ? 'locked' : 'current', detail: 'текущий PIN неверен' };
      }
      const chk = PinVault.validatePinFormat(newPin);
      if (!chk.ok) return { ok: false, reason: 'format', detail: chk.reason };
      if (String(newPin).trim() === String(currentPin).trim()) {
        return { ok: false, reason: 'same', detail: 'новый PIN совпадает с текущим' };
      }
      this.data.salt = randomHex(16);
      this.data.pinHash = await this._lock(String(newPin).trim());
      this.data.changedAt = Date.now();
      this._persist();
      this._emit('pin_change', true);
      await this._audit('pin_change', true, 'PIN изменён');
      return { ok: true };
    }

    /** Сброс к заводскому PIN (требуется текущий). */
    async resetToDefault(currentPin) {
      return this.setPin(currentPin, this.defaultPin);
    }

    _emit(action, ok) {
      if (this.onEvent) {
        try { this.onEvent(action, ok); } catch (e) { /* не ломаем замок из-за статистики */ }
      }
    }

    audit(limit) {
      const a = (this.data && this.data.audit) || [];
      return a.slice(-(limit || 20)).reverse();
    }

    /** Запись аудита с хеш-цепочкой (защита от подделки журнала). */
    async _audit(action, ok, detail) {
      const prev = this.data.audit.length ? this.data.audit[this.data.audit.length - 1].hash : 'genesis';
      const ts = Date.now();
      const hash = await sha256Hex(prev + '|' + ts + '|' + action + '|' + (ok ? 1 : 0) + '|' + (detail || ''));
      this.data.audit.push({ ts, action, ok: !!ok, detail: detail || '', prev, hash });
      if (this.data.audit.length > 200) this.data.audit.shift();
      this._persist();
      return { ts, action, ok, detail, prev, hash };
    }

    /** Проверка целостности журнала аудита. */
    async verifyAudit() {
      if (!this.data) await this.init();
      let prev = 'genesis';
      for (let i = 0; i < this.data.audit.length; i++) {
        const e = this.data.audit[i];
        if (e.prev !== prev) return { ok: false, brokenAt: i };
        const hash = await sha256Hex(prev + '|' + e.ts + '|' + e.action + '|' + (e.ok ? 1 : 0) + '|' + (e.detail || ''));
        if (hash !== e.hash) return { ok: false, brokenAt: i };
        prev = e.hash;
      }
      return { ok: true, brokenAt: -1 };
    }
  }

  RS.PinVault = PinVault;
  RS.DEFAULT_PIN = DEFAULT_PIN;

  RS.fmt = fmt;
  RS.sha256Hex = sha256Hex;
  RS.randomHex = randomHex;
  RS.version = '1.0.0';

  if (typeof module !== 'undefined' && module.exports) module.exports = RS;
})(typeof globalThis !== 'undefined' ? globalThis : this);
