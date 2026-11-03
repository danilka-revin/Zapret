'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');
class El {
  constructor() { this.children = []; this.hidden = false; this.textContent = ''; this.listeners = {}; this.classes = new Set();
    this.classList = {toggle: (c, v) => v ? this.classes.add(c) : this.classes.delete(c), add: c => this.classes.add(c)}; }
  appendChild(el) { this.children.push(el); }
  replaceChildren() { this.children = []; }
  addEventListener(name, fn) { this.listeners[name] = fn; }
}
const elements = new Map();
function el(id) { if (!elements.has(id)) elements.set(id, new El()); return elements.get(id); }
const sections = ['usb', 'serial', 'network', 'bluetooth', 'peripherals', 'peers'];
const data = {hostname:'robot-board', updatedAt:Date.now()/1000, sections:{}};
for (const key of sections) data.sections[key] = {items:[], warnings:[], status:'ok'};
data.sections.usb.items = [{name:'<img src=x onerror=alert(1)>', path:'1-2', authorized:true, vidPid:'1a86:7523'}];
data.sections.serial.items = [{path:'/dev/ttyUSB0', roles:['Колесо FL'], present:true}, {path:'/dev/ttyUSB2',roles:['BMS'],present:false}];
data.sections.network.items = [{name:'wlan0',kind:'Wi-Fi',state:'up',carrier:'1',addresses:['192.168.4.1/24'], profile:'Robot: Wi-Fi'}];
data.sections.bluetooth.items = {adapters:[{name:'BT adapter', address:'11:22:33:44:55:66'}], devices:[{name:'Keyboard',address:'AA:BB:CC:DD:EE:FF',paired:true,connected:false}]};
data.sections.peers.items = [{address:'192.168.4.2',interface:'wlan0',state:['STALE']}];
// Состояние бортового компьютера приходит в том же ответе, что и устройства.
data.system = {
  hostname:'robot-board', status:'warn', stale:false, uptimeS:7200,
  thresholds:{warnTempC:75, critTempC:85, warnMemoryPercent:90, critMemoryPercent:97,
              warnDiskPercent:90, critDiskPercent:97},
  cpu:{available:true, model:'Cortex-A72', cores:4, load1:0.42, usagePercent:37.5, frequencyMHz:1500},
  memory:{available:true, totalMB:3900, usedMB:1800, usedPercent:46.2, swapTotalMB:100, swapUsedPercent:50},
  disk:{available:true, path:'/', totalGB:29.7, freeGB:20.5, usedPercent:31},
  temperature:{available:true, maxC:80.4, hottest:'cpu-thermal', throttleCount:0,
               zones:[{name:'cpu-thermal',celsius:80.4,source:'thermal'},
                      {name:'bcm2711_thermal',celsius:61,celsius_extra:0,source:'thermal'}]},
  warnings:['Процессор 80.4 °C — выше 75 °C, проверьте охлаждение'],
  systemError:null
};
let result = {ok:true, data:null, refreshing:true};
let fail = false, calls = [], nextPoll, visibility;
const document = {hidden:false, body:new El(), getElementById:el, createElement:()=>new El(), addEventListener:(type,fn)=>{visibility=fn;}};
const context = {document, Date, AbortController,
  setTimeout:(fn,ms)=>{if(ms!==3500) nextPoll=fn; return 1;}, clearTimeout(){},
  fetch:async(url,init)=>{calls.push([url,init]); if(fail) throw Error('offline'); return {ok:true,json:async()=>result};}};
context.window = context;
const flush = async()=>{ await new Promise(resolve=>setImmediate(resolve)); };
const text = node => [node.textContent,...node.children.map(text)].join(' ');
(async()=>{
  vm.runInNewContext(fs.readFileSync('slam_gui/connections.js','utf8'),context);
  await flush();
  assert.match(el('connections-status').textContent,/собирает/);
  result = {ok:true,data,refreshing:false,error:null};
  await nextPoll(); await flush();
  assert.equal(el('usb-count').textContent,'1');
  assert.match(text(el('usb-items')),/<img src=x onerror=alert\(1\)>/);
  assert.equal(el('usb-items').children[0].children[0].children[0].textContent,'<img src=x onerror=alert(1)>');
  assert.match(text(el('serial-items')),/Колесо FL/);
  assert.match(text(el('serial-items')),/Отсутствует/);
  assert.match(text(el('network-items')),/192.168.4.1\/24/);
  assert.match(text(el('bluetooth-items')),/Не подключено/);
  assert.match(text(el('bluetooth-items')),/Сопряжено Да/);
  assert.match(text(el('peers-items')),/STALE/);
  assert.match(el('connections-host').textContent,/robot-board/);
  // Бортовой компьютер: температура, память, диск и честные предупреждения.
  assert.equal(el('system-status').textContent,'внимание');
  const sysText = text(el('system-facts'));
  assert.match(sysText,/80\.4 °C/);
  assert.match(sysText,/Процессор 38 %/);
  assert.match(sysText,/Cortex-A72/);
  assert.match(sysText,/Память 46 %/);
  assert.match(sysText,/1800 из 3900 МБ/);
  assert.match(sysText,/подкачка 50 %/);
  assert.match(sysText,/Диск 31 %/);
  assert.match(sysText,/Включён 2 ч 0 мин/);
  assert.equal(el('system-warning').hidden,false);
  assert.match(el('system-warning').textContent,/проверьте охлаждение/);
  assert.match(text(el('system-zones')),/cpu-thermal 80\.4 °C/);
  assert.match(text(el('system-zones')),/bcm2711_thermal 61\.0 °C/);
  // У самого горячего датчика (выше порога 75 °C) есть отметка «hot».
  assert.match(el('system-zones').children[0].className,/\bhot\b/);
  assert(!/\bhot\b/.test(el('system-zones').children[1].className));
  fail = true;
  await nextPoll(); await flush();
  assert(document.body.classes.has('is-stale'));
  assert.match(el('connections-status').textContent,/последние данные/);
  assert.equal(el('usb-count').textContent,'1');
  fail = false;
  data.updatedAt -= 30;
  await nextPoll(); await flush();
  assert.match(el('connections-status').textContent,/устарели/);
  data.updatedAt = Date.now()/1000;
  // Критический перегрев меняет только тон и подпись, числа остаются числами.
  data.system.status = 'critical';
  data.system.temperature = {available:false, maxC:null, zones:[], reason:'датчики не найдены'};
  data.system.warnings = ['Температура недоступна: датчики не найдены'];
  await nextPoll(); await flush();
  assert.equal(el('system-status').textContent,'критично');
  assert.match(text(el('system-facts')),/нет датчика/);
  assert.match(text(el('system-facts')),/датчики не найдены/);
  assert.equal(el('system-zones').children.length,0);
  // Снимка компьютера нет вовсе: экран не рисует нули и не падает.
  data.system = null;
  data.systemError = 'Не удалось прочитать состояние компьютера';
  await nextPoll(); await flush();
  assert.equal(el('system-status').textContent,'—');
  assert.match(text(el('system-facts')),/Не удалось прочитать состояние компьютера/);
  data.sections.usb = {items:[],status:'unavailable',warnings:['Нет доступа к sysfs']};
  await nextPoll(); await flush();
  assert.equal(el('usb-count').textContent,'—');
  assert.equal(el('usb-warning').hidden,false);
  assert.match(text(el('usb-items')),/недоступны/);
  assert(!document.body.classes.has('is-stale'));
  document.hidden = true;
  const count = calls.length;
  await nextPoll(); assert.equal(calls.length,count);
  document.hidden = false; visibility(); await flush(); assert.equal(calls.length,count+1);
  for(const [url,init] of calls){ assert.equal(url,'/api/connections'); assert(!init.method || init.method==='GET'); assert(!init.body); }
  console.log('Connections UI: read-only rendering, XSS safety, Bluetooth states, PC temperature, stale/offline recovery OK');
})().catch(error=>{console.error(error); process.exitCode=1;});
