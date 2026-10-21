'use strict';
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');
class El {
  constructor(tag='div') { this.tagName=tag; this.children=[]; this.parent=null; this.events={}; this.attrs={}; this.checked=false; this.value=''; this.textContent=''; this.open=false; }
  set innerHTML(v) { if(v===''){this.children.forEach(c=>c.parent=null);this.children=[];} }
  get isConnected(){return this.root || !!(this.parent && this.parent.isConnected);}
  appendChild(c){if(c.tagName==='fragment'){for(const child of [...c.children])this.appendChild(child);return c;}this.children.push(c);c.parent=this;return c;}
  addEventListener(k,fn){(this.events[k] ||= []).push(fn);}
  fire(k){for(const fn of this.events[k]||[])fn({target:this});}
  setAttribute(k,v){this.attrs[k]=String(v);}
}
const nodes = new Map();
const $ = id => {if(!nodes.has(id)){const el=new El();el.root=true;nodes.set(id,el);}return nodes.get(id);};
$('log-group-filter').value='all';
const row=(seq,group,level,msg,repeat=1,ts=1000+seq)=>({seq,group,level,msg,repeat,ts,firstTs:1000+seq,source:group,time:String(ts)});
const originals=[row(1,'lidar','info','Скан: 300 точек'), row(2,'wheels','warn','Нет связи с FR'),
 row(3,'battery','info','BMS подключена'),row(4,'system','err','<img src=x onerror=alert(1)>')];
let entries=originals, journalId='first-boot', fail=false, copied='', poll, requests=[];
const context={$, logList:$('log-list'), logCount:$('log-count'), logStatus:$('log-status'),
 logClear:$('log-clear'),logCopy:$('log-copy'),logAuto:$('log-autoscroll'),
 document:{createElement:t=>new El(t),createDocumentFragment:()=>new El('fragment')},
 Date,AbortController,console,
 navigator:{clipboard:{writeText:async text=>{copied=text;}}},
 setInterval:fn=>{poll=fn;},setTimeout:()=>1,clearTimeout(){},
 fetch:async url=>{requests.push(url);return {ok:!fail,status:fail?503:200,json:async()=>({ok:true,data:{entries,journalId,count:entries.length,levels:{err:1}}})};}};
const script=fs.readFileSync('slam_gui/robot.js','utf8');
const start=script.indexOf('    // --- ЛОГИ:');
const end=script.indexOf('    // также свайп индикатор',start);
assert(start>0 && end>start);
vm.runInNewContext('(function(){'+script.slice(start,end)+'})();',context);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const groups=()=>Object.fromEntries($('log-list').children.filter(el=>el.className==='log-group').map(el=>[el.attrs['data-group'],el]));
const lines=()=>Object.values(groups()).flatMap(el=>el.children.filter(child=>child.className==='log-line'));
const text=el=>[el.textContent,...el.children.map(text)].join(' ');
(async()=>{
 await flush();
 assert.equal(lines().length,4);
 assert.equal(groups().lidar.open,false);
 assert.equal(groups().wheels.open,true);
 assert.equal(groups().wheels.attrs['data-alert'],'warn');
 assert.equal(groups().system.attrs['data-alert'],'error');
 assert.match(text(groups().system),/<img src=x onerror=alert\(1\)>/);
 const wheel=groups().wheels;wheel.open=false;wheel.fire('toggle');
 entries=[...originals.slice(1),{...originals[0],msg:'Скан: 312 точек',repeat:22,ts:1050,time:'1050'}];
 await poll();
 assert.equal(lines().length,4,'repeat timestamp/message updates must not create rows');
 assert.match(text(groups().lidar),/312 точек  ×22/);
 assert.equal(groups().wheels.open,false,'user folding persists on polling');
 $('log-group-filter').value='lidar';$('log-group-filter').fire('change');
 assert.deepEqual(Object.keys(groups()),['lidar']);assert.equal(groups().lidar.open,true);
 $('log-group-filter').value='all';$('log-group-filter').fire('change');
 assert.equal(groups().lidar.open,false,'filter-driven expansion must not change the saved fold state');
 $('log-important').checked=true;$('log-important').fire('change');
 assert.deepEqual(Object.keys(groups()),['wheels','system']);assert.equal(lines().length,2);
 assert.equal(groups().wheels.open,true,'important filter exposes warning/error content');
 $('log-copy').fire('click');await flush();
 assert.match(copied,/Колёса и тяга/);assert.match(copied,/Нет связи с FR/);assert(!copied.includes('312 точек'));
 fail=true;await poll();assert.match($('log-status').textContent,/503/);assert(lines().length>=2);
 fail=false;$('log-important').checked=false;$('log-important').fire('change');
 $('log-clear').fire('click');await poll();
 assert.equal(lines().length,1,'local clear does not bring old server rows back every poll');
 assert.match(text($('log-list')),/список очищен/);
 journalId='second-boot';entries=[row(1,'cameras','info','Камера подключена')];await poll();
 assert(groups().cameras);assert(!groups().lidar);assert(!groups().wheels);
 assert(requests.every(url=>url==='api/kiosk_logs?limit=400'));
 console.log('Grouped journal: filters, folding, repeats, restart, clear/copy, XSS and offline state OK');
})().catch(error=>{console.error(error);process.exitCode=1;});
