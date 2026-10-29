'use strict';
// Execute the actual browser script with a minimal DOM to exercise user flows.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname,'../web/index.html'),'utf8');

class Element {
  constructor(id) {
    this.id = id; this.children = []; this.attributes = {}; this.handlers = {};
    this.value = ''; this.textContent = ''; this.disabled = false; this.hidden = false;
    this.checked = false; this.dataset = {};
    this.classList = {toggle: (key,value) => {this.attributes[key] = value;}};
  }
  append(...children) {
    this.children.push(...children);
    if (this.id === 'start' || this.id === 'goal') {
      if (this.value === '' && children.length) this.value = String(children[0].value);
    }
  }
  replaceChildren(...children) {this.children = []; this.value = ''; this.append(...children);}
  addEventListener(name,fn) {this.handlers[name] = fn;}
  setAttribute(name,value) {this.attributes[name] = value;}
}

const elements = Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(m => [m[1],new Element(m[1])]));
elements.heading.value = 'DOWN';
const modes = ['free','competition'].map(mode => {const element = new Element(mode); element.dataset.mode = mode; return element;});
const context = vm.createContext({
  document: {
    getElementById: id => {assert.ok(elements[id],`unknown DOM ID: ${id}`); return elements[id];},
    createElement: tag => new Element(tag),
    querySelectorAll: selector => selector === '[data-mode]' ? modes : [...modes,elements.start,elements.goal],
  },
  EventSource: class {constructor(){context.streamRef = this;}},
  setInterval: () => 1, console, Date, Math, JSON, AbortSignal,
  fetch: async () => ({ok:true,json:async () => ({ok:true})}),
});
vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8'),context);
assert.equal(Number(elements.start.value),1);
assert.equal(Number(elements.goal.value),2);
assert.equal(elements.heading.disabled,false);
assert.equal(elements['exit-start-row'].hidden,false);
assert.equal(elements['exit-start'].checked,false);
assert.equal(elements['exit-start'].disabled,false);
elements['exit-start'].checked = true;
assert.equal(vm.runInContext('settings().exit_start',context),true);
modes[1].handlers.click();
assert.equal(elements.start.children.length,2);
assert.equal(elements.goal.children.length,2);
assert.equal(elements.heading.disabled,true);
assert.equal(elements['align-row'].hidden,true);
assert.equal(elements['exit-start-row'].hidden,true);
assert.equal(vm.runInContext('settings().exit_start',context),false);
elements.start.value = '3';
elements.start.handlers.change();
assert.equal(elements.heading.value,'UP');
modes[0].handlers.click();
assert.equal(elements.start.children.length,9);
assert.equal(elements['align-row'].hidden,false);
assert.equal(elements['exit-start-row'].hidden,false);
elements.start.value = '7';
elements.start.handlers.change();
assert.equal(elements['exit-start'].checked,false);
assert.equal(elements['exit-start'].disabled,true);
elements.start.value = '1';
elements.start.handlers.change();
assert.equal(elements['exit-start'].disabled,false);
elements.goal.value = '7';
elements.align.checked = true;
elements.goal.handlers.change();
assert.equal(elements.align.checked,false);
assert.equal(elements.align.disabled,true);

context.streamRef.onopen();
const owner = vm.runInContext('client',context);
const snapshot = {state:'NAVIGATING',reason:'导航测试',owner,
  settings:{start:7,goal:9},route_count:1,route_index:0,elapsed_s:1.2,wait_s:0,
  odometry:{x_m:.125,y_m:.02,vx_mps:.1,vy_mps:0,wz_rad_s:0},
  odom_session_id:'session-1',motion_epoch:1,yaw_deg:-45,command:[.5,0,0],
  preflight_reason:'',board_fresh:true,odometry_fresh:true,
  board:{connected:true,status_valid:true,pc_control:true,odometry_valid:true,motion_allowed:true},
  plan:{nodes:[7,8,9]},pose:{x:180,y:300,angle:225,edge:[7,8],valid:true},
  trace:[[180,141],[180,300]],events:[{time:'12:00:00',message:'正在导航'}],
  config:{translation_fast_mps:1,translation_slow_mps:.5,turn_fast_deg_s:60,turn_slow_deg_s:30,
    source:'/test/planner.yaml',mission_source:'/test/mission.yaml',parking_offset_mm:275},operations:[]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements.start.disabled,true);
assert.equal(elements['exit-start'].disabled,true);
assert.equal(elements['stop-button'].disabled,false);
assert.equal(elements['planned'].attributes.points,'180,141 180,482 180,800');
assert.equal(elements['car'].attributes.transform,'translate(180 300) rotate(225)');
assert.equal(elements['route'].textContent,'7 → 8 → 9');
snapshot.parking_maneuver = 'EXIT'; snapshot.parking_offset_mm = 275;
snapshot.plan = {nodes:[3],display_points:[[850,850],[821,800]]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['route'].textContent,'斜移出区 · 每轴 275 mm');
assert.equal(elements['planned'].attributes.points,'850,850 821,800');
assert.ok(elements['config-values'].textContent.includes('每轴 275 mm'));
assert.ok(elements['config-path'].textContent.includes('/test/mission.yaml'));
snapshot.parking_maneuver = 'ENTER';
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['route'].textContent,'斜移入区 · 每轴 275 mm');
snapshot.state = 'FAULT'; snapshot.unresolved = true; snapshot.pose.valid = false;
snapshot.reason = '停止未确认';
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['reset-button'].disabled,true);
assert.equal(elements['start-button'].disabled,true);
assert.equal(elements['car'].attributes.invalid,true);
context.streamRef.onerror();
assert.equal(elements['connection'].textContent,'连接中断');
assert.equal(elements['stop-button'].disabled,true);
console.log('Browser user flows passed: modes, settings, progress, faults, connection loss.');
