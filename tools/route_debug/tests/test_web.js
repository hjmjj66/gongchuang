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
const modes = ['free','competition','screen'].map(mode => {const element = new Element(mode); element.dataset.mode = mode; return element;});
const context = vm.createContext({
  document: {
    getElementById: id => {assert.ok(elements[id],`unknown DOM ID: ${id}`); return elements[id];},
    createElement: tag => new Element(tag),
    createElementNS: (namespace,tag) => new Element(tag),
    querySelectorAll: selector => selector === '[data-mode]' ? modes : [...modes,elements.start,elements.goal],
  },
  EventSource: class {constructor(){context.streamRef = this;}},
  setInterval: () => 1, console, Date, Math, JSON, AbortSignal,
  fetch: async () => ({ok:true,json:async () => ({ok:true})}),
});
vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8'),context);
context.roadsFixture = {1:[7,4],2:[4,1],3:[7,8],4:[4,5],5:[1,2],6:[8,5],7:[5,2],8:[8,9],9:[5,6],10:[2,3],11:[9,6],12:[6,3]};
context.streamRef.onmessage({data:JSON.stringify({state:'IDLE',roads:context.roadsFixture})});
const roadChoices = elements['road-choices'].children;
assert.equal(roadChoices.length,12);
assert.equal(roadChoices[0].children[2].attributes.transform,'translate(340 141)');
assert.equal(roadChoices[9].children[2].attributes.transform,'translate(821 641)');
roadChoices[0].handlers.click();
roadChoices[4].handlers.keydown({key:' ',preventDefault(){}});
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[7,4],[1,2]]');
assert.equal(roadChoices[0].attributes['aria-pressed'],'true');
assert.equal(roadChoices[4].attributes.blocked,true);
assert.equal(elements['blocked-summary'].textContent,'障碍路段：1、5（双向禁行）');
assert.equal(elements['clear-blocked'].disabled,false);
vm.runInContext("pending = 'submitting'; updateControls()",context);
roadChoices[0].handlers.click();
elements['clear-blocked'].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[7,4],[1,2]]');
vm.runInContext('pending = null; updateControls()',context);
assert.equal(Number(elements.start.value),1);
assert.equal(Number(elements.goal.value),2);
assert.equal(elements.heading.disabled,false);
assert.equal(elements['exit-start-row'].hidden,false);
assert.equal(elements['exit-start'].checked,false);
assert.equal(elements['exit-start'].disabled,false);
elements['exit-start'].checked = true;
assert.equal(vm.runInContext('settings().exit_start',context),true);
modes[1].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[7,4],[1,2]]');
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
  roads:context.roadsFixture,settings:{start:7,goal:9,blocked_edges:[[4,7],[2,1]]},route_count:1,route_index:0,elapsed_s:1.2,wait_s:0,
  odometry:{x_m:.125,y_m:.02,vx_mps:.1,vy_mps:0,wz_rad_s:0},
  odom_session_id:'session-1',motion_epoch:1,yaw_deg:-45,command:[.5,0,0],
  preflight_reason:'',board_fresh:true,odometry_fresh:true,
  board:{connected:true,status_valid:true,pc_control:true,odometry_valid:true,motion_allowed:true},
  plan:{nodes:[7,8,9]},pose:{x:180,y:300,angle:225,edge:[7,8],valid:true},
  trace:[[180,141],[180,300]],events:[{time:'12:00:00',message:'正在导航'}],
  config:{translation_pid_kp:2,translation_pid_ki:0,translation_pid_kd:.1,
    translation_max_vx_mps:.7,translation_max_vy_mps:.4,translation_arrival_tolerance_mm:3,
    angle_pid_kp:2,angle_pid_ki:0,angle_pid_kd:.1,
    angle_max_speed_deg_s:120,turn_arrival_tolerance_deg:.2,
    source:'/test/planner.yaml',mission_source:'/test/mission.yaml',parking_offset_mm:275},operations:[]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements.start.disabled,true);
assert.equal(elements['exit-start'].disabled,true);
assert.equal(elements['stop-button'].disabled,false);
assert.equal(elements['clear-blocked'].disabled,true);
assert.equal(roadChoices[0].attributes['aria-disabled'],'true');
assert.equal(roadChoices[0].attributes.tabindex,'-1');
roadChoices[0].handlers.click();
roadChoices[2].handlers.keydown({key:'Enter',preventDefault(){}});
elements['clear-blocked'].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[7,4],[1,2]]');
assert.equal(elements['ready-reason'].textContent,'链路就绪');
assert.equal(elements['planned'].attributes.points,'180,141 180,482 180,800');
assert.equal(elements['car'].attributes.transform,'translate(180 300) rotate(225)');
assert.equal(elements['route'].textContent,'7 → 8 → 9');
snapshot.parking_maneuver = 'EXIT'; snapshot.parking_offset_mm = 275;
snapshot.plan = {nodes:[3],display_points:[[850,850],[821,800]]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['route'].textContent,'斜移出区 · 每轴 275 mm');
assert.equal(elements['planned'].attributes.points,'850,850 821,800');
assert.ok(elements['config-values'].textContent.includes('每轴 275 mm'));
assert.ok(elements['config-values'].textContent.includes('vx 上限 0.7 m/s'));
assert.ok(elements['config-values'].textContent.includes('vy 上限 0.4 m/s'));
assert.ok(elements['config-values'].textContent.includes('各轴允许误差 ±3 mm'));
assert.ok(elements['config-values'].textContent.includes('角速度上限 120°/s'));
assert.ok(elements['config-values'].textContent.includes('允许误差 ±0.2°'));
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
snapshot.state = 'STOPPED'; snapshot.unresolved = false;
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(roadChoices[0].attributes['aria-disabled'],'false');
roadChoices[0].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[1,2]]');
elements['clear-blocked'].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[]');
assert.equal(elements['blocked-summary'].textContent,'全部道路畅通');
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[]');
snapshot.state = 'READY'; snapshot.owner = 'another-browser';
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(roadChoices[0].attributes['aria-pressed'],'true');
assert.equal(roadChoices[0].attributes['aria-disabled'],'true');
roadChoices[0].handlers.click();
assert.equal(JSON.stringify(vm.runInContext('settings().blocked_edges',context)),'[[7,4],[1,2]]');
context.streamRef.onerror();
assert.equal(elements['connection'].textContent,'连接中断');
assert.equal(elements['stop-button'].disabled,true);
context.streamRef.onopen();
snapshot.state = 'IDLE'; snapshot.owner = owner; snapshot.settings = null;
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
modes[2].handlers.click();
assert.equal(elements.start.disabled,true);
assert.equal(elements.goal.disabled,true);
assert.equal(elements.heading.disabled,true);
assert.equal(elements.start.value,'');
assert.equal(elements.goal.value,'');
assert.equal(elements['start-marker'].attributes.visibility,'hidden');
assert.equal(elements['clear-blocked'].hidden,true);
assert.equal(roadChoices[0].attributes['aria-disabled'],'true');
assert.equal(JSON.stringify(vm.runInContext('settings()',context)),'{"mode":"screen"}');
assert.equal(elements['align-row'].hidden,true);
assert.equal(elements['exit-start-row'].hidden,true);
snapshot.state = 'SCREEN_SETUP'; snapshot.settings = {mode:'screen',blocked_edges:[[4,7]]};
snapshot.screen_needs_reset = true;
snapshot.screen = {port:'/dev/test-screen',connected:true,accepting:true,error:''};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['state'].textContent,'等待屏幕配置');
assert.equal(elements['start-button'].disabled,true);
assert.equal(elements['reset-button'].disabled,false);
assert.equal(elements['stop-button'].disabled,false);
assert.equal(elements['blocked-summary'].textContent,'障碍路段：1（双向禁行）');
assert.ok(elements['screen-status'].textContent.includes('接收本轮配置'));
assert.equal(elements.start.value,'');
snapshot.state = 'INITIALIZING'; snapshot.wait_s = 1.8;
snapshot.settings = {mode:'screen',start:3,goal:1,heading:'UP',blocked_edges:[[4,7]]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(Number(elements.start.value),3);
assert.equal(Number(elements.goal.value),1);
assert.equal(elements.heading.value,'UP');
assert.equal(elements['countdown'].textContent,'1.8 s');
assert.equal(elements['reset-button'].disabled,true);
snapshot.state = 'STOPPED'; snapshot.wait_s = 0;
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['start-button'].disabled,true);
assert.equal(elements['reset-button'].disabled,false);
snapshot.state = 'IDLE'; snapshot.screen_needs_reset = false;
snapshot.settings = {mode:'screen',blocked_edges:[]};
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(elements['start-button'].disabled,false);
assert.equal(elements['blocked-summary'].textContent,'全部道路畅通');
assert.equal(elements['start-marker'].attributes.visibility,'hidden');
modes[0].handlers.click();
context.streamRef.onmessage({data:JSON.stringify(snapshot)});
assert.equal(vm.runInContext('mode',context),'free');
assert.equal(elements.start.disabled,false);
console.log('Browser user flows passed: modes, settings, progress, faults, connection loss.');
