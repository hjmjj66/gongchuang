'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {validOrder,validTask,fmt,phases}=require('../web/app.js');
for(const order of ['123','132','213','231','312','321']) assert.equal(validOrder(order),true);
for(const order of ['','112','2133','123 ',123,null]) assert.equal(validOrder(order),false);
assert.equal(validTask('storage',2,'516','156'),true);
assert.equal(validTask('raw',1,'213',''),true);
assert.equal(validTask('raw',1,'516',''),true);
for(const order of ['112','789','',null]) assert.equal(validTask('raw',1,order,''),false);
for(const [order,layout] of [['516','123'],['511','156'],['789','789'],['516','']]) assert.equal(validTask('storage',2,order,layout),false);
assert.equal(fmt(.01234), '0.012');
assert.equal(fmt(null),'—');
for(const phase of ['WORK_ENTRY','SCAN_RESTORE_WORK','CENTER_RING2','LOWER','STOP_PLACE','STOP_PICK','DEBUG_RESTORE_WORK','WORK_EXIT','STOW','ALIGN_ENTRY','PLACE','PICK','ALIGN_EXIT','DONE','ERROR','STOPPED']) assert.ok(phases[phase]);
assert.ok(phases.RETURN_REFERENCE);
const html=fs.readFileSync(path.join(__dirname,'../web/index.html'),'utf8');
for(const id of ['start','stop','save','order','positions','parameters','preview','overlay','materialModel','materialModelStatus']) assert.ok(html.includes(`id="${id}"`),id);

class Element {
  constructor() { this.children=[];this.handlers={};this.attributes={};this.value='';this.textContent='';this.disabled=false; }
  append(...items) { this.children.push(...items); }
  replaceChildren(...items) { this.children=[...items]; }
  addEventListener(name,fn) { this.handlers[name]=fn; }
  setAttribute(name,value) { this.attributes[name]=value; }
  reportValidity() { return true; }
  set src(value) {this.imageSource=value;queueMicrotask(()=>this.onload?.());}
  get src() {return this.imageSource;}
}

async function browserFlow() {
  const elements=Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],new Element()]));
  elements.order.value='213';
  const requests=[],timers=[];
  let previewObservation=null;
  const context=vm.createContext({
    document:{getElementById:id=>{assert.ok(elements[id],id);return elements[id];},createElement:()=>new Element()},
    EventSource:class {constructor(){context.stream=this;}},
    setInterval:fn=>timers.push(fn),
    fetch:async(route,options)=>{requests.push([route,options ? JSON.parse(options.body) : null]);return {ok:true,json:async()=>({ok:true}),headers:{get:()=>previewObservation ? JSON.stringify(previewObservation) : null},blob:async()=>({})};},
    URL:{createObjectURL:()=>`blob:${requests.length}`,revokeObjectURL:()=>{}},console,Date,Math,JSON,
  });
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8'),context);
  context.stream.onmessage({data:JSON.stringify({state:'IDLE'})});
  const state={state:'IDLE',active:false,can_start:true,healthy:true,graph_ok:true,reason:'',vision_error:'',
    fields:[{path:'vision.target_uv_px',label:'准心',point:true},
            {path:'place_servo.u_pid.kp',label:'kp',minimum:0,positive:true},
            {path:'gantry_motion.max_fb_rpm',label:'预定位伸缩限速 RPM',positive:true},
            {path:'gantry_motion.length_pid.kp',label:'预定位伸缩比例增益',positive:true}],
    parameters:{'vision.target_uv_px':[],'vision.max_frame_age_s':.5,'place_servo.u_pid.kp':.0003,
                'gantry_motion.max_fb_rpm':2,'gantry_motion.length_pid.kp':.2},
    parameter_groups:{gantry_motion:{label:'粗加工 + 暂存共用 · 机构前往标定点 / 粗加工取回',hint:'角度/脉冲到位；在任一区域保存均影响两个区域。'}},
    fixed_parameters:[{path:'execution.control_rate_hz',label:'控制频率 Hz（固定）',value:50}],
    operations:[],placements:{},reference:null,pose:null,confirmed:[],velocity:[0,0,0],
    entry_alignment:'NOT_RUN',final_alignment:'NOT_RUN',diagnostic:{},config_path:'/tmp/debug.yaml',browser_timeout_s:8,
    vision:{sequence:4,age_s:.1,image_size:[1280,720],rings:{2:[640,360]},center_sources:{2:'circle'}}};
  const emit=()=>context.stream.onmessage({data:JSON.stringify(state)});
  state.vision_performance={capture_fps:30,processing_fps:25,preview_fps:12,
    stages_ms:{yolo:30,geometry:15,preview:20,capture_to_result:48}};
  emit();
  assert.match(elements.performance.textContent,/采集 30.0 FPS · 识别 25.0 FPS · 预览生成 12.0 FPS/);
  assert.match(elements.performance.textContent,/识别发布年龄 48 ms/);
  emit();
  assert.equal(elements.start.disabled,false);
  assert.equal(elements.parameterPanel.hidden,false);
  assert.match(elements.parameterScope.textContent,/粗加工区参数 · 作业流程/);
  const fixedGroup=elements.parameters.children.find(group=>group.children[0].textContent==='执行时限与复核');
  assert.equal(fixedGroup.children[1].children[0].textContent,'控制频率 Hz（固定）');
  assert.equal(fixedGroup.children[1].children[1].textContent,'50');
  assert.equal(fixedGroup.children[1].children[1].name,undefined);
  assert.equal(elements.taskExecution.value,'simulated');
  assert.equal(elements.taskExecution.disabled,false);
  assert.match(elements.route.textContent,/环2 \/ 环1 \/ 环3/);
  assert.match(elements.route.textContent,/机构记录2 \/ 机构记录1 \/ 机构记录3/);
  assert.match(elements.route.textContent,/实际收拢/);
  assert.match(elements.route.textContent,/返回入站纠偏后记录的P0/);
  assert.match(elements.motionNotice.textContent,/不额外下降/);
  assert.match(elements.route.textContent,/各工作位旋转伸缩对准/);
  assert.doesNotMatch(elements.route.textContent,/下降/);
  assert.match(elements.positionCol1.textContent,/脉冲/);
  assert.match(elements.positionCol2.textContent,/rad/);
  assert.equal(elements.positions.children.length,3);
  assert.equal(elements.overlay.attributes.viewBox,'0 0 1280 720');
  Object.assign(state,{state:'ERROR',reason:'C板串口未连接',healthy:false,health_error:'C板串口未连接',can_start:false});emit();
  assert.equal(elements.start.disabled,true);
  assert.match(elements.message.textContent,/上次任务中止：C板串口未连接；当前：C板串口未连接/);
  Object.assign(state,{healthy:true,health_error:'',can_start:false});emit();
  assert.equal(elements.start.disabled,true);
  assert.match(elements.message.textContent,/等待机构停止确认/);
  Object.assign(state,{can_start:true});emit();
  assert.equal(elements.start.disabled,false);
  assert.match(elements.message.textContent,/当前已恢复就绪；上次任务中止：C板串口未连接。请手动重新开始/);
  assert.equal(elements.message.className,'');
  assert.equal(requests.length,0);
  Object.assign(state,{state:'IDLE',reason:''});emit();
  elements.order.value='112';elements.order.handlers.input();
  assert.equal(elements.start.disabled,true);
  elements.order.value='213';elements.order.handlers.input();
  await elements.start.handlers.click();
  const start=requests.at(-1)[1];
  assert.equal(start.settings.execution,'simulated');
  assert.equal(elements.taskExecution.disabled,true);
  assert.equal(start.action,'start');assert.equal(start.settings.order,'213');
  state.active=true;state.can_start=false;state.state='PLACE';state.order=[2,1,3];state.item=1;state.target_ring=2;
  state.operations=[{...start,pending:false,ok:true}];state.placements={2:[3802,.25]};state.arm_position={fb_pulses:3802,yaw_rad:.25,pos_flags:1};emit();emit();
  assert.match(elements.armPosition.textContent,/前后 3802 脉冲 \/ 上下 位置无效/);
  state.arm_position={fb_pulses:0,ud_pulses:-4810,yaw_rad:.25,pos_flags:2};emit();
  assert.match(elements.armPosition.textContent,/前后 位置无效 \/ 上下 -4810 脉冲/);
  assert.equal(elements.positions.children[1].children[1].textContent,'3802');
  assert.match(elements.diagnostic.textContent,/机构实际反馈_用于取回闭环/);
  assert.equal(elements.save.disabled,true);
  assert.equal(elements.order.disabled,true);
  assert.equal(elements.phase.textContent,'模拟放料 · 停 1 秒');
  assert.equal(elements.taskExecution.disabled,true);
  await timers[0]();
  assert.equal(requests.at(-1)[0],'/api/heartbeat');
  assert.equal(requests.at(-1)[1].client,start.client);
  await elements.stop.handlers.click();
  const stop=requests.at(-1)[1];assert.equal(stop.action,'stop');
  state.active=false;state.state='STOPPED';state.operations=[{...stop,pending:false,ok:true}];emit();emit();
  assert.equal(elements.save.disabled,false);
  const fieldsets=elements.parameters.children;
  const formInputs=fieldsets.flatMap(group=>group.children).flatMap(label=>label.children || []);
  const point=formInputs.find(input=>input.name==='vision.target_uv_px');
  const kp=formInputs.find(input=>input.name==='place_servo.u_pid.kp');
  const presets=fieldsets.find(group=>group.children[0].textContent==='粗加工 + 暂存共用 · 机构前往标定点 / 粗加工取回');
  assert.ok(presets);
  assert.match(presets.children[1].textContent,/在任一区域保存均影响两个区域/);
  const presetInputs=presets.children.slice(2).map(label=>label.children[1]);
  const presetSpeed=presetInputs.find(input=>input.name==='gantry_motion.max_fb_rpm');
  const presetGain=presetInputs.find(input=>input.name==='gantry_motion.length_pid.kp');
  assert.equal(Number(presetSpeed.value),2);
  assert.equal(Number(presetGain.value),.2);
  assert.equal(presetSpeed.type,'number');
  presetSpeed.value='4';presetSpeed.handlers.input();presetGain.value='0.3';presetGain.handlers.input();
  point.value='[110,120]';point.handlers.input();kp.value='0.0004';kp.handlers.input();
  emit();assert.equal(kp.value,'0.0004');
  await elements.save.handlers.click();
  const save=requests.at(-1)[1];
  assert.equal(save.action,'save');
  assert.deepEqual(save.settings['vision.target_uv_px'],[110,120]);
  assert.equal(save.settings['place_servo.u_pid.kp'],.0004);
  assert.equal(save.settings['gantry_motion.max_fb_rpm'],4);
  assert.equal(save.settings['gantry_motion.length_pid.kp'],.3);
  assert.equal(save.settings['execution.control_rate_hz'],undefined);
  state.operations=[{...save,pending:false,ok:false,error:'参数错误'}];emit();
  assert.equal(elements.message.textContent,'参数错误');
  Object.assign(state,{area:'storage',visit:1,can_start:true,state:'IDLE',order:[],colors:[]});
  emit();emit();
  assert.match(elements.route.textContent,/实际收纳 → 返回入站纠偏后记录的P0→ 停车完成/);
  assert.match(elements.route.textContent,/停1秒→直接衔接下一标定点/);
  assert.doesNotMatch(elements.route.textContent,/下降/);
  elements.area.value='storage';elements.visit.value='2';
  await elements.area.handlers.change();
  const select=requests.at(-1)[1];
  assert.deepEqual(select.settings,{area:'storage',visit:2});
  Object.assign(state,{area:'storage',visit:2,can_start:true,seen_colors:[1,5],colors:[],
    fields:state.fields,
    parameters:{...state.parameters,'vision.target_uv_px':[630,350]},
    operations:[{...select,pending:false,ok:true}],
    vision:{kind:'material',sequence:5,age_s:.1,image_size:[1280,720],rings:{},materials:[[5,[640,360]]],center_sources:{0:'circle'}}});
  emit();emit();
  assert.equal(elements.layoutGroup.hidden,false);
  assert.equal(elements.order.value,'516');
  elements.layout.value='156';elements.layout.handlers.input();
  assert.equal(elements.start.disabled,false);
  assert.match(elements.route.textContent,/颜色5（环2）/);
  assert.equal(elements.overlay.attributes.viewBox,'0 0 1280 720');
  assert.match(elements.cross.attributes.d,/630/);
  assert.equal(elements.positionsTable.hidden,true);
  assert.equal(elements.positions.children.length,0);
  assert.match(elements.motionNotice.textContent,/不额外下降/);
  assert.match(elements.route.textContent,/底盘全程锁定/);
  assert.doesNotMatch(elements.route.textContent,/下降|正vx移动/);
  assert.match(elements.route.textContent,/机构扫齐三色 → 实际回高位/);
  assert.match(elements.route.textContent,/停1秒→直接衔接下一标定点/);
  assert.match(elements.route.textContent,/实际收纳/);
  assert.doesNotMatch(elements.route.textContent,/P0/);
  await elements.start.handlers.click();
  const stackStart=requests.at(-1)[1];
  assert.deepEqual(stackStart.settings,{area:'storage',visit:2,order:'516',layout:'156',execution:'simulated'});
  Object.assign(state,{active:true,state:'SCAN_FIRST_LAYER',order:[2,1,3],colors:[5,1,6],first_layer:{1:1,5:2,6:3},item:1,
    operations:[{...stackStart,pending:false,ok:true}]});
  emit();emit();
  assert.equal(elements.area.disabled,true);
  assert.equal(elements.layout.disabled,true);
  assert.match(elements.phase.textContent,/扫描/);
  assert.match(elements.scan.textContent,/1 \/ 5/);
  state.state='STACK';state.confirmed=[['STACK',1,5,2]];emit();
  assert.match(elements.progress.textContent,/模拟码垛：1 \/ 3/);
  Object.assign(state,{area:'raw',visit:1,active:false,can_start:true,state:'IDLE',colors:[],
    fields:[{path:'vision.target_uv_px',label:'原料准心',point:true},
            {path:'gantry_servo.yaw_pid.kp',label:'kp',minimum:0,positive:true},
            {path:'motion_prediction.enabled',label:'启用预测',boolean:true},
            {path:'motion_prediction.extra_delay_s',label:'额外延迟',minimum:0,positive:false},
            {path:'grasp_gate.small_fb_rpm',label:'低速阈值',minimum:0,positive:true},
            {path:'grasp_gate.qualified_frames',label:'抓取连续达标帧数',minimum:0,positive:true,integer:true}],
    parameters:{'vision.target_uv_px':[110,120],'vision.max_frame_age_s':.5,
                'motion_prediction.enabled':true,'motion_prediction.extra_delay_s':0,
                'gantry_servo.yaw_pid.kp':.0003,'grasp_gate.small_fb_rpm':.01,
                'grasp_gate.qualified_frames':6},confirmed:[]});
  emit();emit();
  assert.equal(elements.order.value,'213');
  assert.equal(elements.visitGroup.hidden,true);
  assert.equal(elements.layoutGroup.hidden,true);
  assert.equal(elements.colorLegend.hidden,false);
  assert.equal(elements.angleRow.hidden,true);
  assert.equal(elements.positions.children.length,1);
  assert.match(elements.route.textContent,/颜色2（旋转伸缩跟随/);
  assert.match(elements.route.textContent,/6帧达标/);
  assert.match(elements.scan.textContent,/连续6帧达标/);
  assert.match(elements.visionTitle.textContent,/原料/);
  assert.match(elements.motionNotice.textContent,/底盘先前进/);
  assert.match(elements.route.textContent,/高位原地等待下一颜色/);
  assert.match(elements.route.textContent,/实际收纳 → 底盘退回 → 完成/);
  assert.doesNotMatch(elements.route.textContent,/P0|圆心定位/);
  assert.match(elements.referenceLabel.textContent,/P0/);
  assert.equal(elements.positionsTable.hidden,true);
  state.reference=[.432,-.7];emit();
  assert.equal(elements.reference.textContent,'0.432 / -0.700 m');
  const rawInputs=elements.parameters.children.flatMap(group=>group.children.slice(1).map(label=>label.children[1]));
  const prediction=rawInputs.find(input=>input.name==='motion_prediction.enabled');
  assert.equal(prediction.type,'checkbox');
  assert.equal(prediction.checked,true);
  prediction.checked=false;prediction.handlers.input();emit();
  assert.equal(prediction.checked,false);
  for(const input of rawInputs) if(input.name && input.type!=='checkbox') input.value=String(input.value);
  await elements.save.handlers.click();
  const rawSave=requests.at(-1)[1];
  assert.equal(rawSave.settings['grasp_gate.qualified_frames'],6);
  assert.equal(rawSave.settings['motion_prediction.enabled'],false);
  assert.equal(rawSave.settings['motion_prediction.extra_delay_s'],0);
  state.parameters['motion_prediction.enabled']=false;
  state.operations=[{...rawSave,pending:false,ok:true}];emit();emit();
  assert.equal(prediction.checked,false);
  await elements.start.handlers.click();
  const rawStart=requests.at(-1)[1];
  assert.deepEqual(rawStart.settings,{area:'raw',visit:1,order:'213',layout:'156',execution:'simulated'});
  Object.assign(state,{active:true,can_start:false,state:'TRACK_REQUIRED',item:1,order:[2,1,3],colors:[2,1,3],
    tracked_color:2,operations:[{...rawStart,pending:false,ok:true}]});
  emit();emit();
  assert.match(elements.target.textContent,/必需颜色 2 · 跟随 2/);
  assert.match(elements.phase.textContent,/旋转伸缩跟随/);
  assert.equal(elements.area.disabled,true);
  state.state='PICK';state.confirmed=[['PICK',1,2,0]];emit();
  assert.match(elements.progress.textContent,/模拟取料：1 \/ 3/);
  state.state='DONE';state.active=false;emit();
  assert.match(elements.message.textContent,/机构已收纳，可进入下一阶段/);
  elements.taskExecution.value='real';elements.taskExecution.handlers.change();emit();
  assert.equal(elements.taskExecution.value,'real');
  assert.match(elements.route.textContent,/真实抓取并等待完成/);
  assert.doesNotMatch(elements.route.textContent,/停1秒/);
  assert.match(elements.executionHint.textContent,/等待下位机完成/);
  assert.match(elements.motionNotice.textContent,/取放程序真实执行/);
  await elements.start.handlers.click();
  const realStart=requests.at(-1)[1];
  assert.equal(realStart.settings.execution,'real');
  Object.assign(state,{active:true,state:'PICK',task_execution:'real',confirmed:[],
    operations:[{...realStart,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.taskExecution.disabled,true);
  assert.equal(elements.phase.textContent,'真实取料 · 等待下位机完成');
  assert.match(elements.progress.textContent,/真实取料：0/);
  // Running-task state takes precedence over the page's local selection.
  elements.taskExecution.value='simulated';emit();
  assert.equal(elements.taskExecution.value,'real');
  for(const [area,visit,phase,label] of [['coarse',1,'PLACE','真实放料'],['coarse',1,'PICK','真实取料'],['storage',1,'PLACE','真实放料'],['storage',2,'STACK','真实码垛']]) {
    Object.assign(state,{area,visit,state:phase,colors:[5,1,6],first_layer:{1:1,5:2,6:3}});emit();
    assert.match(elements.phase.textContent,new RegExp(label));
    assert.match(elements.route.textContent,new RegExp(label+'并等待完成'));
    assert.doesNotMatch(elements.route.textContent,/停1秒/);
    assert.match(elements.progress.textContent,/真实/);
  }
  Object.assign(state,{area:'raw',visit:1,state:'DONE',active:false,confirmed:[['PICK',1,2,0],['PICK',2,1,0],['PICK',3,3,0]]});emit();
  assert.match(elements.message.textContent,/三件真实取料完成/);
  assert.equal(elements.taskExecution.disabled,false);
  elements.controlMode.value='tracking';
  await elements.controlMode.handlers.change();
  const trackingSelect=requests.at(-1)[1];
  assert.deepEqual(trackingSelect.settings,{area:'raw',visit:1,mode:'tracking'});
  Object.assign(state,{mode:'tracking',state:'IDLE',can_start:true,tracking_target:null,confirmed:[],
    operations:[{...trackingSelect,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.taskInputs.hidden,true);
  assert.equal(elements.trackingInputs.hidden,false);
  assert.equal(elements.parameterPanel.hidden,false);
  assert.equal(elements.save.disabled,false);
  assert.match(elements.parameterScope.textContent,/原料区参数 · 追踪模式/);
  assert.equal(elements.trackingTarget.children.length,6);
  assert.match(elements.trackingAction.textContent,/原料区抓料/);
  assert.equal(elements.trackingAction.disabled,true);
  elements.trackingTarget.value='5';elements.trackingTarget.handlers.change();emit();
  assert.equal(elements.trackingTarget.value,'5');
  assert.equal(elements.start.disabled,false);
  assert.match(elements.route.textContent,/持续追踪颜色5/);
  await elements.start.handlers.click();
  const trackingStart=requests.at(-1)[1];
  assert.deepEqual(trackingStart.settings,{area:'raw',visit:1,mode:'tracking',target:5});
  Object.assign(state,{active:true,can_start:false,state:'TRACKING',tracking_target:5,owner:trackingStart.client,
    operations:[{...trackingStart,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.controlMode.disabled,true);
  assert.equal(elements.trackingTarget.disabled,true);
  assert.equal(elements.trackingAction.disabled,false);
  assert.match(elements.target.textContent,/颜色 5/);
  assert.match(elements.motionNotice.textContent,/底盘及升降不动/);
  state.owner='another-page';emit();
  assert.equal(elements.trackingAction.disabled,true);
  state.owner=trackingStart.client;emit();
  await elements.trackingAction.handlers.click();
  const manual=requests.at(-1)[1];assert.equal(manual.action,'tracking_action');
  Object.assign(state,{state:'TRACK_ACTION_STOP',operations:[{...manual,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.trackingAction.disabled,true);
  assert.match(elements.phase.textContent,/等待机构停稳/);
  Object.assign(state,{state:'DONE',active:false,can_start:true,confirmed:[['MANUAL',1,5,5]]});emit();
  assert.match(elements.message.textContent,/真实取放动作已完成/);
  state.fields=[{path:'vision.target_uv_px',label:'圆环准心',point:true},
                {path:'vision.stack_target_uv_px',label:'码垛准心',point:true},
                {path:'gantry_servo.v_pid.kp',label:'伸缩Kp',positive:true}];
  Object.assign(state.parameters,{'vision.stack_target_uv_px':[50,60],'gantry_servo.v_pid.kp':.15});
  for(const [area,visit,label,n] of [['coarse',1,'粗加工区放料',3],['storage',1,'暂存区圆环放置',3],['storage',2,'暂存区码垛',6]]) {
    Object.assign(state,{area,visit});emit();
    assert.match(elements.trackingAction.textContent,new RegExp(label));
    assert.equal(elements.trackingTarget.children.length,n);
  }
  elements.controlMode.value='calibration';await elements.controlMode.handlers.change();
  const calSelect=requests.at(-1)[1];assert.equal(calSelect.settings.mode,'calibration');
  Object.assign(state,{mode:'calibration',calibration_type:'stack',calibration_selection_id:'stack-session',
    state:'IDLE',active:false,calibration_points:{raw:[10.,20.],ring:[30.,40.],stack:[50.,60.]},calibration_sample:null,
    operations:[{...calSelect,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.calibrationInputs.hidden,false);
  assert.equal(elements.motionButtons.hidden,true);
  assert.equal(elements.parameterPanel.hidden,false);
  assert.equal(elements.save.disabled,false);
  assert.match(elements.parameterScope.textContent,/暂存区（第2次）参数 · 标定模式/);
  assert.match(elements.parameterHint.textContent,/无需切换模式/);
  const calibrationInputs=elements.parameters.children.flatMap(group=>group.children).flatMap(row=>row.children || []);
  const calibrationGain=calibrationInputs.find(input=>input.name==='gantry_servo.v_pid.kp');
  const calibrationAim=calibrationInputs.find(input=>input.name==='vision.stack_target_uv_px');
  calibrationGain.value='0.2';calibrationGain.handlers.input();
  assert.equal(elements.calibrationCapture.disabled,true);
  assert.match(elements.motionNotice.textContent,/不自动驱动/);
  assert.equal(elements.calibrationFbControls.hidden,true);
  previewObservation={token:'displayed-token',category:'stack',selection_id:'stack-session',kind:'material',
    sequence:88,age_s:.01,image_size:[1280,720],rings:{},materials:[[5,[412.25,333.5]]]};
  elements.calibrationTarget.value='5';
  await timers[1]();
  assert.equal(elements.calibrationCapture.disabled,false);
  assert.equal(elements.selectedPoint.attributes.cx,412.25);
  assert.match(elements.cross.attributes.d,/M 32 60/);
  assert.match(elements.cross.attributes.d,/M 50 42/);
  state.vision.sequence=999;state.vision.materials=[[5,[900.,600.]]];emit();
  assert.match(elements.calibrationTargetStatus.textContent,/显示帧 88/);
  assert.match(elements.calibrationTargetStatus.textContent,/412.25, 333.50/);
  await elements.calibrationCapture.handlers.click();
  const capture=requests.at(-1)[1];
  assert.deepEqual(capture.settings,{frame_token:'displayed-token',target:5});
  state.calibration_sample={id:'captured-point',client:capture.client,category:'stack',point:[412.25,333.5],saved:false};
  state.operations=[{...capture,pending:false,ok:true}];emit();emit();
  assert.equal(elements.calibrationPoint.value,'412.25, 333.5');
  assert.equal(elements.calibrationSave.disabled,false);
  await elements.calibrationSave.handlers.click();
  const calSave=requests.at(-1)[1];
  assert.equal(calSave.action,'calibration_save');assert.deepEqual(calSave.settings,{sample_id:'captured-point'});
  state.calibration_sample.saved=true;state.calibration_points.stack=[412.25,333.5];
  state.parameters['vision.stack_target_uv_px']=[412.25,333.5];
  state.operations=[{...calSave,pending:false,ok:true}];emit();emit();
  assert.equal(elements.calibrationSave.disabled,true);
  assert.match(elements.message.textContent,/同步到正式比赛配置/);
  assert.equal(calibrationGain.value,'0.2');
  assert.equal(calibrationAim.value,'412.25,333.5');
  for(const input of calibrationInputs) if(input.name) input.value=String(input.value);
  await elements.save.handlers.click();
  const calibrationParameters=requests.at(-1)[1];
  assert.equal(calibrationParameters.action,'save');
  assert.equal(calibrationParameters.settings['gantry_servo.v_pid.kp'],.2);
  assert.deepEqual(calibrationParameters.settings['vision.stack_target_uv_px'],[412.25,333.5]);
  state.parameters['gantry_servo.v_pid.kp']=.2;
  state.operations=[{...calibrationParameters,pending:false,ok:true}];emit();emit();
  assert.equal(elements.controlMode.value,'calibration');
  assert.equal(elements.calibrationType.value,'stack');
  elements.calibrationType.value='ring';await elements.calibrationType.handlers.change();
  const ringSelect=requests.at(-1)[1];assert.deepEqual(ringSelect.settings,{mode:'calibration',calibration_type:'ring'});
  Object.assign(state,{area:'coarse',visit:1,calibration_type:'ring',calibration_selection_id:'ring-session',calibration_sample:null,
    operations:[{...ringSelect,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.calibrationCapture.disabled,true);
  assert.equal(elements.calibrationTarget.children.length,3);
  state.preset_can_start=true;emit();
  assert.equal(elements.calibrationFbControls.hidden,false);
  assert.equal(elements.calibrationTrackFb.disabled,false);
  await elements.calibrationTrackFb.handlers.click();
  const fbStart=requests.at(-1)[1];assert.equal(fbStart.action,'calibration_track_fb');
  Object.assign(state,{active:true,state:'TRACKING',calibration_fb_tracking:true,tracking_target:2,
    preset_can_start:false,operations:[{...fbStart,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.calibrationTrackFb.disabled,true);
  assert.equal(elements.calibrationCapture.disabled,true);
  assert.equal(elements.calibrationType.disabled,true);
  assert.equal(elements.save.disabled,true);
  assert.match(elements.parameterHint.textContent,/请先停止/);
  assert.match(elements.target.textContent,/圆环 2 · 仅伸缩/);
  assert.match(elements.phase.textContent,/仅伸缩纠正竖直误差/);
  assert.match(elements.motionNotice.textContent,/底盘、旋转和升降不动/);
  state.state='TRACK_HOLD';emit();
  assert.match(elements.phase.textContent,/竖直已对准，持续跟随/);
  await elements.calibrationFbStop.handlers.click();
  const fbStop=requests.at(-1)[1];assert.equal(fbStop.action,'stop');
  Object.assign(state,{active:false,state:'STOPPED',preset_can_start:true,
    operations:[{...fbStop,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.calibrationTrackFb.disabled,false);
  assert.equal(elements.save.disabled,false);
  assert.match(elements.phase.textContent,/已停止/);
  state.calibration_fb_tracking=false;emit();
  await timers[1]();  // Response for the previous type must never enable capture.
  assert.equal(elements.calibrationCapture.disabled,true);
  assert.equal(elements.preview.hidden,true);
  elements.calibrationType.value='shared_pose';await elements.calibrationType.handlers.change();
  const poseSelect=requests.at(-1)[1];
  Object.assign(state,{calibration_type:'shared_pose',calibration_selection_id:'pose-session',tracking_target:null,
    preset_can_start:true,preset_reference_ready:false,preset_saved:false,
    gantry_presets:{coarse:{'1':{fb_pulses:12,yaw_rad:-.2},'3':null},storage:{'1':{fb_pulses:12,yaw_rad:-.2},'3':null}},
    operations:[{...poseSelect,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.presetControls.hidden,false);
  assert.equal(elements.aimCalibrationControls.hidden,true);
  assert.equal(elements.calibrationTarget.children.length,2);
  assert.equal(elements.calibrationTarget.value,'3');
  assert.equal(elements.presetAdjust.disabled,false);
  assert.equal(elements.presetTrack.disabled,false);
  assert.equal(elements.presetSave.disabled,false);
  assert.match(elements.presetPositions.textContent,/粗加工 \/ 暂存区（两次）共用/);
  assert.equal((elements.presetPositions.textContent.match(/1号：/g)||[]).length,1);
  assert.equal((elements.presetPositions.textContent.match(/3号：/g)||[]).length,1);
  assert.ok(html.includes('value="shared_pose"'));
  assert.ok(!html.includes('value="coarse_pose"'));
  assert.ok(!html.includes('value="storage_pose"'));
  assert.match(elements.motionNotice.textContent,/可直接开始PID追踪/);
  await elements.presetAdjust.handlers.click();const adjust=requests.at(-1)[1];
  assert.equal(adjust.action,'preset_adjust');
  Object.assign(state,{active:true,state:'CENTER_RING2',owner:adjust.client,preset_can_start:false,
    operations:[{...adjust,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.calibrationType.disabled,true);
  assert.equal(elements.calibrationTarget.disabled,true);
  Object.assign(state,{active:false,state:'DONE',preset_can_start:true,preset_reference_ready:true});emit();
  assert.equal(elements.presetTrack.disabled,false);
  await elements.presetTrack.handlers.click();const poseTrack=requests.at(-1)[1];
  assert.deepEqual(poseTrack.settings,{target:3});
  Object.assign(state,{active:true,state:'TRACKING',tracking_target:3,preset_can_start:false,
    operations:[{...poseTrack,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.presetSave.disabled,false);
  Object.assign(state,{state:'TRACK_HOLD',diagnostic:{qualified_frames:4}});emit();
  assert.equal(elements.presetSave.disabled,false);
  state.owner='different-page';emit();assert.equal(elements.presetSave.disabled,false);
  state.owner=poseTrack.client;emit();
  await elements.presetSave.handlers.click();const poseSave=requests.at(-1)[1];
  assert.equal(poseSave.action,'preset_save');
  assert.deepEqual(poseSave.settings,{target:3});
  Object.assign(state,{active:false,state:'DONE',preset_can_start:true,preset_saved:true,
    operations:[{...poseSave,pending:false,ok:true}]});emit();emit();
  assert.match(elements.message.textContent,/机构初值已保存并同步正式配置/);
  elements.calibrationTarget.value='1';elements.calibrationTarget.handlers.change();emit();
  assert.equal(elements.calibrationTarget.value,'1');
  assert.equal(elements.presetTrack.disabled,false);
  await elements.presetStop.handlers.click();const poseStop=requests.at(-1)[1];
  assert.equal(poseStop.action,'stop');
  state.operations=[{...poseStop,pending:false,ok:true}];emit();emit();
  Object.assign(state,{active:false,state:'STOPPED',tracking_target:1,preset_saved:false,
    preset_reference_ready:false,diagnostic:{qualified_frames:0}});emit();
  assert.equal(elements.presetSave.disabled,false);
  assert.match(elements.presetStatus.textContent,/停止、异常后也可保存/);
  elements.calibrationTarget.value='3';elements.calibrationTarget.handlers.change();
  assert.equal(elements.presetSave.disabled,false);
  elements.calibrationTarget.value='1';elements.calibrationTarget.handlers.change();
  assert.equal(elements.presetSave.disabled,false);
  await elements.presetSave.handlers.click();const stoppedSave=requests.at(-1)[1];
  assert.equal(stoppedSave.action,'preset_save');
  assert.deepEqual(stoppedSave.settings,{target:1});
  Object.assign(state,{active:false,state:'DONE',
    operations:[{...stoppedSave,pending:false,ok:true}]});emit();emit();
  assert.equal(elements.presetSave.disabled,false);
  Object.assign(state,{state:'ERROR',reason:'C板未处于PC控制模式',health_error:'C板未处于PC控制模式',preset_can_start:false,healthy:false});emit();
  assert.equal(elements.presetTrack.disabled,false);
  assert.equal(elements.presetSave.disabled,false);
  assert.match(elements.health.textContent,/C板未处于PC控制模式/);
  Object.assign(state,{state:'IDLE',reason:'',mode:'task',active:false,
    material_model:{available:true,use_jhb_model:false,switching:false,error:''}});emit();
  assert.equal(elements.materialModel.value,'original');
  assert.equal(elements.materialModel.disabled,false);
  elements.materialModel.value='jhb';elements.materialModel.handlers.change();
  const modelSwitch=requests.at(-1)[1];
  assert.equal(modelSwitch.action,'material_model');
  assert.deepEqual(modelSwitch.settings,{use_jhb_model:true,test_combine:false});
  assert.equal(elements.materialModel.disabled,true);
  assert.equal(elements.preview.hidden,true);
  const beforePreview=requests.length;await timers[1]();
  assert.equal(requests.length,beforePreview);
  state.operations=[{...modelSwitch,pending:false,ok:true}];
  state.material_model.switching=true;state.calibration_selection_id='new-model';emit();emit();
  assert.equal(elements.materialModel.disabled,true);
  assert.match(elements.materialModelStatus.textContent,/正在加载/);
  state.material_model.use_jhb_model=true;state.material_model.switching=false;emit();
  assert.equal(elements.materialModel.value,'jhb');
  assert.equal(elements.materialModel.disabled,false);
  state.active=true;emit();assert.equal(elements.materialModel.disabled,true);
  state.active=false;emit();elements.materialModel.value='original';elements.materialModel.handlers.change();
  const originalSwitch=requests.at(-1)[1];assert.deepEqual(originalSwitch.settings,{use_jhb_model:false,test_combine:false});
  state.operations=[{...originalSwitch,pending:false,ok:true}];
  state.material_model.error='切换失败，继续使用原模式：加载失败';emit();emit();
  assert.equal(elements.materialModel.value,'jhb');
  assert.equal(elements.materialModel.disabled,false);
  assert.match(elements.materialModelStatus.textContent,/切换失败/);
  state.material_model.error='';
  state.material_model.use_jhb_model=false;state.material_model.test_combine=true;emit();
  assert.equal(elements.materialModel.value,'test_combine');
  assert.match(elements.materialModelStatus.textContent,/原 YOLO 颜色 \+ JHB 中心/);
  state.vision_performance.stages_ms={color_yolo:20,jhb_yolo:40,matching:.2,yolo:60};emit();
  assert.match(elements.performance.textContent,/原 YOLO 20 ms \/ JHB 40 ms \/ 匹配 0.2 ms/);
  elements.materialModel.value='test_combine';elements.materialModel.handlers.change();
  assert.deepEqual(requests.at(-1)[1].settings,{use_jhb_model:false,test_combine:true});
  state.mode='calibration';emit();
  context.stream.onerror();
  assert.equal(elements.materialModel.disabled,true);
  assert.equal(elements.presetTrack.disabled,true);
  assert.equal(elements.presetSave.disabled,true);
  assert.equal(elements.start.disabled,true);
  assert.equal(elements.trackingAction.disabled,true);
  console.log('Web render, task/tracking modes, targets, real-action controls, heartbeat and parameter editing passed');
}
browserFlow().catch(error=>{console.error(error);process.exitCode=1;});
