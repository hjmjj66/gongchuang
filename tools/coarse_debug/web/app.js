'use strict';

const phases = {IDLE:'待机',ALIGN_ENTRY:'开始角度纠偏',CAPTURE_REFERENCE:'记录 P0',FIND_RING:'寻找圆环',
  ALIGN_RING:'圆环视觉对准',VERIFY_PLACE:'停车复核圆心',PLACE:'模拟放料 · 停 1 秒',RECORD_PLACE:'保存放料位置',
  RETURN_ITEM:'返回保存位置',PICK:'模拟取料 · 停 1 秒',RETURN_REFERENCE:'返回 P0',ALIGN_EXIT:'结束角度纠偏',
  DONE:'完成',ERROR:'异常停止',STOPPED:'已停止'};
Object.assign(phases,{CAPTURE_STACK_REFERENCE:'记录平移前 P0',SCAN_FIRST_LAYER:'扫描第一层三种颜色',
  PREPOSITION:'机构前往标定初值',STOP_PREPOSITION:'初值到位 · 等待停稳后PID精修',
  TRACK_WAIT:'追踪 · 停轴等待目标',TRACKING:'追踪 · 伸缩旋转对准',TRACK_HOLD:'追踪 · 已对准，持续跟随',
  TRACK_ACTION_STOP:'已停止追踪 · 等待机构停稳',TRACK_ACTION:'实际执行所选区域取放',
  WORK_ENTRY:'实际执行高位工作位',CENTER_RING2:'高位 · 底盘对准2号环',LOWER:'实际定时下降',
  STOP_PLACE:'机构停止 · 记录实际长度与角度',STOP_PICK:'机构停止 · 复核返回位置',
  DEBUG_RESTORE_WORK:'模拟取料后 · 实际回高位',WORK_EXIT:'末次纠偏前 · 实际回高位',STOW:'实际收拢机构',RETURN_REFERENCE:'收纳后 · 返回P0',
  SCAN_RESTORE_WORK:'三色已齐 · 实际回高位工作位',FIND_MATERIAL:'寻找同色物料',ALIGN_MATERIAL:'物料中心视觉对准',VERIFY_STACK:'停车复核物料中心',STACK:'模拟码垛 · 停 1 秒',
  ENTER_WORK_POSE:'实际执行工作位',RESTORE_WORK:'模拟抓取后实际回工作位',
  STOW_ARM:'实际收纳机构',APPROACH:'底盘沿工作方向前进',STOP_EXIT:'收纳后停车 · 交接导航',
  WAIT_REQUIRED:'停轴等待所需颜色',TRACK_REQUIRED:'旋转伸缩跟随 · 中心及两轴小输出判定'});
const groups = {base_servo:'底盘初始对准',gantry_motion:'共用机构位置控制',exit_motion:'收纳后底盘移动',vision:'图像与准心',angle_alignment:'斜率角度纠偏',place_servo:'圆环视觉伺服',
  ring_search:'寻找与扫描',execution:'执行时限与复核',stack_servo:'物料码垛视觉伺服',
  arm:'实际机构动作号（须与电控核对）',base_motion:'原料底盘前进 / 退回',
  gantry_servo:'旋转/伸缩图像伺服',
  
  grasp_gate:'原料连续达标门槛',motion_prediction:'原料卡尔曼与短时延迟补偿'};
const alignment = v => ({NOT_RUN:'未执行',ALIGNED:'已达标',TIMEOUT_SKIPPED:'超时跳过'}[v] || v);
function validOrder(value) { return typeof value === 'string' && value.length === 3 && [...value].sort().join('') === '123'; }
function validColors(value) { return typeof value === 'string' && /^[1-6]{3}$/.test(value) && new Set(value).size === 3; }
function validTask(area,visit,order,layout) {
  if(area==='raw') return validColors(order);
  return area==='storage' && visit===2 ? validColors(order) && validColors(layout) && [...order].sort().join('')===[...layout].sort().join('') : validOrder(order);
}
function fmt(value, digits=3) { return Number.isFinite(value) ? value.toFixed(digits) : '—'; }
if (typeof module !== 'undefined') module.exports = {validOrder, validColors, validTask, fmt, phases};

if (typeof document !== 'undefined') {
  const $ = id => document.getElementById(id);
  const client = globalThis.crypto?.randomUUID?.() || `page-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  let state = null, connected = false, dirty = false, formBuilt = false, pending = null, previewBusy = false, imageURL = null;
  const inputs = new Map();
  let mode = '';
  let displayedFrame = null, displayedAt = 0, calibrationSelection = '';
  const calibrationLabels={raw:'原料区取料',ring:'粗加工 / 暂存第一次圆环',stack:'暂存第二次物料'};
  const isPreset = () => state?.calibration_type==='shared_pose';
  const drafts = {raw1:'213',coarse1:'213',coarse2:'213',storage1:'213',storage2:'516'};
  const text = (id,value) => { $(id).textContent = value; };
  function message(value,error=false) { text('message',value); $('message').className = error ? 'error' : ''; }
  function drawAim(v,point) {
    const [w,h]=v.image_size,[u,y]=point?.length ? point : [(w-1)/2,(h-1)/2];
    $('overlay').hidden=false;$('overlay').setAttribute('viewBox',`0 0 ${w} ${h}`);
    $('cross').setAttribute('d',`M ${u-18} ${y} h 36 M ${u} ${y-18} v 36`);
  }
  function calibrationPoints(frame) {
    const target=state?.active && state.calibration_fb_tracking ? 2 : Number($('calibrationTarget').value);
    return !frame ? [] : frame.kind==='ring' ? (frame.rings[target] ? [frame.rings[target]] : []) : frame.materials.filter(([color])=>color===target).map(([,point])=>point);
  }
  function updateCalibration() {
    if(state?.mode!=='calibration') { $('calibrationCapture').disabled=true;$('calibrationSave').disabled=true;return; }
    const category=state.calibration_type,sample=state.calibration_sample?.client===client ? state.calibration_sample : null;
    const preset=isPreset();
    $('calibrationFbControls').hidden=category!=='ring' && !preset;
    $('calibrationTrackFb').disabled=!connected || !!pending || !state.preset_can_start;
    $('presetControls').hidden=!preset;$('aimCalibrationControls').hidden=preset;
    $('presetAdjust').disabled=!connected || !!pending || !state.preset_can_start;
    $('presetTrack').disabled=!connected || !!pending;
    $('presetSave').disabled=!connected || !!pending;
    text('presetStatus',state.preset_saved ? `已保存${state.tracking_target}号圆环点击保存时的实际位置，并同步到正式流程。` : `当前机构：伸出 ${state.arm_position?.pos_flags & 1 ? fmt(state.arm_position.fb_pulses,0) + " 脉冲" : "位置无效"} / 转角 ${fmt(state.arm_position?.yaw_rad,5)} rad。可直接开始追踪或保存所选圆环的当前位置，无需先调整或等待4帧达标；停止、异常后也可保存。`);
    text('presetPositions','粗加工 / 暂存区（两次）共用\n'+[1,3].map(n=>{const p=state.gantry_presets?.coarse?.[n];return `  ${n}号：${p ? `${fmt(p.fb_pulses,0)} 脉冲 / ${fmt(p.yaw_rad,5)} rad` : '未标定'}`;}).join('\n'));
    const frame=displayedFrame?.selection_id===state.calibration_selection_id && displayedFrame.category===category ? displayedFrame : null;
    const fresh=frame && frame.age_s+(Date.now()-displayedAt)/1000<=state.parameters['vision.max_frame_age_s'];
    const points=calibrationPoints(frame);
    $('calibrationCapture').disabled=preset || state.active || !connected || !!pending || !fresh || points.length!==1;
    $('calibrationSave').disabled=preset || state.active || !connected || !!pending || !sample || sample.saved;
    $('calibrationPoint').value=sample ? sample.point.join(', ') : '';
    text('calibrationSaved',Object.entries(calibrationLabels).map(([name,label])=>`${label}：${state.calibration_points?.[name]?.length ? state.calibration_points[name].map(n=>fmt(n,2)).join(', ') : '图像中心（尚未采点）'}`).join('；'));
    text('calibrationTargetStatus',!frame ? '等待当前标定类型的画面…' : !fresh ? '显示画面已过期，等待实时画面…' : points.length===1 ? `显示帧 ${frame.sequence} · 所选目标 (${points[0].map(n=>fmt(n,2)).join(', ')})` : points.length>1 ? '存在多个同色目标，请调整到仅一个目标可见后采点。' : '所选目标不可见，请调整位置或切换目标。');
    $('selectedPoint').hidden=!frame || points.length!==1;
    if(frame) {
      drawAim(frame,preset || state.calibration_fb_tracking ? state.parameters['vision.target_uv_px'] : sample?.point || state.calibration_points?.[category]);
      $('rings').replaceChildren();
      const rows=frame.kind==='ring' ? Object.entries(frame.rings) : frame.materials;
      for(const [id,point] of rows) {const chip=document.createElement('span');chip.className='chip';chip.textContent=`${frame.kind==='ring'?'环':'颜色'}${id}: (${point.map(n=>fmt(n,2)).join(', ')})`;$('rings').append(chip);}
      if(points.length===1) {$('selectedPoint').setAttribute('cx',points[0][0]);$('selectedPoint').setAttribute('cy',points[0][1]);}
      text('frame',`显示帧 ${frame.sequence}${fresh?'':' · 已过期'}`);
    } else $('overlay').hidden=true;
  }
  function updateRoute() {
    $('materialModel').disabled=!connected || !!pending || !!state?.active || !state?.material_model?.available || state.material_model.switching;
    const area=state?.area || 'coarse',visit=state?.visit || 1,stack=area==='storage' && visit===2;
    updateCalibration();
    if(state?.mode==='calibration') {
      text('route',isPreset() ? '选择1/3号 → 可直接开始纯PID追踪 → 随时保存当前位置并同步正式流程；调整底盘为可选操作' : '选择标定类型和目标 → 手工调整到合适位置 → 取当前目标点 → 保存并同步正式流程');
      if(state.calibration_fb_tracking) text('route','从当前位置 → 只用伸缩持续纠正2号环竖直误差 → 手动停止');
      $('start').disabled=true;return;
    }
    if(state?.mode==='tracking') {
      const material=area==='raw' || stack,target=Number($('trackingTarget').value);
      text('route',`底盘与升降不动 → 持续追踪${material?'颜色':'圆环'}${target} → 对准后保持跟随；丢失停轴等待`);
      $('start').disabled=!connected || !state.can_start || !(target>=1 && target<=(material?6:3)) || !!pending;
      $('trackingAction').disabled=!connected || !state.active || state.owner!==client || !!pending || ['TRACK_ACTION_STOP','TRACK_ACTION'].includes(state.state);
      return;
    }
    const real=(state?.active ? state.task_execution : $('taskExecution').value)==='real';
    const place=real ? '真实放料并等待完成' : '停1秒→直接衔接下一标定点';
    const pick=real ? '真实取料并等待完成' : '停1秒→回高位';
    const rawPick=real ? '真实抓取并等待完成' : '停1秒→实际回工作位';
    const stackPlace=real ? '真实码垛并等待完成' : '停1秒→直接衔接下一标定点';
    const order = state?.active ? (stack ? state.colors : state.order).join('') : $('order').value;
    const layout=state?.active && stack ? Object.entries(state.first_layer).sort((a,b)=>a[1]-b[1]).map(([c])=>c).join('') : $('layout').value;
    let route='请输入 1、2、3 各一次，例如 213';
    if(area==='raw') route=validColors(order) ? `实际工作位 → 底盘前进（距离可调）→ 原地等待所需颜色 → ${[...order].map((c,i)=>`颜色${c}（旋转伸缩跟随→${state.parameters['grasp_gate.qualified_frames']}帧达标→${rawPick}）${i<2?'→高位原地等待下一颜色':''}`).join(' → ')} → 实际收纳 → 底盘退回 → 完成` : '请输入1～6中三个不同颜色编号，例如213';
    else if(stack) route=validTask(area,visit,order,layout) ? `底盘全程锁定 → 实际高位 → 机构扫齐三色 → 实际回高位 → ${[...order].map(c=>`颜色${c}（环${layout.indexOf(c)+1}）高位旋转伸缩对准→4帧达标→${stackPlace}`).join(' → ')} → 实际收纳 → 完成` : '顺序和布局须为相同的三种不同颜色（1～6）';
    else if(validOrder(order)) route=area==='coarse' ? `实际高位 → 水平纠偏（最多3秒） → 记录P0 → 底盘对准环2 → ${[...order].map(n=>`环${n}`).join(' / ')}（各工作位旋转伸缩对准→记录→${place}） → ${[...order].map(n=>`机构记录${n}`).join(' / ')}（各反馈返回→${pick}） → 高位纠偏 → 实际收拢 → 返回入站纠偏后记录的P0→ 停车完成` : `实际高位 → 水平纠偏（最多3秒） → 记录P0 → 底盘对准环2后锁定 → ${[...order].map(n=>`环${n}`).join(' / ')}（各高位旋转伸缩对准→4帧达标→${place}） → 实际收纳 → 返回入站纠偏后记录的P0→ 停车完成`;
    text('route',route);
    $('start').disabled = !connected || !state?.can_start || !validTask(area,visit,$('order').value,$('layout').value) || !!pending;
  }
  function buildForm(s) {
    $('parameters').replaceChildren();inputs.clear();
    for (const group of Object.keys(groups)) {
      const fieldset = document.createElement('fieldset'), legend = document.createElement('legend');
      const fixed=(s.fixed_parameters || []).filter(f=>f.path.startsWith(group+'.'));
      if(!s.fields.some(f=>f.path.startsWith(group+'.')) && !fixed.length) continue;
      const details=s.parameter_groups?.[group];
      legend.textContent = details?.label || groups[group]; fieldset.append(legend);
      if(details?.hint) {
        const hint=document.createElement('p');hint.className='muted';hint.textContent=details.hint;fieldset.append(hint);
      }
      for (const f of s.fields.filter(f=>f.path.startsWith(group+'.'))) {
        const label = document.createElement('label'), title = document.createElement('span'), input = document.createElement('input');
        label.className = 'field'; title.textContent = f.label; input.name = f.path;
        input.title=f.path; title.title=f.path;
        input.type = f.boolean ? 'checkbox' : f.point ? 'text' : 'number';
        if (f.point) input.placeholder = '留空 / 640,360';
        else if (!f.boolean) { input.step = f.integer ? '1' : 'any'; input.min = f.positive ? (f.integer ? '1' : '0.000000001') : '0'; if(f.maximum != null) input.max = f.maximum; }
        input.addEventListener('input',()=>{dirty=true;});
        inputs.set(f.path,input); label.append(title,input); fieldset.append(label);
      }
      for(const f of fixed) {
        const row=document.createElement('p'),title=document.createElement('span'),value=document.createElement('span');
        row.className='field muted';title.textContent=f.label;title.title=f.path;value.textContent=String(f.value);
        row.append(title,value);fieldset.append(row);
      }
      $('parameters').append(fieldset);
    }
    formBuilt = true;
  }
  function render(s) {
    if(!s.fields) return;
    state = s;
    const model=s.material_model;
    $('materialModel').value=model?.test_combine ? 'test_combine' : model?.use_jhb_model ? 'jhb' : 'original';
    text('materialModelStatus',model?.switching ? '正在加载物料模型…' : model?.error || (model?.available ? `当前：${model.test_combine ? 'test_combine · 原 YOLO 颜色 + JHB 中心' : model.use_jhb_model ? 'JHB 模型 · Lab 六色' : '原有识别'}。用于原料区和暂存第二次物料识别；运行中请先停止再切换。` : '等待物料模型就绪…'));
    s.area=s.area || 'coarse';s.visit=s.visit || 1;
    const nextMode=s.area+s.visit,stack=s.area==='storage' && s.visit===2,raw=s.area==='raw',material=stack || raw,tracking=s.mode==='tracking',calibrating=s.mode==='calibration';
    if(nextMode!==mode) {
      if(mode) drafts[mode]=$('order').value;
      $('order').value=drafts[nextMode];mode=nextMode;dirty=false;formBuilt=false;
      $('area').value=s.area;$('visit').value=String(s.visit);
    }
    $('visitGroup').hidden=s.area!=='storage';$('layoutGroup').hidden=!stack;$('colorLegend').hidden=!material;
    $('controlMode').value=s.mode || 'task';$('controlMode').disabled=s.active || !!pending;
    $('taskInputs').hidden=tracking || calibrating;$('trackingInputs').hidden=!tracking;$('trackingActionGroup').hidden=!tracking;
    $('calibrationInputs').hidden=!calibrating;$('areaSelection').hidden=calibrating;$('motionButtons').hidden=calibrating;$('parameterPanel').hidden=false;
    if(calibrationSelection!==s.calibration_selection_id) {
      calibrationSelection=s.calibration_selection_id;displayedFrame=null;$('preview').hidden=true;
    }
    if(calibrating) {
      $('calibrationType').value=s.calibration_type;$('calibrationType').disabled=s.active || !!pending;
      $('calibrationTarget').disabled=s.active || !!pending;
    }
    if(s.active && !tracking && !calibrating) $('taskExecution').value=s.task_execution || 'simulated';
    if(!$('taskExecution').value) $('taskExecution').value='simulated';
    const real=(s.active ? s.task_execution : $('taskExecution').value)==='real';
    const completedReal=s.task_execution==='real',opLabel=completedReal?'真实':'模拟';
    text('executionHint',real ? '真实执行取放完整程序（含程序自身升降与回工作位），等待下位机完成后继续；运行中不能切换。' : '仅取放动作以1秒等待替代；底盘、伸缩、旋转、工作位和收纳仍真实执行。');
    const choices=material ? ['红色','黄色','蓝色','绿色','黑色','浅蓝色'] : ['圆环1','圆环2','圆环3'];
    if($('trackingTarget').children.length!==choices.length) {
      $('trackingTarget').replaceChildren();
      choices.forEach((label,i)=>{const option=document.createElement('option');option.value=String(i+1);option.textContent=material ? `${i+1} · ${label}` : label;$('trackingTarget').append(option);});
      $('trackingTarget').value='1';
    }
    const calibrationChoices=calibrating && isPreset() ? ['圆环1','圆环3'] : choices;
    if($('calibrationTarget').children.length!==calibrationChoices.length) {
      $('calibrationTarget').replaceChildren();
      calibrationChoices.forEach((label,i)=>{const option=document.createElement('option');option.value=String(calibrating && isPreset() ? [1,3][i] : i+1);option.textContent=material ? `${i+1} · ${label}` : label;$('calibrationTarget').append(option);});
      $('calibrationTarget').value=calibrating && isPreset() ? '3' : '1';
    }
    if(calibrating && isPreset()) {
      if(s.active && !s.calibration_fb_tracking && s.tracking_target!=null) $('calibrationTarget').value=String(s.tracking_target);
    }
    if(tracking && s.active && s.tracking_target!=null) $('trackingTarget').value=String(s.tracking_target);
    $('trackingTarget').disabled=s.active || !!pending;
    const actionLabel=raw ? '原料区抓料' : s.area==='coarse' ? '粗加工区放料' : stack ? '暂存区码垛' : '暂存区圆环放置';
    text('trackingAction',`执行${actionLabel}`);
    text('trackingActionHint',`真实执行${actionLabel}完整程序，可能包含升降及回工作位。点击后结束追踪，动作完成后等待重新开始。`);
    const areaLabel={raw:'原料区',coarse:'粗加工区',storage:'暂存区'}[s.area];
    const context=calibrating ? `标定模式 · ${isPreset() ? '粗加工 / 暂存共用机构初值' : calibrationLabels[s.calibration_type]}` : tracking ? '追踪模式' : '作业流程';
    text('parameterScope',`当前编辑：${areaLabel}${s.area==='storage' ? `（第${s.visit}次）` : ''}参数 · ${context}。参数按区域 / 共用组保存，三个模式沿用同一套。`);
    const usage=calibrating ? (isPreset() ? '标定“调整”使用水平纠偏、底盘对环；纯PID追踪使用机构视觉组；前往标定点参数用于后续作业预定位。' : s.calibration_type==='ring' ? '“仅伸缩跟随2号环”使用共用机构视觉组的竖直PID、伸缩限速和对准门槛；普通准心采点不启动伺服。' : '准心采点不启动伺服；运动参数用于后续作业或追踪。')+'下方保存运动参数；上方按钮分别保存准心或机构初值。' : tracking ? '连续追踪使用机构视觉组的PID、限速及对准门槛；原料另使用抓取门槛和预测组。底盘及预定位组用于作业流程。' : '各组说明标明控制对象、单位和适用阶段。';
    text('parameterHint',`三种模式均可在停止后编辑保存，无需切换模式。${s.active ? '当前正在运行，请先停止再编辑保存。' : ''}保存同步正式比赛参数，网页下次开始、正式节点重启后生效；机构位置控制在三区域同步，其余共用组在粗加工与暂存同步。固定规则仅显示，不参与保存。${usage}`);
    $('angleRow').hidden=raw;
    text('motionNotice',raw ? '底盘先前进，三件完成后退回；实际工作位、旋转、伸缩、收纳和底盘退回。模拟方式仅抓取等待1秒，随后实际回工作位。参数须现场验证。' : s.area==='coarse' ? '真实底盘与龙门架运动：工作位旋转、伸缩和收拢；不额外下降，仅取放以停车1秒替代。参数为待标定低速初值。' : '真实底盘与龙门架运动：高位旋转伸缩对准，不额外下降；仅放置/码垛停1秒模拟，完成后直接衔接下一标定点，最终实际收纳。首次收纳后返回P0并停车，第二次底盘全程锁定。参数须现场标定。');
    if(real && !tracking && !calibrating) text('motionNotice',raw ? '底盘先前进，三件完成后退回；取放程序真实执行，包含程序自身升降与回工作位。前三件按序抓取，每件抓完回高位原地等待，第三件后直接收纳。参数须现场验证。' : '真实底盘与龙门架运动：按当前区域完整流程运行，取放程序真实执行，包含程序自身升降与回工作位；等待下位机完成后才继续。参数须现场标定。');
    if(tracking) text('motionNotice','追踪模式：仅自动控制龙门架伸缩和旋转，底盘及升降不动。沿用当前区域准心、PID和限速；手动取放按钮执行真实完整程序。');
    if(calibrating) text('motionNotice',isPreset() ? '机构初值标定：可直接开始PID追踪，随时保存当前实际伸出长度和转角。调整按钮真实执行WORK、角度纠偏及底盘对准2号环，按需使用。' : '标定模式：实时识别、手工对准、一键采集目标中心。网页不自动驱动底盘或龙门架；保存同时同步到正式比赛流程。');
    text('orderLabel',raw ? '取料颜色顺序（1～6 中三个不同编号）' : stack ? '码垛颜色顺序（1～6 中三个不同编号）' : '圆环顺序（1、2、3 各一次）');
    $('order').pattern=material ? '[1-6]{3}' : '[123]{3}';
    text('visionTitle',raw ? '原料物料识别与准心' : stack ? '物料识别与码垛准心' : '圆环识别与准心');
    if(!formBuilt) { buildForm(s); message('控制端已连接，等待底盘与相机就绪。'); }
    if(!dirty) for(const [path,input] of inputs) { const v=s.parameters[path]; if(input.type==='checkbox') input.checked=v; else input.value=Array.isArray(v) ? v.join(',') : v; }
    for(const input of inputs.values()) input.disabled = s.active || !!pending;
    $('save').disabled = !connected || s.active || !!pending;
    $('order').disabled = s.active || !!pending;
    for(const id of ['area','visit','layout','taskExecution']) $(id).disabled=s.active || !!pending;
    text('phase',(completedReal ? {PLACE:'真实放料 · 等待下位机完成',PICK:'真实取料 · 等待下位机完成',STACK:'真实码垛 · 等待下位机完成'}[s.state] : null) || phases[s.state] || s.state);
    text('health',s.healthy ? '就绪' : s.health_error || '等待控制端反馈');
    const target = ['ALIGN_ENTRY','ALIGN_EXIT'].includes(s.state) ? '至少两个圆环拟合直线' :
      ['CAPTURE_REFERENCE','CAPTURE_STACK_REFERENCE','RETURN_REFERENCE'].includes(s.state) ? 'P0' :
      s.state==='SCAN_FIRST_LAYER' ? '先扫齐三色' :
      raw ? `第 ${s.item} 项 · 必需颜色 ${s.colors[Math.min(s.item-1,2)]} · 跟随 ${s.tracked_color ?? '尚未锁定'}` :
      stack ? `第 ${s.item} 项 · 颜色 ${s.colors[Math.min(s.item-1,2)]}` :
      s.state==='CENTER_RING2' ? '2号环 · 底盘高位校准' :
      ['RETURN_ITEM','STOP_PICK','PICK'].includes(s.state) ? `第 ${s.item} 项 · 环${s.target_ring}记录的长度/角度` : `第 ${s.item} 项 · 圆环 ${s.target_ring}`;
    text('target',s.active ? target : '—');
    if(tracking) text('target',s.tracking_target==null ? '—' : `${material?'颜色':'圆环'} ${s.tracking_target}`);
    text('angles',`${alignment(s.entry_alignment)} / ${alignment(s.final_alignment)}`);
    text('pose',s.pose ? `${fmt(s.pose.x)} / ${fmt(s.pose.y)} / ${fmt(s.pose.yaw)} rad` : '—');
    const armPosition=s.arm_position;
    text('armPosition',armPosition ? `前后 ${armPosition.pos_flags & 1 ? fmt(armPosition.fb_pulses,0)+' 脉冲' : '位置无效'} / 上下 ${armPosition.pos_flags & 2 ? fmt(armPosition.ud_pulses,0)+' 脉冲' : '位置无效'}` : '等待反馈');
    text('referenceLabel','P0 底盘 x / y');
    text('reference',s.reference ? `${fmt(s.reference[0])} / ${fmt(s.reference[1])} m` : '—');
    text('positionsTitle',s.area==='coarse' ? '放置时实际机构位置 · 取回目标' : '保存位置 · 固定里程坐标系 / m');
    text('positionCol1',s.area==='coarse' ? '前后位置 脉冲' : 'x / m');
    text('positionCol2',s.area==='coarse' ? '机构角度 rad' : 'y / m');
    $('positionsTitle').hidden=s.area!=='coarse';$('positionsTable').hidden=s.area!=='coarse';
    if(tracking || calibrating) {$('positionsTitle').hidden=true;$('positionsTable').hidden=true;$('angleRow').hidden=true;}
    $('positions').replaceChildren();
    for(const n of (s.area==='storage' ? [] : s.area==='coarse' ? [1,2,3] : [0])) {
      const p=n===0 ? s.reference : s.placements[n], tr=document.createElement('tr');
      for(const value of [s.area==='coarse' ? `环${n}` : `P${n}`,fmt(p?.[0],s.area==='coarse' ? 0 : 3),fmt(p?.[1])]) { const td=document.createElement('td');td.textContent=value;tr.append(td); }
      $('positions').append(tr);
    }
    const count = op => s.confirmed.filter(row=>row[0]===op).length;
    text('progress',raw ? `${opLabel}取料：${count('PICK')} / 3` : s.area==='coarse' ? `${opLabel}放料：${count('PLACE')} / 3　${opLabel}取料：${count('PICK')} / 3` : stack ? `${opLabel}码垛：${count('STACK')} / 3` : `${opLabel}放料：${count('PLACE')} / 3`);
    text('scan',raw ? `底盘先前进，三件完成后退回；记录底盘进区位置，每件抓完回高位原地等待，下一颜色出现立即追踪。中心误差和两轴输出连续${s.parameters['grasp_gate.qualified_frames']}帧达标后${real?'真实抓取并等待完成':'停1秒，再实际回工作位'}。已完成颜色忽略。` : stack ? `扫描已确认颜色：${s.seen_colors?.join(' / ') || '尚未扫描'}；齐三色并实际回高位后才开始逐件高位对准和${real?'真实':'模拟'}码垛。` : s.area==='coarse' ? '入站纠偏后记录P0；取放期间底盘不平移，收拢后返回P0。放置按图像误差对准；取回只用实际长度和机构角度反馈。' : `底盘${s.base_locked ? '已锁定' : s.state==='RETURN_REFERENCE' ? '正在返回P0' : '正在进行入站校准'}；入站纠偏后记录P0。放置逐件高位对准，不额外下降；收纳后返回P0并停车，无末次纠偏。`);
    if(tracking) {
      text('scan','提前摆好高度；不自动扫描、回位或切换目标。点击取放按钮前请确认现场位置。');
      text('progress',s.confirmed.length ? '真实取放已完成' : !s.active ? '等待开始追踪' : ['TRACK_ACTION_STOP','TRACK_ACTION'].includes(s.state) ? '已结束追踪，正在执行真实取放' : '持续追踪，等待手动执行取放');
    }
    if(calibrating && !isPreset()) {text('phase','手工标定 · 只预览与采点');text('health',s.vision && !s.vision_error ? '相机预览 · 无需PC控制' : '等待相机画面');text('target',calibrationLabels[s.calibration_type]);text('scan','准心以原图像素保存；圆环标定同时应用到粗加工与暂存第一次。');text('progress','暂存第二次的物料标定点独立保存。');}
    if(calibrating && isPreset()) {text('target',s.tracking_target ? `圆环 ${s.tracking_target}` : '2号环 · 底盘基准调整');text('scan','1号、3号各保存一组角度与伸缩位置，粗加工和暂存两次共用，并同步两区正式配置。标定追踪使用粗加工 / 暂存共用机构视觉参数。');text('progress',s.preset_saved ? '机构初值已保存并同步正式配置' : '可直接追踪，随时保存所选圆环的当前位置');}
    if(calibrating && s.calibration_fb_tracking) {
      text('phase',s.state==='TRACKING' ? '2号环 · 仅伸缩纠正竖直误差' : s.state==='TRACK_HOLD' ? '2号环 · 竖直已对准，持续跟随' : phases[s.state] || s.state);
      text('health',s.healthy ? '就绪' : '等待有效反馈 / PC控制');text('target','圆环 2 · 仅伸缩');
      text('motionNotice','2号环伸缩调试：只用竖直图像PID持续跟随，底盘、旋转和升降不动；手动停止。');
      text('scan','沿用两区共用伸缩PID与限速，以及当前作业已保存的准心；目标丢失时停轴等待。');text('progress','独立debug，不保存准心或机构初值。');
    }
    text('diagnostic',JSON.stringify({观测:s.diagnostic,底盘_vx_vy_wz:s.velocity,旋转rad_s_伸缩RPM:s.arm_velocity,机构动作:s.arm,...(s.area==='storage' ? {底盘已锁定:s.base_locked,当前机构请求:s.arm_request,机构实际反馈_仅诊断:s.arm_position} : {}),...(s.area==='coarse' ? {机构实际反馈_用于取回闭环:s.arm_position} : {}),...(raw ? {底盘已锁定:s.base_locked,机构实际位置:s.arm_position} : {})},null,2));
    text('configPath',`共用运动配置：${s.motion_config_path || '随当前配置保存'}；区域配置：${s.config_path} · ${{raw:'原料区',storage:'暂存区',coarse:'粗加工区'}[s.area]}参数；正式同步：${(s.production_paths || []).join("、") || "—"}`);
    text('browserTimeout',String(s.browser_timeout_s));
    const v=s.vision, fresh=v && v.age_s<=s.parameters['vision.max_frame_age_s'];
    text('frame',v ? `帧 ${v.sequence} · ${fmt(v.age_s*1000,0)} ms${fresh?'':' · 已过期'}` : '等待图像');
    const perf=s.vision_performance,stages=perf?.stages_ms;
    const combineTiming=stages?.color_yolo == null ? '' : ` · 原 YOLO ${fmt(stages.color_yolo,0)} ms / JHB ${fmt(stages.jhb_yolo,0)} ms / 匹配 ${fmt(stages.matching,1)} ms`;
    text('performance',perf ? `采集 ${fmt(perf.capture_fps,1)} FPS · 识别 ${fmt(perf.processing_fps,1)} FPS · 预览生成 ${fmt(perf.preview_fps,1)} FPS · 模型 ${fmt(stages?.yolo,0)} ms / 几何 ${fmt(stages?.geometry,0)} ms / 预览 ${fmt(stages?.preview,0)} ms${combineTiming} · 识别发布年龄 ${fmt(stages?.capture_to_result,0)} ms` : '');
    text('visionStatus',s.vision_error || (fresh ? `原图坐标：左上角为原点；黄色十字为${raw?'原料':stack?'码垛':'圆环'}准心。相机随夹爪旋转、伸缩。` : '当前无新鲜识别画面，旧画面仅供查看。'));
    $('rings').replaceChildren();
    if(v) for(const [n,p] of Object.entries(v.rings)) {const chip=document.createElement('span');chip.className='chip';chip.textContent=`环${n}: (${fmt(p[0],1)}, ${fmt(p[1],1)}) · ${v.center_sources[n] || ''}`;$('rings').append(chip);}
    if(v) for(const [i,[c,p]] of (v.materials || []).entries()) {const chip=document.createElement('span');chip.className='chip';chip.textContent=`${c ? `颜色${c}` : '颜色未知（不追踪）'}: (${fmt(p[0],1)}, ${fmt(p[1],1)}) · ${v.center_sources[i] || ''}`;$('rings').append(chip);}
    if(!calibrating) {
      $('selectedPoint').hidden=true;$('overlay').hidden=!v;
      if(v) drawAim(v,s.parameters[stack ? 'vision.stack_target_uv_px' : 'vision.target_uv_px'] ?? s.parameters['vision.target_uv_px']);
    }
    if(pending) {
      const result=s.operations.find(op=>op.client===client && op.operation_id===pending.id && !op.pending);
      if(result) {const action=pending.action;pending=null;if(result.ok){
        if(action==='save')dirty=false;
        if(action==='calibration_save') {
          const path=s.calibration_type==='stack' ? 'vision.stack_target_uv_px' : 'vision.target_uv_px';
          inputs.get(path).value=s.parameters[path].join(',');
        }
        message(action==='material_model'?'物料模型切换请求已确认，请查看相机区域的加载状态。':action==='calibration_capture'?'已采集显示画面中的目标中心，请点击保存并同步正式流程。':action==='calibration_save'?'标定点已保存，并同步到正式比赛配置；未保存的运动参数仍保留在下方面板。':action==='save'?'参数已保存并同步正式比赛配置；网页下次开始、正式节点重启后生效。':action==='start'?'任务已开始。':action==='calibration_track_fb'?'已开始仅伸缩跟随2号环，点击停止结束。':action==='preset_adjust'?'正在执行WORK、角度纠偏与2号环底盘校准。':action==='preset_track'?'已开始纯PID追踪，可随时保存当前位置。':action==='preset_save'?'已保存点击时的实际机构位置，并同步正式流程。':action==='select'?'已切换，请确认输入、相机和现场摆放。':action==='tracking_action'?'已停止追踪，等待执行真实取放。':'已发送停止。');}else { $('area').value=s.area;$('visit').value=String(s.visit);message(result.error,true);}}
    }
    if(s.reason && s.state==='ERROR' && !s.active) {
      const ready=s.can_start || s.preset_can_start;
      message(ready ? `当前已恢复就绪；上次任务中止：${s.reason}。请手动重新开始。` : `上次任务中止：${s.reason}；当前：${s.health_error || '等待机构停止确认'}`,!ready);
    }
    else if(s.reason) message(s.reason,s.state==='ERROR');
    else if(s.state==='DONE') message(calibrating && isPreset() ? (s.preset_saved ? '机构初值已保存并同步正式配置。可选择另一个圆环继续标定。' : '底盘调整完成，请选择圆环并开始纯PID追踪。') : tracking ? '真实取放动作已完成；点击开始可重新追踪。' : raw ? `三件${opLabel}取料完成，机构已收纳，可进入下一阶段。` : s.area==='storage' ? `三件${opLabel}放置完成，机构已实际收纳。` : `任务完成。首尾纠偏：${alignment(s.entry_alignment)} / ${alignment(s.final_alignment)}`);
    if(model?.switching || pending?.action==='material_model') {
      displayedFrame=null;$('preview').hidden=true;$('overlay').hidden=true;$('rings').replaceChildren();
    }
    updateRoute();
  }
  async function command(action,settings={}) {
    if(pending && action!=='stop')return;
    const id=globalThis.crypto?.randomUUID?.() || `op-${Date.now()}-${Math.random().toString(16).slice(2)}`;
    pending={id,action};$('taskExecution').disabled=true;updateRoute();
    try {const res=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client,operation_id:id,action,settings})});const body=await res.json();if(!res.ok)throw Error(body.error || '请求失败');message('请求已提交，等待控制端确认…');}
    catch(error){pending=null;$('taskExecution').disabled=!!state?.active;message(`请求结果未知：${error.message}。请查看任务状态，必要时按停止。`,true);updateRoute();}
  }
  $('materialModel').addEventListener('change',()=>{
    if($('materialModel').disabled)return;
    displayedFrame=null;$('preview').hidden=true;$('overlay').hidden=true;$('rings').replaceChildren();
    command('material_model',{use_jhb_model:$('materialModel').value==='jhb',test_combine:$('materialModel').value==='test_combine'});
  });
  $('taskExecution').addEventListener('change',()=>render(state));
  $('order').addEventListener('input',updateRoute);
  $('layout').addEventListener('input',updateRoute);
  $('trackingTarget').addEventListener('change',updateRoute);
  $('calibrationTarget').addEventListener('change',updateCalibration);
  $('calibrationType').addEventListener('change',()=>command('select',{mode:'calibration',calibration_type:$('calibrationType').value}));
  $('calibrationCapture').addEventListener('click',()=>{if(!$('calibrationCapture').disabled) return command('calibration_capture',{frame_token:displayedFrame.token,target:Number($('calibrationTarget').value)});});
  $('calibrationSave').addEventListener('click',()=>{if(!$('calibrationSave').disabled) return command('calibration_save',{sample_id:state.calibration_sample.id});});
  $('presetAdjust').addEventListener('click',()=>{if(!$('presetAdjust').disabled) return command('preset_adjust');});
  $('presetTrack').addEventListener('click',()=>{if(!$('presetTrack').disabled) return command('preset_track',{target:Number($('calibrationTarget').value)});});
  $('presetSave').addEventListener('click',()=>{if(!$('presetSave').disabled) return command('preset_save',{target:Number($('calibrationTarget').value)});});
  $('presetStop').addEventListener('click',()=>command('stop'));
  $('calibrationTrackFb').addEventListener('click',()=>{if(!$('calibrationTrackFb').disabled) return command('calibration_track_fb');});
  $('calibrationFbStop').addEventListener('click',()=>command('stop'));
  $('controlMode').addEventListener('change',()=>command('select',{area:state.area,visit:state.visit,mode:$('controlMode').value}));
  for(const id of ['area','visit']) $(id).addEventListener('change',()=>command('select',{area:$('area').value,visit:Number($('visit').value)}));
  $('start').addEventListener('click',()=>command('start',state.mode==='tracking' ? {area:state.area,visit:state.visit,mode:'tracking',target:Number($('trackingTarget').value)} : {area:state.area,visit:state.visit,order:$('order').value,layout:$('layout').value,execution:$('taskExecution').value}));
  $('trackingAction').addEventListener('click',()=>command('tracking_action'));
  $('stop').addEventListener('click',()=>command('stop'));
  $('parameters').addEventListener('submit',event=>event.preventDefault());
  $('save').addEventListener('click',()=>{
    if(!$('parameters').reportValidity())return;
    const values={};
    for(const f of state.fields) {
      const input=inputs.get(f.path);
      if(f.boolean) {values[f.path]=input.checked;continue;}
      const raw=input.value.trim();values[f.path]=f.point ? (raw ? raw.replace(/^\[|\]$/g,'').split(',').map(Number) : []) : Number(raw);
    }
    command('save',values);
  });
  const stream=new EventSource('/api/events');
  stream.onmessage=event=>{connected=true;text('connection','控制端已连接');try{render(JSON.parse(event.data));}catch(error){message(error.message,true);}};
  stream.onerror=()=>{connected=false;text('connection','连接中断');$('save').disabled=true;updateRoute();message('控制端连接中断；不会自动续跑。',true);};
  setInterval(()=>{if(connected)fetch('/api/heartbeat',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client})}).catch(()=>{});},1000);
  setInterval(async()=>{
    if(previewBusy || !connected || state?.material_model?.switching || pending?.action==='material_model')return;previewBusy=true;
    const selection=state?.calibration_selection_id;
    let nextURL=null;
    try {
      const res=await fetch('/preview.jpg');if(!res.ok)return;
      const header=res.headers?.get('X-Preview-Observation'),metadata=header ? JSON.parse(header) : null;
      const blob=await res.blob();nextURL=URL.createObjectURL(blob);
      if(selection!==state?.calibration_selection_id || state?.material_model?.switching || pending?.action==='material_model')return;
      if(state?.mode==='calibration' && (!metadata || metadata.selection_id!==state.calibration_selection_id))return;
      if(state?.mode==='calibration') {displayedFrame=null;updateCalibration();}
      await new Promise((resolve,reject)=>{const preview=$('preview');preview.onload=resolve;preview.onerror=reject;preview.src=nextURL;});
      if(selection!==state?.calibration_selection_id || state?.material_model?.switching || pending?.action==='material_model') {$('preview').hidden=true;return;}
      const old=imageURL;imageURL=nextURL;nextURL=null;if(old)URL.revokeObjectURL(old);
      if(state?.mode==='calibration') {
        displayedFrame=metadata;displayedAt=Date.now();
        $('preview').hidden=metadata?.selection_id!==state.calibration_selection_id;
        updateCalibration();
      } else {$('preview').hidden=false;displayedFrame=null;}
    }catch{}finally{if(nextURL)URL.revokeObjectURL(nextURL);previewBusy=false;}
  },50);
}
