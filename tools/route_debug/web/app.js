'use strict';
const $ = id => document.getElementById(id);
const uuid = () => globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}-${Math.random().toString(16).slice(2)}`;
const client = uuid();
const anchors = {1:[821,141],2:[821,482],3:[821,800],4:[500,141],5:[500,482],6:[500,800],7:[180,141],8:[180,482],9:[180,800]};
const stateNames = {IDLE:'待命',SCREEN_SETUP:'等待屏幕配置',READY:'已就绪',INITIALIZING:'初始化中',NAVIGATING:'导航中',WAITING:'工作等待',STOPPING:'停止中',STOPPED:'已停止',FINISHED:'已完成',FAULT:'故障'};
let mode = 'free', state = null, connected = false, pending = null, lastEvents = '', lastStateAt = 0;
let roads = {}, blockedEdges = new Set(), roadControls = [], settingsLocked = false, receivedState = false;
const edgeKey = ([a,b]) => [Math.min(a,b),Math.max(a,b)].join('-');

function buildRoadChoices(map) {
  roads = map;
  const svg = tag => document.createElementNS('http://www.w3.org/2000/svg',tag);
  roadControls = Object.entries(roads).map(([number,edge]) => {
    const [a,b] = edge.map(n => anchors[n]), key = edgeKey(edge);
    const group = svg('g'), line = svg('line'), badge = svg('g');
    const circle = svg('circle'), label = svg('text'), title = svg('title');
    group.setAttribute('class','road-choice');
    group.setAttribute('role','button');
    group.setAttribute('data-road',number);
    title.textContent = `路段${number}：节点${edge[0]} ↔ ${edge[1]}，点击切换双向禁行`;
    for (const [axis,i] of [['x',0],['y',1]]) {
      line.setAttribute(`${axis}1`,a[i]+(b[i]-a[i])*.15);
      line.setAttribute(`${axis}2`,a[i]+(b[i]-a[i])*.85);
    }
    badge.setAttribute('transform',`translate(${(a[0]+b[0])/2} ${(a[1]+b[1])/2})`);
    circle.setAttribute('r',27);
    label.textContent = number;
    badge.append(circle,label);
    group.append(title,line,badge);
    const toggle = () => {
      if (settingsLocked) return;
      if (blockedEdges.has(key)) blockedEdges.delete(key); else blockedEdges.add(key);
      updateControls();
    };
    group.addEventListener('click',toggle);
    group.addEventListener('keydown',event => {
      if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); toggle(); }
    });
    return {group,key,number};
  });
  $('road-choices').replaceChildren(...roadControls.map(road => road.group));
}

function drawRoadChoices() {
  const selected = [];
  for (const {group,key,number} of roadControls) {
    const blocked = blockedEdges.has(key);
    group.classList.toggle('blocked',blocked);
    group.setAttribute('aria-pressed',String(blocked));
    group.setAttribute('aria-disabled',String(settingsLocked));
    group.setAttribute('tabindex',settingsLocked ? '-1' : '0');
    group.setAttribute('aria-label',`路段${number}，${blocked ? '双向禁行' : '畅通'}`);
    if (blocked) selected.push(number);
  }
  $('blocked-summary').textContent = selected.length ? `障碍路段：${selected.join('、')}（双向禁行）` : '全部道路畅通';
  $('map-status').classList.toggle('has-blocked',selected.length > 0);
  $('clear-blocked').disabled = settingsLocked || selected.length === 0;
}

function selections() {
  const competition = mode !== 'free';
  for (const id of ['start','goal']) {
    const old = Number($(id).value);
    $(id).replaceChildren();
    for (const n of competition ? [1,3] : [1,2,3,4,5,6,7,8,9]) {
      const option = document.createElement('option');
      option.value = n;
      option.textContent = competition ? `启停区${n === 1 ? 1 : 2}（节点${n}）` : `节点 ${n}`;
      $(id).append(option);
    }
    $(id).value = (competition ? [1,3] : [1,2,3,4,5,6,7,8,9]).includes(old) ? old : (id === 'goal' && !competition ? 2 : 1);
  }
  $('align-row').hidden = competition;
  $('exit-start-row').hidden = competition;
  $('mode-hint').textContent = (mode === 'screen' ? '网页开始 → 屏幕选择禁行路径并按 OK → 选择起终点并按 OK → 静止至少2秒。停止为急停；停止或完成后须复位再开始。' : '') + (competition ? '斜移出区 → 2 → 4 → 6 → 8 → 4 → 6 → 8 → 终点角点 → 斜移入区。两轴同时运动，距离读取比赛配置；工作区停好后等待3秒，跳过扫码与抓放。' : '按所选起点、当前朝向及终点导航；可选从启停区内先斜移到角点，再走完整路线。未开启工作对齐时保持车头朝向。');
  $('screen-status').hidden = mode !== 'screen';
  $('clear-blocked').hidden = mode === 'screen';
  $('road-hint').textContent = mode === 'screen' ? '起终点、朝向和禁行路段只读，均由显示屏本轮配置决定。网页保留地图、轨迹、状态及运动参数显示。' : '地图路段编号1～12与比赛一致，区别于底图节点编号1～9。可多选，再次点击取消；红色路段双向禁行。运行时锁定，停止后可修改。';
  updateSettings();
}

function updateSettings() {
  if (mode !== 'free') $('heading').value = Number($('start').value) === 1 ? 'DOWN' : 'UP';
  if (![2,4,6,8].includes(Number($('goal').value))) $('align').checked = false;
  if (![1,3].includes(Number($('start').value))) $('exit-start').checked = false;
  updateControls();
  if (mode !== 'screen' && (!state || !['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(state.state))) drawMarkers(Number($('start').value), Number($('goal').value));
}

function updateControls() {
  const busy = state && ['SCREEN_SETUP','INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(state.state);
  const other = state?.owner && state.owner !== client && ['SCREEN_SETUP','INITIALIZING','NAVIGATING','WAITING','READY'].includes(state.state);
  const locked = Boolean(busy || other || pending);
  settingsLocked = locked || mode === 'screen';
  drawRoadChoices();
  for (const el of document.querySelectorAll('[data-mode],#start,#goal')) el.disabled = locked;
  for (const id of ['start','goal']) $(id).disabled = locked || mode === 'screen';
  $('heading').disabled = locked || mode !== 'free';
  $('align').disabled = locked || ![2,4,6,8].includes(Number($('goal').value));
  $('exit-start').disabled = locked || mode !== 'free' || ![1,3].includes(Number($('start').value));
  $('start-button').disabled = !connected || locked || Boolean(state?.unresolved) || (mode === 'screen' && Boolean(state?.screen_needs_reset));
  $('reset-button').disabled = !connected || Boolean(other || pending || state?.unresolved) || Boolean(busy && !(mode === 'screen' && state.state === 'SCREEN_SETUP'));
  $('stop-button').disabled = !connected || !state || !['SCREEN_SETUP','INITIALIZING','NAVIGATING','WAITING','READY'].includes(state.state);
  if (other) $('operation-message').textContent = '其他页面持有控制权，本页只读；仍可停止。';
}

function settings() {
  if (mode === 'screen') return {mode};
  return {mode, start:Number($('start').value), goal:Number($('goal').value), heading:$('heading').value, align:mode === 'free' && $('align').checked, exit_start:mode === 'free' && $('exit-start').checked, blocked_edges:Object.values(roads).filter(edge => blockedEdges.has(edgeKey(edge)))};
}

async function post(path, payload) {
  const response = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},body:JSON.stringify({...payload,client}),signal:AbortSignal.timeout(3500)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '请求失败');
  return result;
}

async function command(action) {
  const operation_id = uuid();
  pending = operation_id;
  $('operation-message').textContent = '正在提交…';
  updateControls();
  try {
    await post('/api/command', {action, settings:settings(), operation_id});
  } catch (error) {
    // Preserve the same ID if the HTTP response was lost; the server deduplicates it.
    try { await post('/api/command', {action, settings:settings(), operation_id}); }
    catch (retryError) {
      $('operation-message').textContent = `请求状态未知：${retryError.message}。请查看当前状态，勿重复启动。`;
      pending = null;
      updateControls();
    }
  }
}

function drawMarkers(start, goal) {
  for (const [id,n] of [['start-marker',start],['goal-marker',goal]]) {
    if (anchors[n]) { $(id).setAttribute('transform',`translate(${anchors[n].join(' ')})`); $(id).setAttribute('visibility','visible'); }
    else $(id).setAttribute('visibility','hidden');
  }
}

function health(id, label, okay) {
  $(id).textContent = `${label} ${okay ? '✓' : '—'}`;
  $(id).className = okay ? 'ok' : 'bad';
}

function render(s) {
  state = s;
  const screenActive = s.settings?.mode === 'screen' && (!receivedState || ['SCREEN_SETUP','INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(s.state));
  if (screenActive && mode !== 'screen') {
    mode = 'screen';
    document.querySelectorAll('[data-mode]').forEach(b => b.classList.toggle('selected',b.dataset.mode === mode));
    selections();
  }
  if (mode === 'screen' && s.settings?.mode === 'screen') {
    for (const id of ['start','goal']) {
      if (s.settings[id]) $(id).value = s.settings[id];
      else $(id).value = '';
    }
    $('heading').value = s.settings.heading || '';
  }
  if (s.roads && !roadControls.length) buildRoadChoices(s.roads);
  if (s.settings && ((mode === 'screen' && s.settings.mode === 'screen') || !receivedState || ['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(s.state) || (s.state === 'READY' && s.owner !== client))) {
    blockedEdges = new Set((s.settings.blocked_edges || []).map(edgeKey));
  }
  receivedState = true;
  lastStateAt = Date.now();
  if (pending) {
    const result = s.operations?.find(op => op.client === client && op.operation_id === pending);
    if (result && !result.pending) {
      $('operation-message').textContent = result.ok ? '操作已接受' : result.error;
      pending = null;
    }
  }
  $('state').textContent = stateNames[s.state] || s.state;
  $('state').className = `pill ${s.state === 'FAULT' ? 'offline' : ['READY','FINISHED'].includes(s.state) ? 'healthy' : ''}`;
  $('stage').textContent = s.reason || '请选择模式并摆好车辆';
  $('ready-reason').textContent = s.preflight_reason ? `就绪检查：${s.preflight_reason}` : '链路就绪';
  $('route-count').textContent = s.route_count ? `${Math.min(s.route_index+1,s.route_count)} / ${s.route_count}` : '—';
  $('countdown').textContent = (s.state === 'WAITING' || (s.state === 'INITIALIZING' && s.wait_s > 0)) ? `${s.wait_s.toFixed(1)} s` : '—';
  $('screen-status').textContent = s.screen ? `显示屏：${s.screen.error || (s.screen.accepting ? '接收本轮配置' : s.screen.connected ? '已连接，操作接收关闭' : '点击开始时连接')}；串口 ${s.screen.port}` : '等待显示屏状态';
  $('elapsed').textContent = `${(s.elapsed_s || 0).toFixed(1)} s`;
  $('route').textContent = s.parking_maneuver ? `${s.parking_maneuver === 'EXIT' ? '斜移出区' : '斜移入区'} · 每轴 ${s.parking_offset_mm} mm` : (s.plan?.nodes?.length ? s.plan.nodes.join(' → ') : (s.goal ? `目标节点 ${s.goal}` : '—'));
  const o = s.odometry;
  $('distance').textContent = o ? `x ${o.x_m.toFixed(3)} / y ${o.y_m.toFixed(3)} m` : '—';
  $('actual-speed').textContent = o ? `${o.vx_mps.toFixed(3)} / ${o.vy_mps.toFixed(3)} m/s / ${o.wz_rad_s.toFixed(3)} rad/s` : '—';
  $('coordinate').textContent = s.odom_session_id ? `${s.odom_session_id.slice(0,8)} / ${s.motion_epoch}` : '等待原点';
  $('segment-progress').textContent = s.segment ? `${s.segment.progress.toFixed(1)} / ${s.segment.amount.toFixed(1)}；横向 ${s.segment.cross_track_mm.toFixed(1)} mm` : '—';
  $('yaw').textContent = `${(s.yaw_deg || 0).toFixed(2)}°`;
  $('speed').textContent = s.command ? `${s.command[0].toFixed(2)} / ${s.command[1].toFixed(2)} m/s / ${s.command[2].toFixed(1)}°/s` : '—';
  const b = s.board || {}, fresh = s.board_fresh;
  health('board-link','C板连接',fresh && b.connected && b.status_valid);
  health('pc-control','PC控制',fresh && b.pc_control);
  health('odom-health','有效里程',fresh && b.odometry_valid && s.odometry_fresh);
  health('motion-health','运动许可',fresh && b.motion_allowed);
  $('board-reason').textContent = b.fault_reason || (fresh ? '运动许可不等同于物理停稳确认' : '等待新鲜C板反馈');
  if (s.config) {
    const c = s.config;
    $('config-values').textContent = `位置 PID ${c.translation_pid_kp} / ${c.translation_pid_ki} / ${c.translation_pid_kd}；vx 上限 ${c.translation_max_vx_mps} m/s，vy 上限 ${c.translation_max_vy_mps} m/s，各轴允许误差 ±${c.translation_arrival_tolerance_mm} mm；角度 PID ${c.angle_pid_kp} / ${c.angle_pid_ki} / ${c.angle_pid_kd}；角速度上限 ${c.angle_max_speed_deg_s}°/s，允许误差 ±${c.turn_arrival_tolerance_deg}°；出入区斜移每轴 ${c.parking_offset_mm} mm。`;
    $('config-path').textContent = `${c.source}\n斜移参数：${c.mission_source}`;
  }
  $('planned').setAttribute('points',s.plan?.display_points?.map(p => p.join(',')).join(' ') || s.plan?.nodes?.map(n => anchors[n].join(',')).join(' ') || '');
  $('trace').setAttribute('points',s.trace?.map(p => p.join(',')).join(' ') || '');
  if (s.pose) {
    $('car').setAttribute('visibility','visible');
    $('car').setAttribute('transform',`translate(${s.pose.x} ${s.pose.y}) rotate(${s.pose.angle})`);
    $('car').classList.toggle('invalid',!s.pose.valid);
    $('pose-label').textContent = s.pose.valid ? ('实测里程轨迹' + (s.pose.calibration_warning ? '；边长标定非矩形，像素显示近似' : '')) : '位置反馈失效 / 待重新同步';
  } else { $('car').setAttribute('visibility','hidden'); }
  if (s.settings && ((mode === 'screen' && s.settings.mode === 'screen') || ['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(s.state))) drawMarkers(s.settings.start,s.settings.goal);
  const key = JSON.stringify(s.events);
  if (key !== lastEvents) {
    lastEvents = key;
    const list = (s.events || []).slice().reverse().map(event => {
      const li = document.createElement('li'), time = document.createElement('time'), text = document.createElement('span');
      time.textContent = event.time; text.textContent = event.message; li.append(time,text); return li;
    });
    $('logs').replaceChildren(...list);
  }
  updateControls();
}

for (const button of document.querySelectorAll('[data-mode]')) button.addEventListener('click',() => {
  mode = button.dataset.mode;
  document.querySelectorAll('[data-mode]').forEach(b => b.classList.toggle('selected',b === button));
  selections();
  if (mode === 'screen') {
    blockedEdges = new Set((state?.settings?.mode === 'screen' ? state.settings.blocked_edges : [])?.map(edgeKey));
    for (const id of ['start','goal','heading']) $(id).value = state?.settings?.mode === 'screen' ? state.settings[id] || '' : '';
    drawMarkers(state?.settings?.mode === 'screen' ? state.settings.start : null,state?.settings?.mode === 'screen' ? state.settings.goal : null);
    updateControls();
  }
});
for (const id of ['start','goal','heading']) $(id).addEventListener('change',updateSettings);
for (const action of ['start','stop','reset']) $(`${action}-button`).addEventListener('click',() => command(action));
$('clear-blocked').addEventListener('click',() => {
  if (settingsLocked) return;
  blockedEdges.clear();
  updateControls();
});
selections();
const stream = new EventSource('/api/events');
stream.onopen = () => { connected = true; $('connection').textContent = '页面已连接'; $('connection').className = 'pill healthy'; updateControls(); };
stream.onmessage = event => { try { render(JSON.parse(event.data)); } catch(error) { console.error(error); } };
stream.onerror = () => { connected = false; $('connection').textContent = '连接中断'; $('connection').className = 'pill offline'; updateControls(); };
setInterval(() => {
  // Browser connectivity never substitutes for ROS control freshness checks.
  if (connected && Date.now() - lastStateAt < 2500) post('/api/heartbeat',{}).catch(() => {});
},1000);
