'use strict';
const $ = id => document.getElementById(id);
const uuid = () => globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}-${Math.random().toString(16).slice(2)}`;
const client = uuid();
const anchors = {1:[821,141],2:[821,482],3:[821,800],4:[500,141],5:[500,482],6:[500,800],7:[180,141],8:[180,482],9:[180,800]};
const stateNames = {IDLE:'待命',READY:'已就绪',INITIALIZING:'初始化中',NAVIGATING:'导航中',WAITING:'工作等待',STOPPING:'停止中',STOPPED:'已停止',FINISHED:'已完成',FAULT:'故障'};
let mode = 'free', state = null, connected = false, pending = null, lastEvents = '', lastStateAt = 0;

function selections() {
  const competition = mode === 'competition';
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
  $('mode-hint').textContent = competition ? '斜移出区 → 2 → 4 → 6 → 8 → 4 → 6 → 8 → 终点角点 → 斜移入区。两轴同时运动，距离读取比赛配置；工作区停好后等待3秒，跳过扫码与抓放。' : '按所选起点、当前朝向及终点导航；可选从启停区内先斜移到角点，再走完整路线。未开启工作对齐时保持车头朝向。';
  updateSettings();
}

function updateSettings() {
  if (mode === 'competition') $('heading').value = Number($('start').value) === 1 ? 'DOWN' : 'UP';
  if (![2,4,6,8].includes(Number($('goal').value))) $('align').checked = false;
  if (![1,3].includes(Number($('start').value))) $('exit-start').checked = false;
  updateControls();
  if (!state || !['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(state.state)) drawMarkers(Number($('start').value), Number($('goal').value));
}

function updateControls() {
  const busy = state && ['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(state.state);
  const other = state?.owner && state.owner !== client && ['INITIALIZING','NAVIGATING','WAITING','READY'].includes(state.state);
  const locked = Boolean(busy || other || pending);
  for (const el of document.querySelectorAll('[data-mode],#start,#goal')) el.disabled = locked;
  $('heading').disabled = locked || mode === 'competition';
  $('align').disabled = locked || ![2,4,6,8].includes(Number($('goal').value));
  $('exit-start').disabled = locked || mode === 'competition' || ![1,3].includes(Number($('start').value));
  $('start-button').disabled = !connected || locked || Boolean(state?.unresolved);
  $('reset-button').disabled = !connected || locked || Boolean(state?.unresolved);
  $('stop-button').disabled = !connected || !state || !['INITIALIZING','NAVIGATING','WAITING','READY'].includes(state.state);
  if (other) $('operation-message').textContent = '其他页面持有控制权，本页只读；仍可停止。';
}

function settings() {
  return {mode, start:Number($('start').value), goal:Number($('goal').value), heading:$('heading').value, align:mode === 'free' && $('align').checked, exit_start:mode === 'free' && $('exit-start').checked, blocked_edges:[]};
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
  }
}

function health(id, label, okay) {
  $(id).textContent = `${label} ${okay ? '✓' : '—'}`;
  $(id).className = okay ? 'ok' : 'bad';
}

function render(s) {
  state = s;
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
  $('ready-reason').textContent = s.preflight_reason ? `就绪检查：${s.preflight_reason}` : '链路就绪 · 全通地图';
  $('route-count').textContent = s.route_count ? `${Math.min(s.route_index+1,s.route_count)} / ${s.route_count}` : '—';
  $('countdown').textContent = s.state === 'WAITING' ? `${s.wait_s.toFixed(1)} s` : '—';
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
    $('config-values').textContent = `平移 ${c.translation_fast_mps} / ${c.translation_slow_mps} m/s；转向 ${c.turn_fast_deg_s} / ${c.turn_slow_deg_s}°/s（快速 / 慢速）；出入区斜移每轴 ${c.parking_offset_mm} mm。`;
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
  if (s.settings && ['INITIALIZING','NAVIGATING','WAITING','STOPPING'].includes(s.state)) drawMarkers(s.settings.start,s.settings.goal);
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
});
for (const id of ['start','goal','heading']) $(id).addEventListener('change',updateSettings);
for (const action of ['start','stop','reset']) $(`${action}-button`).addEventListener('click',() => command(action));
selections();
const stream = new EventSource('/api/events');
stream.onopen = () => { connected = true; $('connection').textContent = '页面已连接'; $('connection').className = 'pill healthy'; updateControls(); };
stream.onmessage = event => { try { render(JSON.parse(event.data)); } catch(error) { console.error(error); } };
stream.onerror = () => { connected = false; $('connection').textContent = '连接中断'; $('connection').className = 'pill offline'; updateControls(); };
setInterval(() => {
  // Browser connectivity never substitutes for ROS control freshness checks.
  if (connected && Date.now() - lastStateAt < 2500) post('/api/heartbeat',{}).catch(() => {});
},1000);
