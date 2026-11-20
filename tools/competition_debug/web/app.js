'use strict';
const $ = id => document.getElementById(id);
const uuid = () => globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}`;
const client = uuid(), anchors = {1:[821,141],2:[821,482],3:[821,800],4:[500,141],5:[500,482],6:[500,800],7:[180,141],8:[180,482],9:[180,800]};
const names = {IDLE:'待命',SCREEN_SETUP:'等待屏幕配置',INITIALIZING:'初始化中',NAVIGATING:'运行 · 导航中',SCANNING:'等待 · 真实扫码',WORKING:'运行 · 区域作业',STOPPING:'正在停止，等待释放确认',STOPPED:'停止完成',FINISHED:'整场完成',FAULT:'故障'};
let state = null, mode = 'web', view = 'map', connected = false, pending = null, lastAt = 0, logKey = '';
let blocked = new Set(), roads = {}, choices = [];
const edgeKey = edge => [...edge].sort((a,b)=>a-b).join('-');

function controls() {
  const lock = Boolean(state?.locked || pending);
  const busy = ['SCREEN_SETUP','INITIALIZING','NAVIGATING','SCANNING','WORKING','STOPPING'].includes(state?.state);
  for (const el of document.querySelectorAll('[data-mode],#raw,#coarse,#storage')) el.disabled = lock;
  for (const id of ['start','goal','clear']) $(id).disabled = lock || mode === 'screen';
  $('start-button').disabled = !connected || lock;
  $('stop-button').disabled = !connected || (!busy && !state?.camera_manual) || state?.state === 'STOPPING';
  $('reset-button').disabled = !connected || busy || Boolean(pending || state?.unresolved);
  $('camera-toggle').disabled = !connected || lock || state?.state !== 'IDLE';
  $('camera-toggle').textContent = state?.camera_manual && state.camera_kind !== 'qr' ? '关闭主相机预览' : '打开主相机预览';
  $('qr-camera-toggle').disabled = !connected || lock || state?.state !== 'IDLE';
  $('qr-camera-toggle').textContent = state?.camera_manual && state.camera_kind === 'qr' ? '关闭扫码相机预览' : '打开扫码相机预览';
  for (const b of document.querySelectorAll('[data-mode]')) b.setAttribute('aria-pressed', String(b.dataset.mode === mode));
  $('mode-hint').textContent = mode === 'screen' ? '网页开始后才接收屏幕操作：路径页OK → 起终点页OK（1/3）→ 大START按钮 → 至少静止2秒且执行端就绪后启动。' : '选择起终点，点击地图路段编号设置禁行，然后一键跑完整场。';
  const selected = [];
  for (const {g,key,number} of choices) {
    g.classList.toggle('blocked',blocked.has(key));
    g.setAttribute('aria-disabled',String(lock || mode === 'screen'));
    g.setAttribute('aria-pressed',String(blocked.has(key)));
    g.setAttribute('tabindex', lock || mode === 'screen' ? '-1' : '0');
    if (blocked.has(key)) selected.push(number);
  }
  $('blocked').textContent = '禁行路径：'+(selected.join('、') || '无')+'（双向）';
}

function buildRoads(map) {
  roads = map;
  const svg = tag => document.createElementNS('http://www.w3.org/2000/svg',tag);
  choices = Object.entries(roads).map(([number,edge]) => {
    const g=svg('g'), line=svg('line'), badge=svg('g'), circle=svg('circle'), text=svg('text');
    const a=anchors[edge[0]], b=anchors[edge[1]], key=edgeKey(edge);
    g.setAttribute('class','road'); g.setAttribute('role','button'); g.setAttribute('aria-label',`路段${number}双向禁行`);
    for (const [axis,i] of [['x',0],['y',1]]) {line.setAttribute(axis+'1',a[i]);line.setAttribute(axis+'2',b[i]);}
    badge.setAttribute('transform',`translate(${(a[0]+b[0])/2} ${(a[1]+b[1])/2})`);
    text.textContent=number; badge.append(circle,text); g.append(line,badge);
    const toggle=()=>{if(state?.locked || pending || mode==='screen')return; if(blocked.has(key))blocked.delete(key);else blocked.add(key);controls();};
    g.addEventListener('click',toggle);g.addEventListener('keydown',e=>{if(e.key===' '||e.key==='Enter'){e.preventDefault();toggle();}});
    return {g,key,number};
  });
  $('roads').replaceChildren(...choices.map(c=>c.g));
}

function settings() {
  return {mode,start:Number($('start').value),goal:Number($('goal').value),
    blocked_edges:Object.values(roads).filter(e=>blocked.has(edgeKey(e))),
    real_actions:Object.fromEntries(['raw','coarse','storage'].map(a=>[a,$(a).checked]))};
}

async function post(path, payload) {
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...payload,client}),signal:AbortSignal.timeout(3500)});
  const value=await response.json();if(!response.ok)throw new Error(value.error||'请求失败');return value;
}

async function command(action) {
  const operation_id=uuid();pending=operation_id;controls();$('operation').textContent='正在提交…';
  try {await post('/api/command',{action,settings:settings(),operation_id});}
  catch(error) {pending=null;$('operation').textContent='请求响应未知：'+error.message+'；请查看执行状态，停止按钮仍可用';controls();}
}

function render(s) {
  if(s.camera_kind!==state?.camera_kind){$('preview').hidden=true;$('camera-error').hidden=true;}
  state=s;lastAt=Date.now();
  if (!choices.length && s.roads) buildRoads(s.roads);
  const op=s.operations?.find(o=>o.client===client && o.operation_id===pending && !o.pending);
  if(op){pending=null;$('operation').textContent=op.error||'操作已接受';}
  if(s.locked){
    mode=s.config_mode;
    for(const area of ['raw','coarse','storage']) $(area).checked=s.real_actions[area];
    for(const id of ['start','goal']) $(id).value=s.settings?.[id]||'';
    blocked=new Set((s.settings?.blocked_edges||[]).map(edgeKey));
  }
  $('status').textContent=s.state==='WORKING' && (s.reason||'').includes('等待') ? '等待 · 区域作业' : names[s.state]||s.state;
  $('stage').textContent=(s.batch?`第${s.batch}轮 · `:'')+(s.stage||'')+' · '+(s.action||'');
  $('reason').textContent=s.reason||s.preflight_reason||'';
  $('progress').textContent=`导航 ${Math.min(s.route_index+1,s.route_count)}/${s.route_count}；${s.wait_s>0?'启动等待 '+s.wait_s.toFixed(1)+' 秒；':''}本轮 ${Math.round(s.elapsed_s||0)} 秒`;
  const h=s.hardware||{};
  $('hardware').textContent=`底盘 ${h.board?'就绪':'未就绪'} · 导航 ${h.planner?'在线':'等待'} · 机构 ${h.arm?'就绪':'等待'} · 屏幕 ${h.screen?'已连接':'未连接'} · 雷达 ${h.lidar||'—'}`;
  $('task-code').textContent='任务码：'+(s.qr_result||'等待本轮真实扫码');
  $('run-config').textContent=(s.config_mode==='screen'?'显示屏配置':'网页配置')+`；起点 ${s.settings?.start||'待选'} → 最终停车 ${s.settings?.goal||'待选'}；`+['raw','coarse','storage'].map((a,i)=>['原料','粗加工','暂存'][i]+(s.real_actions?.[a]?'真实':'停留替代')).join(' / ');
  const results=s.results||[];
  $('result').textContent=`真实取放完成 ${results.filter(r=>r.outcome==='REAL_COMPLETED').length} 次；停留替代完成 ${results.filter(r=>r.outcome==='DWELL_COMPLETED').length} 次`;
  $('camera-state').textContent=s.camera||'当前阶段未启用相机';
  if(!s.camera_enabled){$('preview').hidden=true;$('camera-error').hidden=true;}
  const c=s.config;
  if(c){
    $('config').textContent=`雷达 ${c.use_lidar?'开启':'关闭'}；出入区每轴 ${c.parking_offset_mm} mm；粗加工及首次暂存收纳后返回P0；导航上限 vx ${c.translation_max_vx_mps} / vy ${c.translation_max_vy_mps} m/s。区域视觉、PID、动作号沿用正式配置，无网页调参。`;
    $('sources').textContent=JSON.stringify({比赛:c.mission_source,导航:c.source,C板:c.cboard_source,扫码:c.qr_source,屏幕:c.screen_source,区域:c.region_sources,动作号:c.arm_commands,区域视觉:c.camera_sources},null,2);
  }
  $('planned').setAttribute('points',(s.plan?.display_points||s.plan?.nodes?.map(n=>anchors[n])||[]).map(p=>p.join(',')).join(' '));
  $('trace').setAttribute('points',(s.trace||[]).map(p=>p.join(',')).join(' '));
  $('car').setAttribute('visibility',s.pose?'visible':'hidden');
  if(s.pose){$('car').setAttribute('transform',`translate(${s.pose.x} ${s.pose.y}) rotate(${s.pose.angle})`);$('car').classList.toggle('invalid',!s.pose.valid);}
  const key=JSON.stringify(s.events);
  if(key!==logKey){logKey=key;$('logs').replaceChildren(...(s.events||[]).slice().reverse().map(e=>{const li=document.createElement('li'),t=document.createElement('time');t.textContent=e.time;li.append(t,document.createTextNode((e.batch?`第${e.batch}轮：`:'')+e.message));return li;}));}
  controls();
}

for(const b of document.querySelectorAll('[data-mode]'))b.addEventListener('click',()=>{mode=b.dataset.mode;controls();});
for(const b of document.querySelectorAll('[data-view]'))b.addEventListener('click',()=>{view=b.dataset.view;$('map-view').hidden=view!=='map';$('camera-view').hidden=view!=='camera';for(const x of document.querySelectorAll('[data-view]'))x.setAttribute('aria-pressed',String(x===b));});
for(const action of ['start','stop','reset'])$(action+'-button').addEventListener('click',()=>command(action));
$('camera-toggle').addEventListener('click',()=>command(state?.camera_manual && state.camera_kind!=='qr'?'camera_close':'camera_open'));
$('qr-camera-toggle').addEventListener('click',()=>command(state?.camera_manual && state.camera_kind==='qr'?'camera_close':'qr_camera_open'));
$('clear').addEventListener('click',()=>{blocked.clear();controls();});
$('save').addEventListener('click',()=>{
  if(!state)return;
  const content=JSON.stringify({run_id:state.run_id,config:state.config,mode:state.config_mode,settings:state.settings,real_actions:state.real_actions,task_code:state.qr_result,state:state.state,reason:state.reason,results:state.results,log:state.log},null,2);
  const url=URL.createObjectURL(new Blob([content],{type:'application/json;charset=utf-8'})),a=document.createElement('a');a.href=url;a.download=`全链路比赛-${state.run_id||'待命'}.json`;a.click();URL.revokeObjectURL(url);
});
const stream=new EventSource('/api/events');
stream.onopen=()=>{connected=true;$('connection').textContent='页面已连接';controls();};
stream.onmessage=e=>render(JSON.parse(e.data));
stream.onerror=()=>{connected=false;$('connection').textContent='连接中断';controls();};
setInterval(()=>{if(connected && Date.now()-lastAt<2500)post('/api/heartbeat',{}).catch(()=>{});},1000);
let previewBusy=false;
setInterval(async()=>{
  if(view!=='camera'||!state?.camera_enabled||previewBusy)return;previewBusy=true;
  const cameraKind=state.camera_kind;
  try{
    const r=await fetch('/preview.jpg?t='+Date.now(),{signal:AbortSignal.timeout(2500)});
    if(!r.ok)throw new Error(r.status===503?'暂无新画面，请查看上方相机状态':`画面请求失败（HTTP ${r.status}）`);
    const blob=await r.blob();
    if(!state?.camera_enabled||state.camera_kind!==cameraKind)return;
    const url=URL.createObjectURL(blob),old=$('preview').src;
    $('preview').src=url;$('preview').hidden=false;$('camera-error').hidden=true;
    if(old?.startsWith('blob:'))URL.revokeObjectURL(old);
  }
  catch(error){if(state?.camera_enabled && state.camera_kind===cameraKind){$('preview').hidden=true;$('camera-error').textContent='预览未显示：'+error.message;$('camera-error').hidden=false;}}
  finally{previewBusy=false;}
},250);
controls();
