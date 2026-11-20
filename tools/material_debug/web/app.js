'use strict';

function point(value) {
  return value ? `(${value[0].toFixed(2)}, ${value[1].toFixed(2)})` : '—';
}

function observationView(item) {
  const statuses = { OK:'中心可用', BBOX_ONLY:'框中心估计（可用）',
    TRUNCATED:'目标触边，无效', COLOR_AMBIGUOUS:'颜色歧义，无效',
    GEOMETRY_FAILED:'圆检测失败，无效', BOX_OUTSIDE_IMAGE:'框在图像外，无效',
    COLOR_UNMATCHED:'未匹配到 YOLO 颜色，中心仅供初始跟随',
    COLOR_MATCH_AMBIGUOUS:'YOLO 颜色匹配歧义，中心仅供初始跟随',
    COLOR_REJECTED:item.center_px ? 'Lab 判色拒判，已有中心仅供初始跟随' : 'Lab 判色拒判，未取得中心' };
  const b = item.bbox_center_px, c = item.circle_center_px;
  const circle = c ? point(c) : item.geometry_reason === 'NOT_APPLICABLE' ? '不适用' : '未取得';
  return { title:`${item.color_id} · ${item.color_name} · ${item.raw_label}`,
    status:statuses[item.status] || item.status,
    confidence:item.yolo_confidence == null ? '—' : `${(item.yolo_confidence * 100).toFixed(2)}%`,
    box:point(b), circle, final:point(item.center_px), source:item.center_source || '—',
    delta:b && c ? Math.hypot(b[0]-c[0], b[1]-c[1]).toFixed(2) : '—',
    reason:item.geometry_reason || '—',
    colorConfidence:item.color_confidence == null ? '—' : `${(item.color_confidence*100).toFixed(2)}%`,
    colorDistance:item.color_d2 == null ? '—' : item.color_d2.toFixed(2),
    colorReason:item.color_reason || '—',
    conflicts:(item.conflicting_colors || []).join(', ') || '—' };
}

function renderObservations(items) {
  const root = document.getElementById('observations');
  root.replaceChildren();
  for (const [index,item] of items.entries()) {
    const view = observationView(item);
    const card = document.createElement('article');
    const title = document.createElement('h2'); title.textContent = `#${index+1} ${view.title}`;
    const status = document.createElement('p'); status.textContent = view.status;
    const list = document.createElement('dl');
    for (const [label,value,color] of [['YOLO 置信度',view.confidence,''],
      ['框中心 B (px)',view.box,'bbox'], ['几何圆心 C (px)',view.circle,'circle'],
      ['最终中心 F (px)',view.final,'final'], ['B 到 C (px)',view.delta,''],
      ['中心来源',view.source,''], ['冲突颜色编号',view.conflicts,'']]) {
      const term = document.createElement('dt'); term.textContent = label;
      const detail = document.createElement('dd'); detail.textContent = value; detail.className = color;
      list.append(term, detail);
    }
    if (item.identity_source === 'jhb_lab') {
      for (const [label,value] of [['Lab 判色置信度',view.colorConfidence],
        ['Lab 距离 d²',view.colorDistance],['判色状态',view.colorReason]]) {
        const term=document.createElement('dt'); term.textContent=label;
        const detail=document.createElement('dd'); detail.textContent=value;
        list.append(term,detail);
      }
    }
    if (item.identity_source === 'yolo_jhb') {
      for (const [label,value] of [['原 YOLO 颜色置信度',view.colorConfidence],
        ['原 YOLO 颜色框',item.color_bbox_xyxy ? item.color_bbox_xyxy.map(n=>n.toFixed(1)).join(', ') : '—'],
        ['匹配状态',view.colorReason]]) {
        const term=document.createElement('dt'); term.textContent=label;
        const detail=document.createElement('dd'); detail.textContent=value;
        list.append(term,detail);
      }
    }
    const reason = document.createElement('p'); reason.textContent = `几何状态：${view.reason}`;
    card.append(title,status,list,reason); root.append(card);
  }
}

class DisplayRate {
  constructor() { this.reset(); }
  reset() { this.samples = []; this.key = null; }
  update(key, now) {
    if (key !== null && key !== this.key) { this.samples.push(now); this.key = key; }
    while (this.samples.length > 2 && this.samples[1] < now-2000) this.samples.shift();
    if (this.samples.length < 2 || now-this.samples.at(-1) > 1000) return '0.0';
    const elapsed = Math.max(this.samples.at(-1),now-1000)-this.samples[0];
    return elapsed > 0 ? ((this.samples.length-1)*1000/elapsed).toFixed(1) : '0.0';
  }
}

function start() {
  const byId = id => document.getElementById(id), rate = new DisplayRate();
  const preview = byId('preview');
  const modelSelect = byId('model-select'), switchStatus = byId('switch-status');
  let displayedKey = null, switchingRequest = false, switchMessage = '', selectionEpoch = 0;
  modelSelect.addEventListener('change',async () => {
    selectionEpoch += 1;
    switchingRequest = true;
    modelSelect.disabled = true;
    switchMessage = '';
    switchStatus.textContent = '正在请求切换…';
    preview.hidden = true; preview.removeAttribute('src'); displayedKey = null; rate.reset();
    renderObservations([]);
    const controller = new AbortController(), timeout = setTimeout(()=>controller.abort(),10000);
    try {
      const response = await fetch('/api/mode',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({use_jhb_model:modelSelect.value === 'jhb',test_combine:modelSelect.value === 'test_combine'}),signal:controller.signal});
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    } catch (error) {
      switchMessage = `切换请求失败：${error.message}`;
    } finally {
      clearTimeout(timeout);
      switchingRequest = false;
    }
  });
  async function poll() {
    const epoch = selectionEpoch;
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(),4000);
    try {
      const response = await fetch('/api/frame',{cache:'no-store',signal:controller.signal});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      if (switchingRequest || epoch !== selectionEpoch) return;
      modelSelect.value = data.test_combine ? 'test_combine' : data.use_jhb_model ? 'jhb' : 'original';
      modelSelect.disabled = data.switching || data.can_switch_model === false;
      switchStatus.textContent = data.switching ? '正在加载模型…' : switchMessage || data.switch_error ||
        (data.can_switch_model === false ? '手动几何模式不支持切换模型' : `当前：${data.test_combine ? 'test_combine · 原 YOLO 颜色 + JHB 中心' : data.use_jhb_model ? 'JHB 模型' : '原有识别'}，选择后立即切换`);
      const key = data.image_base64 ? `${data.mode_revision || 0}:${data.source}:${data.display_frame_seq}:${data.result ? 'annotated' : 'raw'}` : null;
      if (key !== displayedKey) {
        if (data.image_base64) {
          preview.src = `data:image/jpeg;base64,${data.image_base64}`;
          await preview.decode();
          if (switchingRequest || epoch !== selectionEpoch) return;
        } else preview.removeAttribute('src');
        displayedKey = key;
      }
      preview.hidden = !data.image_base64;
      byId('empty').hidden = Boolean(data.image_base64);
      byId('empty').textContent = data.message;
      byId('state').textContent = data.state;
      byId('state').className = `badge ${data.state === 'READY' ? 'ready' : 'error'}`;
      byId('message').textContent = data.message;
      const fps = value => value == null ? '—' : value.toFixed(1);
      byId('camera-fps').textContent = fps(data.camera_fps);
      byId('processing-fps').textContent = fps(data.processing_fps);
      if (!key) rate.reset();
      byId('display-fps').textContent = data.source === 'image' ? '—' : rate.update(key,performance.now());
      const ms = data.result ? data.result.processing_ms : data.last_processing_ms;
      const stages = data.result ? data.result.stages_ms : data.last_stages_ms;
      byId('processing-ms').textContent = ms == null ? '—' : ms.toFixed(1);
      const stageText = Object.entries(stages || {}).map(([name,value]) => `${name} ${value.toFixed(1)} ms`).join(' · ');
      const age = data.result_age_ms == null ? '—' : `${data.result_age_ms.toFixed(0)} ms`;
      byId('frame-info').textContent = `原图 ${data.image_size ? data.image_size.join(' × ') : '—'} · 采集帧 #${data.camera_frame_seq ?? '—'} · 显示帧 #${data.display_frame_seq ?? '—'} · 结果年龄 ${age}（软件读帧起算） · ${stageText} · 坐标单位 px`;
      byId('mode').textContent = data.backend === 'test_combine' ? 'test_combine：原 YOLO 六色类别 + JHB 顶帽中心（不运行 Lab 判色）' : data.backend === 'jhb' ? `JHB：物料／顶帽配对 → ${data.refine_circle ? '几何圆心／框中心' : '框中心'} → Lab 六色判别` : data.backend === 'manual' ? '手动顶面 ROI：人工指定颜色' : data.backend === 'hsv' ? '传统 HSV 六色分割' : data.model_kind === 'top' ? (data.refine_circle === false ? 'YOLO 顶面：仅框中心估计' : 'YOLO 顶面：霍夫圆心 / 框中心估计') : 'YOLO 整体：框中心估计';
      byId('model').textContent = data.model_path ? `${data.model_path}${data.color_model_path ? ` · 颜色模型：${data.color_model_path}` : ''} · ${data.class_labels.join(', ')}` : '';
      byId('rule').textContent = data.backend === 'test_combine' ? '同帧框 IoU、顶帽中心包含和一对一匹配；未匹配或歧义时只保留未知中心供初始跟随。' : data.backend === 'jhb' ? `保留同色多实例；顶帽与物料一对一配对，Lab 拒判时中心仅供初始跟随。筛除顶帽 ${data.result ? (data.result.rejected_candidates || []).length : 0} 个。` : data.backend === 'manual' ? '手动框与颜色不代表模型识别。' : data.backend === 'hsv' ? '保留全部轮廓候选；高度重叠颜色标记歧义。' : `保留所有实例；置信度严格大于 ${data.confidence_threshold}。`;
      byId('device').textContent = `输入：${data.device}${data.source === 'image' ? '（静态图片）' : ''}`;
      const warnings = data.result ? (data.result.warnings || []) : [];
      if (warnings.length) byId('message').textContent += ` · ${warnings.join('; ')}`;
      renderObservations(data.result ? data.result.observations : []);
    } catch (error) {
      modelSelect.disabled = true;
      switchStatus.textContent = '连接中断，暂不可切换';
      preview.hidden = true; preview.removeAttribute('src'); displayedKey = null; rate.reset();
      byId('empty').hidden = false; byId('empty').textContent = '连接中断，等待恢复';
      byId('state').textContent = '连接中断'; byId('state').className = 'badge error';
      byId('message').textContent = `无法获取当前画面：${error.message}`;
      for (const id of ['camera-fps','processing-fps','display-fps','processing-ms']) byId(id).textContent = '—';
      byId('frame-info').textContent = '当前画面与坐标已清空';
      renderObservations([]);
    } finally { clearTimeout(timeout); setTimeout(poll,10); }
  }
  renderObservations([]); poll();
}

if (typeof document !== 'undefined') start();
if (typeof module !== 'undefined') module.exports = {point,observationView,DisplayRate,renderObservations,start};
