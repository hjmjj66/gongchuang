'use strict';

function point(value) {
  return value ? `(${value[0].toFixed(2)}, ${value[1].toFixed(2)})` : '—';
}

function observationView(item, id) {
  const statuses = { OK: '圆心定位成功', BBOX_ONLY: '仅框中心（退化）', NOT_DETECTED: '未检出',
    TRUNCATED: '目标触边，定位无效', GEOMETRY_FAILED: '圆检测失败', BOX_OUTSIDE_IMAGE: '框在图像外' };
  const box = item && item.bbox_center_px;
  const circle = item && item.circle_center_px;
  const delta = box && circle ? Math.hypot(circle[0] - box[0], circle[1] - box[1]).toFixed(2) : '—';
  return { id, status: item ? (statuses[item.status] || item.status) : '等待有效结果',
    confidence: item && item.yolo_confidence != null ? `${(item.yolo_confidence * 100).toFixed(2)}%` : '—',
    box: point(box), circle: point(circle), delta,
    source: item && item.center_source ? item.center_source : '—',
    reason: item && item.geometry_reason ? item.geometry_reason : '—' };
}

function renderObservations(items) {
  const root = document.getElementById('observations');
  root.replaceChildren();
  for (const id of [1, 2, 3]) {
    const view = observationView(items.find(item => item.ring_id === id), id);
    const card = document.createElement('article');
    const title = document.createElement('h2');
    title.textContent = `ring_${id} · 编号 ${id}`;
    const status = document.createElement('p');
    status.textContent = view.status;
    const list = document.createElement('dl');
    for (const [label, value, className] of [
      ['YOLO 置信度', view.confidence, ''], ['框中心 B (px)', view.box, 'bbox'],
      ['视觉圆心 C (px)', view.circle, 'circle'], ['两中心距离 (px)', view.delta, ''],
      ['最终中心来源', view.source, '']
    ]) {
      const term = document.createElement('dt'); term.textContent = label;
      const detail = document.createElement('dd'); detail.textContent = value; detail.className = className;
      list.append(term, detail);
    }
    const reason = document.createElement('p'); reason.className = 'reason';
    reason.textContent = `几何状态：${view.reason}`;
    card.append(title, status, list, reason); root.append(card);
  }
}

class DisplayRate {
  constructor() { this.samples = []; this.key = null; }
  update(key, now) {
    if (key !== this.key) { this.samples.push(now); this.key = key; }
    while (this.samples.length > 2 && this.samples[1] < now - 2000) this.samples.shift();
    if (this.samples.length < 2 || now - this.samples.at(-1) > 1000) return '0.0';
    return ((this.samples.length - 1) * 1000 / (this.samples.at(-1) - this.samples[0])).toFixed(1);
  }
  reset() { this.samples = []; this.key = null; }
}

function start() {
  const byId = id => document.getElementById(id);
  const rate = new DisplayRate();
  const preview = byId('preview');
  let displayedKey = null;
  renderObservations([]);
  async function poll() {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 4000);
    try {
      const response = await fetch('/api/frame', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      const key = data.image_base64 ? `${data.source}:${data.display_frame_seq}:${data.result ? 'annotated' : 'raw'}` : null;
      if (key !== displayedKey) {
        if (data.image_base64) {
          preview.src = `data:image/jpeg;base64,${data.image_base64}`;
          await preview.decode();
        } else { preview.removeAttribute('src'); }
        displayedKey = key;
      }
      preview.hidden = !data.image_base64;
      byId('empty').hidden = Boolean(data.image_base64);
      byId('empty').textContent = data.message;
      byId('state').textContent = data.state === 'READY' ? '已连接' : data.state;
      byId('state').className = `badge ${data.state === 'READY' ? 'ready' : 'error'}`;
      byId('message').textContent = data.message;
      const formatFps = value => value == null ? '—' : value.toFixed(1);
      byId('camera-fps').textContent = formatFps(data.camera_fps);
      byId('processing-fps').textContent = formatFps(data.processing_fps);
      if (!key) rate.reset();
      byId('display-fps').textContent = data.source === 'image' ? '—' : rate.update(key, performance.now());
      const processingMs = data.result ? data.result.processing_ms : data.last_processing_ms;
      byId('processing-ms').textContent = processingMs == null ? '—' : processingMs.toFixed(1);
      const stages = data.result ? data.result.stages_ms : data.last_stages_ms;
      byId('processing-ms').title = stages && stages.yolo != null
        ? `YOLO ${stages.yolo.toFixed(1)} ms · 几何 ${stages.geometry.toFixed(1)} ms` : '';
      const size = data.image_size ? data.image_size.join(' × ') : '—';
      const age = data.result_age_ms == null ? '—' : `${data.result_age_ms.toFixed(0)} ms`;
      const stageInfo = stages && stages.yolo != null
        ? ` · YOLO ${stages.yolo.toFixed(1)} ms · 几何 ${stages.geometry.toFixed(1)} ms` : '';
      byId('frame-info').textContent = `原图 ${size} · 采集帧 #${data.camera_frame_seq ?? '—'} · 显示帧 #${data.display_frame_seq ?? '—'} · 结果帧年龄 ${age}（软件读帧起算）${stageInfo} · 坐标单位 px`;
      byId('rule').textContent = `每类仅保留最高置信度框，候选置信度须严格大于 ${data.confidence_threshold}（等于也排除）。`;
      byId('device').textContent = `输入：${data.device}${data.source === 'image' ? '（静态图片，不统计实时帧率）' : ''}`;
      renderObservations(data.result ? data.result.observations : []);
    } catch (error) {
      preview.hidden = true; preview.removeAttribute('src'); displayedKey = null; rate.reset();
      byId('empty').hidden = false; byId('empty').textContent = '连接中断，等待恢复';
      byId('message').textContent = `无法获取当前画面：${error.message}`;
      byId('state').textContent = '连接中断'; byId('state').className = 'badge error';
      for (const id of ['camera-fps', 'processing-fps', 'display-fps', 'processing-ms']) byId(id).textContent = '—';
      byId('frame-info').textContent = '当前画面与坐标已清空';
      renderObservations([]);
    } finally { clearTimeout(timeout); setTimeout(poll, 10); }
  }
  poll();
}

if (typeof document !== 'undefined') start();
if (typeof module !== 'undefined') module.exports = { point, observationView, DisplayRate };
