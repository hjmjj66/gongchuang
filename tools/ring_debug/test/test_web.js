'use strict';
const assert = require('node:assert/strict');
const { observationView, point, DisplayRate } = require('../web/app.js');

assert.equal(point(null), '—');
assert.equal(point([123.456, 78.9]), '(123.46, 78.90)');
const bbox = { status: 'BBOX_ONLY', yolo_confidence: 0.88, bbox_center_px: [20, 30],
  circle_center_px: null, center_source: 'bbox', geometry_reason: 'NO_CIRCLE' };
const fallback = observationView(bbox, 1);
assert.equal(fallback.circle, '—');
assert.equal(fallback.box, '(20.00, 30.00)');
assert.equal(fallback.confidence, '88.00%');
assert.equal(fallback.status, '仅框中心（退化）');
const circle = observationView({ ...bbox, status: 'OK', circle_center_px: [23, 34], center_source: 'circle' }, 1);
assert.equal(circle.delta, '5.00');
assert.equal(circle.circle, '(23.00, 34.00)');
assert.equal(observationView(null, 2).box, '—');
assert.equal(observationView({ status: 'NOT_DETECTED' }, 3).circle, '—');
const rate = new DisplayRate();
assert.equal(rate.update('1', 1000), '0.0');
assert.equal(rate.update('2', 1100), '10.0');
assert.equal(rate.update('2', 1200), '10.0'); // repeated API payloads are not new displayed frames
assert.equal(rate.update('2', 2300), '0.0');
rate.reset();
assert.equal(rate.update('3', 2400), '0.0');
console.log('ring_debug web tests passed');

const vm = require('node:vm');
const fs = require('node:fs');
class Element {
  constructor() { this.children = []; this.hidden = false; this.textContent = ''; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = [...children]; }
  removeAttribute(name) { delete this[name]; }
  async decode() {}
}

async function testPageTransitions() {
  const elements = new Map();
  const byId = id => {
    if (!elements.has(id)) elements.set(id, new Element());
    return elements.get(id);
  };
  const callbacks = [];
  const payload = { state: 'READY', message: '识别中', source: 'camera', confidence_threshold: 0.75,
    image_base64: 'JPEG', display_frame_seq: 1, camera_frame_seq: 1, image_size: [1280, 720],
    camera_fps: 30, processing_fps: 14, result_age_ms: 70, device: '/dev/test',
    result: { processing_ms: 25, observations: [{ ...bbox, ring_id: 1 }] } };
  let current = payload;
  vm.runInNewContext(fs.readFileSync(require.resolve('../web/app.js'), 'utf8'), {
    document: { getElementById: byId, createElement: () => new Element() },
    fetch: async () => { if (current instanceof Error) throw current; return { ok: true, json: async () => current }; },
    AbortController, performance: { now: () => 1000 },
    setTimeout: (fn, delay) => { if (delay === 10) callbacks.push(fn); return 1; }, clearTimeout() {}
  });
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(byId('preview').hidden, false);
  assert.equal(byId('camera-fps').textContent, '30.0');
  assert.equal(byId('processing-fps').textContent, '14.0');
  assert.equal(byId('observations').children.length, 3);
  assert.equal(byId('observations').children[0].children[2].children[5].textContent, '—');
  assert.match(byId('rule').textContent, /严格大于 0.75/);
  current = { ...payload, state: 'STALE_FRAME', image_base64: null, result: null, display_frame_seq: null };
  await callbacks.shift()();
  assert.equal(byId('preview').hidden, true);
  assert.equal(byId('observations').children[0].children[2].children[3].textContent, '—');
  current = new Error('offline');
  await callbacks.shift()();
  assert.equal(byId('camera-fps').textContent, '—');
  assert.equal(byId('state').textContent, '连接中断');
  assert.equal(byId('preview').hidden, true);
  console.log('ring_debug page transition tests passed');
}
testPageTransitions().catch(error => { console.error(error); process.exitCode = 1; });
