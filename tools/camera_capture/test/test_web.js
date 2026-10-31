'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent: '', className: '', hidden: false, disabled: false,
    listeners: {},
    addEventListener(name, handler) { this.listeners[name] = handler; },
    querySelector() { return element(`${id}/span`); },
  });
  return elements.get(id);
}
const documentListeners = {};
let captureRequests = 0;
let completeCapture;
let failCapture = false;
const saved = {
  filename: 'ring_test_000001.jpg', count: 1, image_size: [1280, 720],
};
const fetch = async (url, options) => {
  if (url === '/api/status') return {
    ok: true,
    json: async () => ({ready: true, output_dir: '/tmp/photos', device: '/dev/test',
      image_size: [1280, 720], count: 0, last_saved: null, message: '相机就绪'}),
  };
  assert.equal(url, '/api/capture');
  assert.equal(options.method, 'POST');
  captureRequests += 1;
  return new Promise(resolve => {
    completeCapture = () => resolve({
      ok: !failCapture, status: failCapture ? 503 : 201,
      json: async () => failCapture ? {error: '相机画面已过期'} : saved,
    });
  });
};
const context = vm.createContext({
  document: {getElementById: element, addEventListener(name, handler) { documentListeners[name] = handler; }},
  fetch, AbortSignal, Date, encodeURIComponent, setTimeout() {},
});
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8'), context);
const flush = () => new Promise(resolve => setImmediate(resolve));
function space({repeat = false, editing = false, target = 'body'} = {}) {
  let prevented = false;
  documentListeners.keydown({
    code: 'Space', repeat, target: {closest() { return editing ? {} : null; }, tagName: target},
    preventDefault() { prevented = true; },
  });
  return prevented;
}

(async () => {
  await flush();
  assert.equal(element('capture').disabled, false);
  assert.equal(space({editing: true}), false);
  assert.equal(captureRequests, 0);
  assert.equal(space(), true);
  assert.equal(captureRequests, 1);
  assert.equal(element('capture').disabled, true);
  space({repeat: true});
  element('capture').listeners.click();
  assert.equal(captureRequests, 1, 'holding space and a click during saving must not duplicate capture');
  completeCapture();
  await flush();
  assert.equal(element('count').textContent, 1);
  assert.equal(element('last-photo').hidden, false);
  assert.equal(element('capture').disabled, false);
  space({repeat: true});
  assert.equal(captureRequests, 1, 'a held key must not capture again after a response');
  assert.equal(space({target: 'BUTTON'}), true);
  assert.equal(captureRequests, 2, 'space on the focused button uses the same capture path');
  failCapture = true;
  completeCapture();
  await flush();
  assert.equal(element('count').textContent, 1);
  assert.equal(element('feedback').textContent, '相机画面已过期');
  console.log('PASS: space, held key, focused button, save feedback, failed capture');
})().catch(error => { console.error(error); process.exitCode = 1; });
