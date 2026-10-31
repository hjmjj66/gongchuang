(() => {
  'use strict';
  const byId = id => document.getElementById(id);
  const button = byId('capture');
  const feedback = byId('feedback');
  let ready = false;
  let busy = false;
  let lastName = null;

  function updateButton() {
    button.disabled = busy || !ready;
    byId('capture-label').textContent = busy ? '正在保存…' : '拍照保存';
  }

  function showSaved(saved) {
    if (!saved || saved.filename === lastName) return;
    lastName = saved.filename;
    byId('count').textContent = saved.count;
    byId('last-name').textContent = saved.filename;
    byId('saved-size').textContent = saved.image_size.join(' × ');
    byId('last-photo').src = `/last.jpg?n=${encodeURIComponent(saved.filename)}`;
    byId('last-photo').hidden = false;
    byId('last-empty').hidden = true;
  }

  async function request(path, options = {}) {
    const response = await fetch(path, {
      ...options, cache: 'no-store', signal: AbortSignal.timeout(10000),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    return data;
  }

  async function pollStatus() {
    try {
      const status = await request('/api/status');
      ready = status.ready;
      byId('connection').className = `status${ready ? ' ready' : ''}`;
      byId('connection').querySelector('span').textContent = ready ? '相机已连接' : '等待相机';
      byId('output-dir').textContent = status.output_dir;
      byId('device').textContent = status.device;
      byId('count').textContent = status.count;
      byId('resolution').textContent = status.image_size ? `${status.image_size.join(' × ')} 原图` : '等待相机';
      byId('preview-message').hidden = ready;
      byId('preview-message').textContent = status.message;
      showSaved(status.last_saved);
      if (!ready && !busy) {
        feedback.className = '';
        feedback.textContent = status.message;
      } else if (ready && !busy && !lastName) {
        feedback.textContent = '点击按钮，或按空格保存一张原图。';
      }
    } catch (error) {
      ready = false;
      byId('connection').className = 'status error';
      byId('connection').querySelector('span').textContent = '连接中断';
      byId('preview-message').hidden = false;
      byId('preview-message').textContent = '无法连接板端服务，请检查网络和采集程序。';
      feedback.className = 'error';
      feedback.textContent = `连接失败：${error.message}`;
    } finally {
      updateButton();
      setTimeout(pollStatus, 1000);
    }
  }

  async function capture() {
    if (busy || !ready) return;
    busy = true;
    updateButton();
    feedback.className = '';
    feedback.textContent = '正在将原图保存到板子…';
    try {
      const saved = await request('/api/capture', {method: 'POST'});
      showSaved(saved);
      feedback.className = 'success';
      feedback.textContent = `已保存第 ${saved.count} 张照片。`;
    } catch (error) {
      feedback.className = 'error';
      feedback.textContent = error.name === 'TimeoutError'
        ? '请求超时，保存结果暂未确认；请查看照片计数和最近保存记录。'
        : error.message;
    } finally {
      busy = false;
      updateButton();
    }
  }

  button.addEventListener('click', capture);
  document.addEventListener('keydown', event => {
    if (event.code !== 'Space' || event.ctrlKey || event.altKey || event.metaKey) return;
    if (event.target.closest('input, textarea, select, [contenteditable="true"]')) return;
    event.preventDefault();
    if (!event.repeat) capture();
  });
  byId('preview').addEventListener('error', () => {
    setTimeout(() => { byId('preview').src = `/stream.mjpg?t=${Date.now()}`; }, 2000);
  });
  pollStatus();
})();
