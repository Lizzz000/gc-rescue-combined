// 浏览器观看页正文。监听、发送和帧准入在 mjpeg.cpp。

// 内嵌观看页：展示标题与实时视频，脚本负责完整 multipart 帧解码、退避重连和离页资源回收。
// HTML／JavaScript 属于响应正文；外部 C++ 注释不进入正文。
// ---- 观看页与 HTTP 辅助；下方字符串属于实际响应正文 ----
const char * kViewerPage =
  "<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
  "<title>GC Rescue</title><style>"
  ":root{color-scheme:light;background:#fff}*{box-sizing:border-box}"
  "body{margin:0;background:#fff;color:#222;font:14px/1.4 sans-serif}"
  "header{padding:10px 16px}"
  "h1{margin:0;font-size:15px;font-weight:650;letter-spacing:.04em}"
  "main{padding:0 12px 8px}img{width:100%;max-height:calc(100vh - 92px);object-fit:contain;"
  "background:#000;border-radius:8px}"
  "img:not([src]){display:none}#connection{margin:4px 0 0;color:#555}"
  "</style></head><body><header><h1>GC Rescue</h1>"
  "<p id=\"connection\" role=\"status\" aria-live=\"polite\" data-state=\"reconnecting\">正在连接画面…</p></header>"
  "<main><img id=\"stream\" alt=\"stream\"></main>"
  // 原生 img 的 multipart 连接结束后不会自行恢复；只保留一个可取消的 fetch，完整帧才交给 img。
  // EOF、网络错误、无有效帧及半帧换代都走同一退避；不改变服务端丢弃过期字节的规则。
  R"VIEWER(<script>
(() => {
  'use strict';
  const view = document.getElementById('stream');
  const status = document.getElementById('connection');
  const frameTimeoutMs = __FRAME_TIMEOUT_MS__;
  const maxFrameBytes = 16 * 1024 * 1024;
  let stopped = false, active = null, currentUrl = null;
  let retryTimer = null, wakeRetry = null, retryMs = 250;
  let received = 0, displayed = 0, replaced = 0;
  Object.assign(view.dataset, {received: '0', displayed: '0', replaced: '0', pending: '0', decoding: '0'});

  function reconnecting() {
    view.removeAttribute('src');
    if (currentUrl) {URL.revokeObjectURL(currentUrl); currentUrl = null;}
    status.dataset.state = 'reconnecting';
    status.textContent = '画面已断开，正在重新连接…';
  }

  async function display(frame, signal) {
    const started = performance.now();
    const url = URL.createObjectURL(new Blob([frame.bytes], {type: 'image/jpeg'}));
    const next = new Image();
    next.src = url;
    try {
      await new Promise((resolve, reject) => {
        const abort = () => reject(new Error('stream cancelled'));
        signal.addEventListener('abort', abort, {once: true});
        next.decode().then(() => {
          signal.removeEventListener('abort', abort);
          resolve();
        }, error => {
          signal.removeEventListener('abort', abort);
          reject(error);
        });
      });
      const decoded = performance.now();
      if (stopped || signal.aborted || decoded - frame.receivedAt >= frameTimeoutMs) {
        throw new Error('stream cancelled or decoded frame expired');
      }
      view.src = url;
      if (currentUrl) {URL.revokeObjectURL(currentUrl);}
      currentUrl = url;
      // 本机收全帧到开始解码、decode Promise 完成的时间；不把它们称为网络或屏幕呈现延迟。
      Object.assign(view.dataset, {
        displayed: String(++displayed), sequence: frame.sequence,
        sourceStampNs: frame.sourceStampNs, ageAtPublishMs: frame.ageAtPublishMs,
        queueMs: String(started - frame.receivedAt), decodeMs: String(decoded - started)
      });
      status.dataset.state = 'online';
      status.textContent = '实时画面';
    } catch (error) {
      URL.revokeObjectURL(url);
      throw error;
    }
  }

  async function run() {
    while (!stopped) {
      const controller = new AbortController();
      active = controller;
      let reader = null, watchdog = null, pending = null, decoding = null;
      const armWatchdog = (receivedAt = performance.now()) => {
        clearTimeout(watchdog);
        // EOF清理可发生在display完成与consume续体之间；旧会话不能重新挂表或清新图。
        if (stopped || controller.signal.aborted || active !== controller) {return;}
        watchdog = setTimeout(() => {
          if (stopped || controller.signal.aborted || active !== controller) {return;}
          reconnecting();
          controller.abort();
        }, Math.max(0, frameTimeoutMs - (performance.now() - receivedAt)));
      };
      const consume = async () => {
        view.dataset.decoding = '1';
        try {
          while (pending && !stopped && !controller.signal.aborted) {
            const frame = pending;
            pending = null;
            view.dataset.pending = '0';
            await display(frame, controller.signal);
            retryMs = 250;
            // 解码耗时不能为旧图续期；有效期从完整帧收到时计算。
            armWatchdog(frame.receivedAt);
          }
        } catch (_) {
          controller.abort();
        } finally {
          decoding = null;
          view.dataset.decoding = '0';
        }
      };
      // 网络读取不等待解码。最多一个正在解码和一个最新完整待解码帧，避免在 fetch 后排旧图。
      const enqueue = frame => {
        if (pending) {view.dataset.replaced = String(++replaced);}
        pending = frame;
        view.dataset.pending = '1';
        view.dataset.received = String(++received);
        view.dataset.receivedSequence = frame.sequence;
        if (!decoding) {decoding = consume();}
      };
      try {
        armWatchdog();
        const response = await fetch('/stream.mjpg', {
          cache: 'no-store', signal: controller.signal
        });
        if (!response.ok || !response.body) {throw new Error('stream unavailable');}
        reader = response.body.getReader();
        let header = '', frame = null, filled = 0, metadata = null;
        while (!stopped) {
          const {value, done} = await reader.read();
          if (done) {break;}
          let offset = 0;
          while (offset < value.length) {
            if (!frame) {
              header += String.fromCharCode(value[offset++]);
              if (header.length > 8192) {throw new Error('oversized stream header');}
              if (!header.endsWith('\r\n\r\n')) {continue;}
              const length = /(?:^|\r\n)Content-Length: *([0-9]+)\r\n/i.exec(header);
              if (!/^(?:\r\n)?--frame\r\n/.test(header) || !length ||
                  Number(length[1]) < 1 || Number(length[1]) > maxFrameBytes) {
                throw new Error('invalid stream frame');
              }
              const field = name => new RegExp('(?:^|\\r\\n)' + name + ': *([^\\r\\n]+)\\r\\n', 'i').exec(header)?.[1] || '';
              metadata = {sequence: field('X-Frame-Sequence'),
                sourceStampNs: field('X-Source-Stamp-Ns'), ageAtPublishMs: field('X-Frame-Age-At-Publish-Ms')};
              frame = new Uint8Array(Number(length[1]));
              filled = 0;
              header = '';
            } else {
              const count = Math.min(frame.length - filled, value.length - offset);
              frame.set(value.subarray(offset, offset + count), filled);
              filled += count;
              offset += count;
              if (filled === frame.length) {
                enqueue({bytes: frame, receivedAt: performance.now(), ...metadata});
                frame = null;
              }
            }
          }
        }
      } catch (_) {
        // 短暂断网、服务重启、半帧中断都由下面同一个循环恢复。
      } finally {
        clearTimeout(watchdog);
        reconnecting();
        controller.abort();
        pending = null;
        view.dataset.pending = '0';
        if (reader) {
          try {await reader.cancel();} catch (_) {}
          reader.releaseLock();
        }
        if (decoding) {await decoding;}
        clearTimeout(watchdog);
        active = null;
      }
      if (stopped) {break;}
      await new Promise(resolve => {
        wakeRetry = resolve;
        retryTimer = setTimeout(resolve, retryMs);
      });
      wakeRetry = null;
      retryMs = Math.min(retryMs * 2, 5000);
    }
  }
  window.addEventListener('pagehide', () => {
    stopped = true;
    if (active) {active.abort();}
    clearTimeout(retryTimer);
    if (wakeRetry) {wakeRetry();}
    reconnecting();
  });
  window.addEventListener('pageshow', event => {
    if (event.persisted) {window.location.reload();}
  });
  run();
})();
</script></body></html>)VIEWER";
