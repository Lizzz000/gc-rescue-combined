const byId = (id) => document.getElementById(id);
const token = document.querySelector('meta[name="control-token"]').content;
const portSelect = byId("port-select");
const controls = {
  connect: byId("connect"),
  refresh: byId("refresh-ports"),
  arm: byId("arm"),
  run: byId("run"),
  stop: byId("stop"),
  origin: byId("origin"),
  return: byId("return"),
  duration: byId("duration"),
  linear: byId("linear-speed"),
  angular: byId("angular-speed"),
  returnLinear: byId("return-linear"),
  returnAngular: byId("return-angular"),
  modeTimed: byId("mode-timed"),
  modeKeys: byId("mode-keys"),
};
const form = byId("motion-form");
const keyPad = byId("key-pad");
const directionSet = form.querySelector("fieldset");
const stateLabels = ["已失能", "使能准备中", "就绪", "运行中", "制动中", "返航中", "返航到位，提交零速中"];
const keyboardKeys = new Set();
const pointerKeys = new Set();
let inputMode = "timed";
let driving = false;
let driveFlight = false;
let driveQueued = null;
const vectors = {
  forward: [1, 0, 0], backward: [-1, 0, 0], left: [0, 1, 0], right: [0, -1, 0],
  ccw: [0, 0, 1], cw: [0, 0, -1],
};
let state = {};
let pending = null;
let stopping = false;
let fetchingPorts = false;
let issueOrder = 0;
let appliedOrder = 0;
let sessionOwned = false;
let localMotion = null;
let transientMessage = "";
let transientIsError = false;
let transientUntil = 0;
let previousServerMessage = "";

async function request(path, payload, options = {}) {
  const headers = { "X-Control-Token": token };
  if (payload !== undefined) headers["Content-Type"] = "application/json";
  const response = await fetch("/api/" + path, {
    method: payload === undefined ? "GET" : "POST",
    headers,
    cache: "no-store",
    body: payload === undefined ? undefined : JSON.stringify(payload),
    ...options,
  });
  const data = await response.json();
  if (!response.ok) {
    const error = new Error(data.error || "控制服务请求失败");
    error.rejected = true;
    throw error;
  }
  return data;
}

function feedback(message, isError = false) {
  transientMessage = message;
  transientIsError = isError;
  transientUntil = performance.now() + 4000;
  render();
}

function applyState(next, order) {
  if (order < appliedOrder) return;
  appliedOrder = order;
  if (next.message && next.message !== previousServerMessage && !transientIsError) {
    transientUntil = 0;
  }
  previousServerMessage = next.message;
  state = next;
  if (localMotion && order > localMotion.acknowledgedAt) checkMotionComplete(next);
  if (!next.armed && pending !== "arm") sessionOwned = false;
  if (!next.armed || !next.fresh) clearDriveKeys();
  render();
}

function checkMotionComplete(snapshot) {
  if (snapshot.fresh && (!snapshot.armed ||
      (snapshot.mode === "idle" && (snapshot.state === 2 || snapshot.can_release) && !snapshot.pending))) localMotion = null;
}

async function motionRequest(path, payload, order) {
  const previous = localMotion;
  const motion = { acknowledgedAt: Infinity };
  localMotion = motion;
  try {
    const next = await request(path, payload);
    if (localMotion === motion) {
      motion.acknowledgedAt = issueOrder;
      sessionOwned = next.armed;
      checkMotionComplete(next);
    }
    applyState(next, order);
  } catch (error) {
    if (localMotion === motion) {
      if (error.rejected) localMotion = previous;
      else motion.acknowledgedAt = issueOrder;
    }
    throw error;
  }
}

function render() {
  const connected = Boolean(state.connected);
  const fresh = connected && state.fresh;
  const idle = state.mode === "idle";
  const waiting = pending || Boolean(state.pending);
  const status = byId("device-state");
  const label = byId("state-text");
  let stateText = "未连接";
  let tone = "neutral";
  if (connected && !fresh) {
    stateText = "等待串口状态";
    tone = "warning";
  } else if (fresh) {
    stateText = stateLabels[state.state] || "状态未知";
    if (state.state === 0 && !state.disarm_confirmed) {
      stateText = "软件失能，待电机确认";
      tone = "warning";
    }
    if (state.state === 2) tone = "ready";
    if (state.state === 3 || state.state === 5) tone = "active";
    if (state.state === 4 || state.state === 6) tone = "warning";
    if (state.mode === "timed") { stateText = "定时运行中"; tone = "active"; }
    if (state.mode === "keys") { stateText = "按键遥控中"; tone = "active"; }
  }
  label.textContent = stateText;
  status.dataset.tone = tone;
  byId("serial-caption").textContent = state.port ? state.port + " · 115200 · 8N1" : "115200 · 8N1";
  portSelect.disabled = connected || fetchingPorts || pending;
  controls.refresh.disabled = connected || fetchingPorts || pending;
  controls.connect.textContent = connected ? "断开" : "连接";
  controls.connect.disabled = pending || (!connected && (fetchingPorts || !portSelect.value));
  controls.arm.textContent = state.armed ? "归还控制" : "接管底盘";
  controls.arm.disabled = waiting || stopping || !(state.can_arm || state.can_release) || !idle;
  if (fresh) {
    for (const [input, limit] of [
      [controls.linear, state.linear_limit], [controls.returnLinear, state.linear_limit],
      [controls.angular, state.angular_limit], [controls.returnAngular, state.angular_limit],
    ]) {
      input.max = limit;
      if (input.valueAsNumber > limit) input.value = limit;
    }
  }
  controls.run.disabled = waiting || stopping || !state.armed || !fresh || ![2, 3].includes(state.state) || !idle;
  controls.origin.disabled = controls.run.disabled || state.state !== 2 || !state.odom;
  controls.return.disabled = controls.origin.disabled || !state.origin;
  controls.stop.disabled = !connected || stopping;
  const locked = connected && !idle && state.mode !== "keys";
  const modeLocked = ["timed", "return"].includes(state.mode);
  directionSet.disabled = locked;
  controls.linear.disabled = locked;
  controls.angular.disabled = locked;
  controls.duration.disabled = locked;
  controls.modeTimed.disabled = modeLocked;
  controls.modeKeys.disabled = modeLocked;
  controls.returnLinear.disabled = state.mode === "return";
  controls.returnAngular.disabled = state.mode === "return";
  form.hidden = inputMode !== "timed";
  keyPad.hidden = inputMode !== "keys";
  controls.modeTimed.setAttribute("aria-pressed", String(inputMode === "timed"));
  controls.modeKeys.setAttribute("aria-pressed", String(inputMode === "keys"));
  const remaining = byId("timer");
  const hasTime = typeof state.remaining === "number" && Number.isFinite(state.remaining);
  remaining.textContent = hasTime ? Math.max(0, state.remaining).toFixed(1) + " s" : "—";
  remaining.dataset.active = String(hasTime);
  const odometryReady = fresh && Boolean(state.odom);
  const originReady = fresh && Boolean(state.origin);
  const odomStatus = byId("odom-status");
  const originStatus = byId("origin-status");
  odomStatus.textContent = odometryReady ? "里程计有效" : "里程计未就绪";
  originStatus.textContent = originReady ? "原点已建立" : "原点未建立";
  odomStatus.dataset.ready = String(odometryReady);
  originStatus.dataset.ready = String(originReady);
  const pose = odometryReady && originReady ? state.pose : null;
  byId("pose-x").textContent = Number.isFinite(pose?.x) ? pose.x.toFixed(3) : "—";
  byId("pose-y").textContent = Number.isFinite(pose?.y) ? pose.y.toFixed(3) : "—";
  byId("pose-yaw").textContent = Number.isFinite(pose?.yaw) ? pose.yaw.toFixed(2) : "—";
  const notification = byId("feedback");
  const showTransient = performance.now() < transientUntil;
  const message = (showTransient ? transientMessage : state.message) || (connected ? stateText : "请选择串口。");
  if (notification.textContent !== message) notification.textContent = message;
  notification.dataset.error = String(showTransient && transientIsError);
}

async function scanPorts() {
  if (fetchingPorts || state.connected) return;
  fetchingPorts = true;
  const selected = portSelect.value;
  render();
  try {
    const { ports } = await request("ports");
    portSelect.replaceChildren();
    if (ports.length === 0) {
      portSelect.add(new Option("未找到串口", ""));
    } else {
      for (const port of ports) portSelect.add(new Option(port.device, port.device));
      if (ports.some(({ device }) => device === selected)) portSelect.value = selected;
      else if (ports.some(({ device }) => device === "COM9")) portSelect.value = "COM9";
    }
  } catch (error) {
    portSelect.replaceChildren(new Option("无法读取串口", ""));
    feedback(error.message, true);
  } finally {
    fetchingPorts = false;
    render();
  }
}

async function action(path, payload = {}) {
  if (pending || stopping) return;
  pending = path;
  transientIsError = false;
  transientUntil = 0;
  const order = ++issueOrder;
  render();
  try {
    if (path === "run" || path === "return") {
      await motionRequest(path, payload, order);
    } else {
      const next = await request(path, payload, { keepalive: path === "release" });
      if (path === "arm" && next.armed) sessionOwned = true;
      if (path === "release" || path === "disconnect") sessionOwned = false;
      applyState(next, order);
    }
  } catch (error) {
    if (order >= appliedOrder) feedback(error.message, true);
  } finally {
    pending = null;
    render();
  }
}

async function stopMotion() {
  if (stopping || (!state.connected && !pending)) return;
  localMotion = null;
  sessionOwned = false;
  clearDriveKeys();
  stopping = true;
  const order = ++issueOrder;
  render();
  try {
    applyState(await request("stop", {}, { keepalive: true }), order);
  } catch (error) {
    if (!document.hidden) feedback(error.message, true);
  } finally {
    stopping = false;
    render();
  }
}

async function poll() {
  if (!document.hidden && document.hasFocus()) {
    const order = ++issueOrder;
    try {
      applyState(await request("state", undefined, { signal: AbortSignal.timeout(600) }), order);
    } catch {
      if (order >= appliedOrder) {
        appliedOrder = order;
        state = { ...state, fresh: false };
        if (state.connected) feedback("控制服务暂不可用", true);
      }
    }
    if (inputMode === "keys" && heldKeys().size > 0) publishDrive();
  }
  window.setTimeout(poll, 100);
}

portSelect.addEventListener("change", render);
controls.refresh.addEventListener("click", scanPorts);
controls.connect.addEventListener("click", () => {
  if (state.connected) action("disconnect");
  else if (portSelect.value) action("connect", { port: portSelect.value });
});
function heldKeys() {
  return new Set([...keyboardKeys, ...pointerKeys]);
}

function syncKeys() {
  for (const button of keyPad.querySelectorAll("[data-key]")) {
    button.dataset.held = String(heldKeys().has(button.dataset.key));
  }
}

function canDrive() {
  return inputMode === "keys" && state.armed && state.fresh && !state.pending && !stopping &&
    (state.mode === "idle" || state.mode === "keys") && (state.state === 2 || state.state === 3);
}

function postDrive(keys) {
  driveQueued = keys;
  if (driveFlight) return;
  driveFlight = true;
  const send = async () => {
    while (driveQueued !== null) {
      const keysNow = driveQueued;
      driveQueued = null;
      const order = ++issueOrder;
      try {
        await motionRequest("drive", {
          keys: keysNow,
          linear: controls.linear.valueAsNumber,
          angular: controls.angular.valueAsNumber,
        }, order);
      } catch (error) {
        if (order >= appliedOrder) feedback(error.message, true);
      }
    }
    driveFlight = false;
  };
  send();
}

function publishDrive() {
  const keys = [...heldKeys()];
  if (keys.length > 0) {
    if (!canDrive()) return;
    driving = true;
    postDrive(keys);
    return;
  }
  if (!driving) return;
  driving = false;
  // 松键时首个运动请求可能还未回执，零速仍须排在它后面。
  postDrive([]);
}

function clearDriveKeys() {
  keyboardKeys.clear();
  pointerKeys.clear();
  driveQueued = null;
  driving = false;
  syncKeys();
}

function setInputMode(mode) {
  if (mode === inputMode || ["timed", "return"].includes(state.mode)) return;
  if (inputMode === "keys") {
    const wasDriving = driving;
    clearDriveKeys();
    if (wasDriving) postDrive([]);
  }
  inputMode = mode;
  render();
}

controls.modeTimed.addEventListener("click", () => setInputMode("timed"));
controls.modeKeys.addEventListener("click", () => setInputMode("keys"));
controls.arm.addEventListener("click", () => action(state.armed ? "release" : "arm"));
controls.origin.addEventListener("click", () => action("origin"));
controls.return.addEventListener("click", () => {
  const linear = controls.returnLinear.valueAsNumber;
  const angular = controls.returnAngular.valueAsNumber;
  if (!(linear > 0) || !(angular > 0)) {
    feedback("返航平移和转向速度都必须大于零。", true);
    return;
  }
  action("return", { linear, angular });
});
controls.stop.addEventListener("click", stopMotion);
for (const button of keyPad.querySelectorAll("[data-key]")) {
  const key = button.dataset.key;
  const press = (event) => {
    if (event.button !== 0 || !canDrive()) return;
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    pointerKeys.add(key);
    syncKeys();
    publishDrive();
  };
  const release = () => {
    if (!pointerKeys.delete(key)) return;
    syncKeys();
    publishDrive();
  };
  button.addEventListener("pointerdown", press);
  button.addEventListener("pointerup", release);
  button.addEventListener("pointercancel", release);
  button.addEventListener("lostpointercapture", release);
}
form.addEventListener("submit", (event) => {
  event.preventDefault();
  if (controls.run.disabled || !form.reportValidity()) return;
  const [x, y, yaw] = vectors[form.elements.direction.value];
  const speed = controls.linear.valueAsNumber;
  const angular = controls.angular.valueAsNumber;
  const seconds = controls.duration.valueAsNumber;
  if (!(seconds >= 0.1)) {
    feedback("运行时间至少为 0.1 秒。", true);
    controls.duration.focus();
    return;
  }
  if (yaw === 0 ? !(speed > 0) : !(angular > 0)) {
    feedback("所选方向的速度必须大于零。", true);
    (yaw === 0 ? controls.linear : controls.angular).focus();
    return;
  }
  action("run", { vx: x * speed, vy: y * speed, wz: yaw * angular, seconds });
});
document.addEventListener("keydown", (event) => {
  if (event.repeat || event.ctrlKey || event.altKey || event.metaKey) return;
  if (event.code === "Space" && state.connected) {
    event.preventDefault();
    stopMotion();
    return;
  }
  if (event.target.closest("input, select, textarea, [contenteditable]")) return;
  const key = event.key.toLowerCase();
  if ("wasdqe".includes(key) && key.length === 1 && canDrive()) {
    event.preventDefault();
    keyboardKeys.add(key);
    syncKeys();
    publishDrive();
  } else if (key === "o" && !controls.origin.disabled) {
    action("origin");
  } else if (key === "r" && !controls.return.disabled) {
    controls.return.click();
  }
});
document.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!keyboardKeys.delete(key)) return;
  syncKeys();
  publishDrive();
});
function stopOwnedMotion() {
  if (localMotion) stopMotion();
  else if (sessionOwned && state.can_release && !pending) action("release");
}
window.addEventListener("pagehide", stopOwnedMotion);
window.addEventListener("blur", stopOwnedMotion);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopOwnedMotion();
});

render();
scanPorts();
poll();
