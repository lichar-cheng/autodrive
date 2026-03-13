let ws;
let pingTimer = null;
let reconnectTimer = null;
let reconnectAttempts = 0;
let driveTimer = null;
let healthTimer = null;
let connectGeneration = 0;
let suppressReconnect = false;
let messageWatchdogTimer = null;
let lastMessageAtMs = 0;

let lastPose = { x: 0, y: 0, yaw: 0, vx: 0, wz: 0 };
let lastGps = { lat: 0, lon: 0 };
let lastChassis = { wheel_speed_l: 0, wheel_speed_r: 0, battery: 0, mode: "-" };
let pathNodes = [];
let poiNodes = [];
let cameraInbox = new Map();
let cameraDisplay = new Map();

const scanSession = {
  active: false,
  startedAtMs: 0,
  frontFrames: 0,
  rearFrames: 0,
  totalLivePoints: 0,
  occupiedCells: new Map(),
  freeCells: new Map(),
  lastSavedFile: "",
  savedPointCount: 0,
  voxelSize: 0.12,
};

const streamHealth = {
  msgTotal: 0,
  checksumErr: 0,
  checksumSkipped: 0,
  staleTsErr: 0,
  gapErr: 0,
  lastSeq: {},
  lastLagMs: 0,
  retriesHttp: 0,
  lastApiError: "",
  lastHealth: null,
};

const drawState = {
  pathMode: "both",
  showPath: true,
  showPoi: true,
  showRobot: true,
};

const viewState = {
  scale: 25,
  panX: 0,
  panY: 0,
  dragging: false,
  moved: false,
  downX: 0,
  downY: 0,
  originPanX: 0,
  originPanY: 0,
};

const keyState = new Set();

const canvas = document.getElementById("lidarCanvas");
const ctx = canvas.getContext("2d");

const elements = {
  posePanel: document.getElementById("posePanel"),
  pathList: document.getElementById("pathList"),
  cameraGrid: document.getElementById("cameraGrid"),
  gpsPanel: document.getElementById("gpsPanel"),
  chassisPanel: document.getElementById("chassisPanel"),
  commPanel: document.getElementById("commPanel"),
  scanPanel: document.getElementById("scanPanel"),
  status: document.getElementById("status"),
  statusDetail: document.getElementById("statusDetail"),
  scanState: document.getElementById("scanState"),
  keyboardState: document.getElementById("keyboardState"),
  serverUrl: document.getElementById("serverUrl"),
  mapNameInput: document.getElementById("mapNameInput"),
  mapNotesInput: document.getElementById("mapNotesInput"),
  voxelSizeInput: document.getElementById("voxelSizeInput"),
  forwardSpeedInput: document.getElementById("forwardSpeedInput"),
  reverseSpeedInput: document.getElementById("reverseSpeedInput"),
  turnRateInput: document.getElementById("turnRateInput"),
  cmdDurationInput: document.getElementById("cmdDurationInput"),
  repeatMsInput: document.getElementById("repeatMsInput"),
  stopOnKeyupInput: document.getElementById("stopOnKeyupInput"),
  pathDrawMode: document.getElementById("pathDrawMode"),
  showPathToggle: document.getElementById("showPathToggle"),
  showPoiToggle: document.getElementById("showPoiToggle"),
  showRobotToggle: document.getElementById("showRobotToggle"),
  drawModeBadge: document.getElementById("drawModeBadge"),
  refreshCameraBtn: document.getElementById("refreshCameraBtn"),
  cameraRefreshStatus: document.getElementById("cameraRefreshStatus"),
  viewMetrics: document.getElementById("viewMetrics"),
  connectBtn: document.getElementById("connectBtn"),
};

const topicHandler = {
  "/robot/pose": (msg) => {
    lastPose = msg.payload;
    elements.posePanel.innerText = JSON.stringify(lastPose, null, 2);
  },
  "/robot/gps": (msg) => {
    lastGps = msg.payload;
    elements.gpsPanel.innerText = JSON.stringify(lastGps, null, 2);
  },
  "/chassis/status": (msg) => {
    lastChassis = msg.payload;
    elements.chassisPanel.innerText = JSON.stringify(lastChassis, null, 2);
    renderScanState();
  },
  "/chassis/odom": () => {},
  "/lidar/front": (msg) => {
    scanSession.frontFrames += 1;
    accumulatePoints(msg.payload.points || [], "front");
  },
  "/lidar/rear": (msg) => {
    scanSession.rearFrames += 1;
    accumulatePoints(msg.payload.points || [], "rear");
  },
  "/map/grid": () => {},
};

for (let i = 1; i <= 4; i += 1) {
  topicHandler[`/camera/${i}/compressed`] = (msg) => {
    cameraInbox.set(i, {
      objects: msg.payload.objects || [],
      receivedAtMs: Date.now(),
      serverStamp: msg.stamp,
      seq: msg.seq,
    });
    updateCameraRefreshStatus();
  };
}

function stableStringify(obj) {
  if (obj === null || typeof obj !== "object") {
    return JSON.stringify(obj);
  }
  if (Array.isArray(obj)) {
    return `[${obj.map(stableStringify).join(",")}]`;
  }
  const keys = Object.keys(obj).sort();
  return `{${keys.map((key) => `${JSON.stringify(key)}:${stableStringify(obj[key])}`).join(",")}}`;
}

async function sha256Hex(text) {
  const enc = new TextEncoder().encode(text);
  const buf = await crypto.subtle.digest("SHA-256", enc);
  return [...new Uint8Array(buf)].map((x) => x.toString(16).padStart(2, "0")).join("");
}

async function validateMessage(msg) {
  streamHealth.msgTotal += 1;
  const shouldSkipChecksum = msg.topic.startsWith("/lidar/") || msg.topic === "/map/grid";
  if (shouldSkipChecksum) {
    streamHealth.checksumSkipped += 1;
  } else {
    const basis = `${msg.topic}|${Number(msg.stamp).toFixed(6)}|${msg.seq}|${stableStringify(msg.payload)}`;
    const digest = await sha256Hex(basis);
    if (digest !== msg.checksum) {
      streamHealth.checksumErr += 1;
    }
  }

  const lagMs = Date.now() - Number(msg.server_time_ms || Date.now());
  streamHealth.lastLagMs = lagMs;
  if (Math.abs(lagMs) > 5000) {
    streamHealth.staleTsErr += 1;
  }

  const last = streamHealth.lastSeq[msg.topic];
  if (typeof last === "number" && msg.seq > last + 1) {
    streamHealth.gapErr += msg.seq - last - 1;
  }
  streamHealth.lastSeq[msg.topic] = msg.seq;
}

function setStatus(text, level = "") {
  elements.status.innerText = text;
  elements.status.className = `status-chip ${level}`.trim();
  canvas.classList.remove("ws-connected", "ws-warning", "ws-error");
  if (level === "connected") {
    canvas.classList.add("ws-connected");
  } else if (level === "warning") {
    canvas.classList.add("ws-warning");
  } else if (level === "error") {
    canvas.classList.add("ws-error");
  }
  elements.connectBtn.textContent = level === "connected" ? "Disconnect" : "Connect";
}

function setScanState(text, level = "") {
  elements.scanState.innerText = text;
  elements.scanState.className = `scan-state ${level}`.trim();
}

function setKeyboardState(text, level = "") {
  elements.keyboardState.innerText = text;
  elements.keyboardState.className = `kbd-state ${level}`.trim();
}

function wsToHttpBase(url) {
  return url.replace(/^ws/, "http").replace(/\/ws\/stream$/, "");
}

async function callApi(path, body = {}, retries = 3) {
  const base = wsToHttpBase(elements.serverUrl.value.trim());
  let lastErr;
  for (let i = 0; i <= retries; i += 1) {
    try {
      const res = await fetch(`${base}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      if (!res.ok) {
        throw new Error(`HTTP ${res.status}`);
      }
      streamHealth.lastApiError = "";
      return await res.json();
    } catch (err) {
      lastErr = err;
      streamHealth.lastApiError = err.message;
      if (i < retries) {
        streamHealth.retriesHttp += 1;
        await new Promise((resolve) => setTimeout(resolve, 200 * (2 ** i)));
      }
    }
  }
  throw lastErr;
}

async function fetchJson(path) {
  const base = wsToHttpBase(elements.serverUrl.value.trim());
  const res = await fetch(`${base}${path}`);
  if (!res.ok) {
    throw new Error(`HTTP ${res.status}`);
  }
  return res.json();
}

function scheduleReconnect() {
  if (reconnectTimer) {
    return;
  }
  if (suppressReconnect) {
    return;
  }
  reconnectAttempts += 1;
  const backoff = Math.min(10000, Math.round((2 ** Math.min(reconnectAttempts, 6)) * 200 + Math.random() * 300));
  setStatus(`Reconnect in ${backoff} ms`, "warning");
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connect();
  }, backoff);
}

function startPing() {
  clearInterval(pingTimer);
  pingTimer = setInterval(() => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send("ping");
    }
  }, 5000);
}

function stopPing() {
  clearInterval(pingTimer);
  pingTimer = null;
}

function startHealthPolling() {
  clearInterval(healthTimer);
  healthTimer = setInterval(async () => {
    try {
      streamHealth.lastHealth = await fetchJson("/health");
    } catch (err) {
      streamHealth.lastApiError = err.message;
    }
  }, 3000);
}

function stopHealthPolling() {
  clearInterval(healthTimer);
  healthTimer = null;
}

function startMessageWatchdog(generation) {
  clearInterval(messageWatchdogTimer);
  lastMessageAtMs = Date.now();
  messageWatchdogTimer = setInterval(() => {
    if (generation !== connectGeneration) {
      clearInterval(messageWatchdogTimer);
      messageWatchdogTimer = null;
      return;
    }
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return;
    }
    const idleMs = Date.now() - lastMessageAtMs;
    if (idleMs > 12000) {
      elements.statusDetail.innerText = `WS stalled ${Math.round(idleMs / 1000)}s, reconnecting`;
      suppressReconnect = false;
      try {
        ws.close(4000, "watchdog_timeout");
      } catch (err) {
        console.error("watchdog close failed", err);
      }
    }
  }, 2000);
}

function stopMessageWatchdog() {
  clearInterval(messageWatchdogTimer);
  messageWatchdogTimer = null;
}

function disconnectSocket(reason = "manual") {
  clearTimeout(reconnectTimer);
  reconnectTimer = null;
  stopPing();
  stopHealthPolling();
  stopMessageWatchdog();
  suppressReconnect = true;
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    try {
      ws.close(1000, reason);
    } catch (err) {
      console.error("ws close failed", err);
    }
  }
}

function connect() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    disconnectSocket("manual_disconnect");
    setStatus("Disconnected", "warning");
    elements.statusDetail.innerText = "WS manually disconnected";
    ws = null;
    return;
  }
  const url = elements.serverUrl.value.trim();
  if (!url) {
    setStatus("Missing WS URL", "error");
    elements.statusDetail.innerText = "WS URL empty";
    return;
  }
  clearTimeout(reconnectTimer);
  reconnectTimer = null;
  suppressReconnect = false;
  connectGeneration += 1;
  const generation = connectGeneration;
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    suppressReconnect = true;
    ws.close();
  }
  ws = new WebSocket(url);
  setStatus("Connecting...", "warning");
  elements.statusDetail.innerText = "WS connecting";

  ws.onopen = () => {
    if (generation !== connectGeneration) {
      ws.close();
      return;
    }
    suppressReconnect = false;
    reconnectAttempts = 0;
    setStatus("Connected", "connected");
    elements.statusDetail.innerText = "WS connected";
    startPing();
    startHealthPolling();
    startMessageWatchdog(generation);
  };

  ws.onclose = () => {
    if (generation !== connectGeneration) {
      return;
    }
    stopPing();
    stopHealthPolling();
    stopMessageWatchdog();
    if (suppressReconnect) {
      suppressReconnect = false;
      elements.statusDetail.innerText = "WS switched";
      return;
    }
    setStatus("Disconnected", "warning");
    elements.statusDetail.innerText = "WS closed, retrying";
    scheduleReconnect();
  };

  ws.onerror = () => {
    if (generation !== connectGeneration) {
      return;
    }
    setStatus("Connection error", "error");
    elements.statusDetail.innerText = "WS error";
  };

  ws.onmessage = async (event) => {
    if (generation !== connectGeneration) {
      return;
    }
    if (event.data === "pong") {
      lastMessageAtMs = Date.now();
      return;
    }
    lastMessageAtMs = Date.now();
    const msg = JSON.parse(event.data);
    await validateMessage(msg);
    if (topicHandler[msg.topic]) {
      topicHandler[msg.topic](msg);
    }
  };
}

function numberFromInput(el, fallback) {
  const value = Number(el.value);
  return Number.isFinite(value) ? value : fallback;
}

function readMotionConfig() {
  return {
    forwardSpeed: numberFromInput(elements.forwardSpeedInput, 0.8),
    reverseSpeed: numberFromInput(elements.reverseSpeedInput, 0.5),
    turnRate: numberFromInput(elements.turnRateInput, 1.0),
    cmdDuration: numberFromInput(elements.cmdDurationInput, 0.15),
    repeatMs: numberFromInput(elements.repeatMsInput, 120),
    stopOnKeyup: elements.stopOnKeyupInput.checked,
  };
}

function sendMove(v, w, d) {
  return callApi("/control/move", { velocity: v, yaw_rate: w, duration: d }).catch((err) => {
    streamHealth.lastApiError = err.message;
  });
}

function stopMove() {
  return callApi("/control/stop", {}).catch((err) => {
    streamHealth.lastApiError = err.message;
  });
}

function computeDriveCommand() {
  const cfg = readMotionConfig();
  let velocity = 0;
  let yawRate = 0;

  if (keyState.has("w") || keyState.has("arrowup")) {
    velocity += cfg.forwardSpeed;
  }
  if (keyState.has("s") || keyState.has("arrowdown")) {
    velocity -= cfg.reverseSpeed;
  }
  if (keyState.has("a") || keyState.has("arrowleft")) {
    yawRate += cfg.turnRate;
  }
  if (keyState.has("d") || keyState.has("arrowright")) {
    yawRate -= cfg.turnRate;
  }

  return { velocity, yawRate, cfg };
}

function tickKeyboardControl() {
  const { velocity, yawRate, cfg } = computeDriveCommand();
  if (velocity === 0 && yawRate === 0) {
    if (cfg.stopOnKeyup) {
      stopMove();
    }
    if (driveTimer) {
      clearInterval(driveTimer);
      driveTimer = null;
    }
    setKeyboardState("Keyboard idle");
    return;
  }

  const parts = [];
  if (velocity > 0) {
    parts.push(`forward ${velocity.toFixed(2)}`);
  }
  if (velocity < 0) {
    parts.push(`reverse ${Math.abs(velocity).toFixed(2)}`);
  }
  if (yawRate > 0) {
    parts.push(`left ${yawRate.toFixed(2)}`);
  }
  if (yawRate < 0) {
    parts.push(`right ${Math.abs(yawRate).toFixed(2)}`);
  }
  setKeyboardState(parts.join(" | "), "active");
  sendMove(velocity, yawRate, cfg.cmdDuration);
}

function ensureDriveLoop() {
  const { repeatMs } = readMotionConfig();
  if (driveTimer) {
    clearInterval(driveTimer);
  }
  driveTimer = setInterval(tickKeyboardControl, repeatMs);
}

function onKeyDown(event) {
  const target = event.target;
  const tagName = target && target.tagName ? target.tagName.toLowerCase() : "";
  if (tagName === "input" || tagName === "textarea" || tagName === "select") {
    return;
  }

  const key = event.key.toLowerCase();
  if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright", " "].includes(key)) {
    event.preventDefault();
  }

  if (key === " ") {
    keyState.clear();
    if (driveTimer) {
      clearInterval(driveTimer);
      driveTimer = null;
    }
    setKeyboardState("Emergency stop", "error");
    stopMove();
    return;
  }

  if (!["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(key)) {
    return;
  }

  keyState.add(key);
  ensureDriveLoop();
  tickKeyboardControl();
}

function onKeyUp(event) {
  const key = event.key.toLowerCase();
  keyState.delete(key);
  tickKeyboardControl();
}

function clearAccumulation() {
  scanSession.occupiedCells.clear();
  scanSession.freeCells.clear();
  scanSession.frontFrames = 0;
  scanSession.rearFrames = 0;
  scanSession.totalLivePoints = 0;
  scanSession.savedPointCount = 0;
  scanSession.lastSavedFile = "";
  renderScanState();
}

function startScanSession() {
  clearAccumulation();
  scanSession.active = true;
  scanSession.startedAtMs = Date.now();
  renderScanState();
}

function stopScanSession() {
  scanSession.active = false;
  renderScanState();
}

function worldToCell(x, y) {
  const size = scanSession.voxelSize;
  return [Math.round(x / size), Math.round(y / size)];
}

function cellKey(ix, iy) {
  return `${ix}:${iy}`;
}

function markFreeCell(ix, iy) {
  const key = cellKey(ix, iy);
  const slot = scanSession.freeCells.get(key) || { ix, iy, hits: 0 };
  slot.hits += 1;
  scanSession.freeCells.set(key, slot);
}

function markOccupiedCell(ix, iy, intensity) {
  const key = cellKey(ix, iy);
  const slot = scanSession.occupiedCells.get(key) || { ix, iy, hits: 0, intensity: 0 };
  slot.hits += 1;
  slot.intensity = Math.max(slot.intensity, intensity);
  scanSession.occupiedCells.set(key, slot);
}

function raytraceFreeCells(startX, startY, endX, endY) {
  const dx = endX - startX;
  const dy = endY - startY;
  const steps = Math.max(Math.abs(dx), Math.abs(dy));
  if (steps <= 1) {
    return;
  }
  for (let step = 0; step < steps; step += 1) {
    const t = step / steps;
    const ix = Math.round(startX + dx * t);
    const iy = Math.round(startY + dy * t);
    markFreeCell(ix, iy);
  }
}

function accumulatePoints(points) {
  if (!scanSession.active || !Array.isArray(points) || points.length === 0) {
    renderScanState();
    return;
  }

  scanSession.voxelSize = Math.max(0.02, numberFromInput(elements.voxelSizeInput, 0.12));
  scanSession.totalLivePoints += points.length;
  const [robotIx, robotIy] = worldToCell(lastPose.x, lastPose.y);

  for (const point of points) {
    const [x, y, intensity] = point;
    const [ix, iy] = worldToCell(x, y);
    raytraceFreeCells(robotIx, robotIy, ix, iy);
    markOccupiedCell(ix, iy, intensity);
  }

  renderScanState();
}

function occupiedPointsForSave() {
  return Array.from(scanSession.occupiedCells.values())
    .filter((cell) => cell.hits >= 2)
    .map((cell) => [
      cell.ix * scanSession.voxelSize,
      cell.iy * scanSession.voxelSize,
      Number(cell.intensity || 1),
    ]);
}

function renderScanState() {
  const elapsedSec = scanSession.startedAtMs ? Math.max(0, (Date.now() - scanSession.startedAtMs) / 1000) : 0;
  const occupiedCount = scanSession.occupiedCells.size;
  const freeCount = scanSession.freeCells.size;
  const payload = {
    scanActive: scanSession.active,
    chassisMode: lastChassis.mode || "-",
    elapsedSec: Number(elapsedSec.toFixed(1)),
    obstacleCells: occupiedCount,
    safeCells: freeCount,
    rawLidarPoints: scanSession.totalLivePoints,
    frontFrames: scanSession.frontFrames,
    rearFrames: scanSession.rearFrames,
    voxelSize: scanSession.voxelSize,
    lastSavedFile: scanSession.lastSavedFile || "-",
    lastSavedPointCount: scanSession.savedPointCount,
  };
  elements.scanPanel.innerText = JSON.stringify(payload, null, 2);

  if (scanSession.active) {
    setScanState(`Recording ${occupiedCount} obstacle cells`, "active");
    return;
  }
  if (occupiedCount > 0 || freeCount > 0) {
    setScanState(`Stopped | ${occupiedCount} obs / ${freeCount} safe`, "warning");
    return;
  }
  setScanState("Idle");
}

function resetPathList() {
  elements.pathList.innerHTML = "";
  for (const [index, node] of pathNodes.entries()) {
    const li = document.createElement("li");
    const lat = Number.isFinite(node.lat) ? node.lat.toFixed(6) : "n/a";
    const lon = Number.isFinite(node.lon) ? node.lon.toFixed(6) : "n/a";
    li.textContent = `${index + 1}. (${node.x.toFixed(2)}, ${node.y.toFixed(2)}) lat=${lat} lon=${lon}`;
    elements.pathList.appendChild(li);
  }
}

function formatTime(ms) {
  if (!ms) {
    return "n/a";
  }
  return new Date(ms).toLocaleTimeString("zh-CN", { hour12: false });
}

function updateCameraRefreshStatus() {
  const newest = Array.from(cameraInbox.values()).sort((a, b) => b.receivedAtMs - a.receivedAtMs)[0];
  if (!newest) {
    elements.cameraRefreshStatus.innerText = "No buffered frame";
    return;
  }
  elements.cameraRefreshStatus.innerText = `Buffered latest ${formatTime(newest.receivedAtMs)}`;
}

function refreshCameraSnapshot() {
  cameraDisplay = new Map(cameraInbox);
  const newest = Array.from(cameraDisplay.values()).sort((a, b) => b.receivedAtMs - a.receivedAtMs)[0];
  elements.cameraRefreshStatus.innerText = newest
    ? `Displayed snapshot ${formatTime(newest.receivedAtMs)}`
    : "No snapshot yet";
  renderCamera();
}

async function startScan() {
  await callApi("/scan/start", {});
  startScanSession();
}

async function stopScan() {
  await callApi("/scan/stop", {});
  stopScanSession();
}

async function saveMap() {
  const name = elements.mapNameInput.value.trim() || "demo_map";
  const voxelSize = Math.max(0.02, numberFromInput(elements.voxelSizeInput, 0.12));
  const pointsToSave = occupiedPointsForSave();
  const notes = {
    text: elements.mapNotesInput.value.trim(),
    browserObstacleCells: scanSession.occupiedCells.size,
    browserSafeCells: scanSession.freeCells.size,
    browserRawLidarPoints: scanSession.totalLivePoints,
    voxelSize,
    manualCameraSnapshotAt: elements.cameraRefreshStatus.innerText,
    clientObstaclePreview: pointsToSave.length,
  };
  const response = await callApi("/map/save", {
    name,
    notes: JSON.stringify(notes, null, 2),
    voxel_size: voxelSize,
    reset_after_save: false,
  });
  scanSession.lastSavedFile = response.file || "";
  scanSession.savedPointCount = response.contains ? response.contains.radar_points : 0;
  renderScanState();
  alert(`Map saved: ${response.file}`);
}

async function sendPath() {
  await callApi("/path/plan", { nodes: pathNodes });
}

function clearPath() {
  pathNodes = [];
  resetPathList();
}

function canvasToWorld(screenX, screenY) {
  const x = (screenX - canvas.width / 2 - viewState.panX) / viewState.scale;
  const y = (canvas.height / 2 + viewState.panY - screenY) / viewState.scale;
  return [x, y];
}

function worldToCanvas(worldX, worldY) {
  return [
    canvas.width / 2 + viewState.panX + worldX * viewState.scale,
    canvas.height / 2 + viewState.panY - worldY * viewState.scale,
  ];
}

function centerViewOnRobot() {
  viewState.panX = -lastPose.x * viewState.scale;
  viewState.panY = lastPose.y * viewState.scale;
  updateViewMetrics();
}

function resetView() {
  viewState.scale = 25;
  centerViewOnRobot();
}

function updateViewMetrics() {
  const panWorldX = (-viewState.panX / viewState.scale).toFixed(2);
  const panWorldY = (viewState.panY / viewState.scale).toFixed(2);
  elements.viewMetrics.innerText = `Pan ${panWorldX}, ${panWorldY} | Zoom ${viewState.scale.toFixed(1)} px/m`;
}

function handleCanvasClick(clientX, clientY, shiftKey) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const [x, y] = canvasToWorld((clientX - rect.left) * scaleX, (clientY - rect.top) * scaleY);

  if (shiftKey) {
    const poi = { name: `POI-${poiNodes.length + 1}`, x, y, lat: lastGps.lat, lon: lastGps.lon };
    poiNodes.push(poi);
    callApi("/map/poi", { poi }).catch((err) => {
      alert(`POI failed: ${err.message}`);
    });
    return;
  }

  pathNodes.push({ x, y, lat: lastGps.lat, lon: lastGps.lon });
  resetPathList();
}

function bindCanvasInteractions() {
  canvas.addEventListener("pointerdown", (event) => {
    const rect = canvas.getBoundingClientRect();
    canvas.setPointerCapture(event.pointerId);
    viewState.dragging = true;
    viewState.moved = false;
    viewState.downX = event.clientX - rect.left;
    viewState.downY = event.clientY - rect.top;
    viewState.originPanX = viewState.panX;
    viewState.originPanY = viewState.panY;
    canvas.classList.add("dragging");
  });

  canvas.addEventListener("pointermove", (event) => {
    if (!viewState.dragging) {
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const dx = event.clientX - rect.left - viewState.downX;
    const dy = event.clientY - rect.top - viewState.downY;
    if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
      viewState.moved = true;
    }
    viewState.panX = viewState.originPanX + dx;
    viewState.panY = viewState.originPanY + dy;
    updateViewMetrics();
  });

  canvas.addEventListener("pointerup", (event) => {
    if (!viewState.dragging) {
      return;
    }
    canvas.releasePointerCapture(event.pointerId);
    canvas.classList.remove("dragging");
    const wasMoved = viewState.moved;
    viewState.dragging = false;
    if (!wasMoved) {
      handleCanvasClick(event.clientX, event.clientY, event.shiftKey);
    }
  });

  canvas.addEventListener("pointerleave", () => {
    if (!viewState.dragging) {
      canvas.classList.remove("dragging");
    }
  });

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const pointerX = (event.clientX - rect.left) * scaleX;
    const pointerY = (event.clientY - rect.top) * scaleY;
    const [worldX, worldY] = canvasToWorld(pointerX, pointerY);
    const nextScale = Math.min(80, Math.max(8, viewState.scale * (event.deltaY < 0 ? 1.08 : 0.92)));
    viewState.scale = nextScale;
    const [newCanvasX, newCanvasY] = worldToCanvas(worldX, worldY);
    viewState.panX += pointerX - newCanvasX;
    viewState.panY += pointerY - newCanvasY;
    updateViewMetrics();
  }, { passive: false });
}

function drawGrid() {
  const spacingWorld = 2;
  const spacingPx = spacingWorld * viewState.scale;
  if (spacingPx < 20) {
    return;
  }

  ctx.strokeStyle = "rgba(255, 255, 255, 0.12)";
  ctx.lineWidth = 1;

  const worldLeft = canvasToWorld(0, canvas.height)[0];
  const worldRight = canvasToWorld(canvas.width, 0)[0];
  const worldBottom = canvasToWorld(0, canvas.height)[1];
  const worldTop = canvasToWorld(canvas.width, 0)[1];

  const startX = Math.floor(worldLeft / spacingWorld) * spacingWorld;
  const endX = Math.ceil(worldRight / spacingWorld) * spacingWorld;
  const startY = Math.floor(worldBottom / spacingWorld) * spacingWorld;
  const endY = Math.ceil(worldTop / spacingWorld) * spacingWorld;

  for (let x = startX; x <= endX; x += spacingWorld) {
    const [sx1, sy1] = worldToCanvas(x, startY);
    const [sx2, sy2] = worldToCanvas(x, endY);
    ctx.beginPath();
    ctx.moveTo(sx1, sy1);
    ctx.lineTo(sx2, sy2);
    ctx.stroke();
  }

  for (let y = startY; y <= endY; y += spacingWorld) {
    const [sx1, sy1] = worldToCanvas(startX, y);
    const [sx2, sy2] = worldToCanvas(endX, y);
    ctx.beginPath();
    ctx.moveTo(sx1, sy1);
    ctx.lineTo(sx2, sy2);
    ctx.stroke();
  }
}

function drawOccupancy() {
  const sizePx = Math.max(2, scanSession.voxelSize * viewState.scale);

  for (const cell of scanSession.freeCells.values()) {
    const occ = scanSession.occupiedCells.get(cellKey(cell.ix, cell.iy));
    const freeWeight = cell.hits;
    const occWeight = occ ? occ.hits : 0;
    if (freeWeight <= occWeight * 0.8) {
      continue;
    }
    const worldX = cell.ix * scanSession.voxelSize;
    const worldY = cell.iy * scanSession.voxelSize;
    const [sx, sy] = worldToCanvas(worldX, worldY);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(sx - sizePx / 2, sy - sizePx / 2, sizePx, sizePx);
  }

  for (const cell of scanSession.occupiedCells.values()) {
    const free = scanSession.freeCells.get(cellKey(cell.ix, cell.iy));
    const freeWeight = free ? free.hits : 0;
    if (cell.hits < 2 || cell.hits < freeWeight * 0.9) {
      continue;
    }
    const worldX = cell.ix * scanSession.voxelSize;
    const worldY = cell.iy * scanSession.voxelSize;
    const [sx, sy] = worldToCanvas(worldX, worldY);
    ctx.fillStyle = "#0c0f12";
    ctx.fillRect(sx - sizePx / 2, sy - sizePx / 2, sizePx, sizePx);
  }
}

function drawPathOverlay() {
  if (!drawState.showPath || !pathNodes.length) {
    return;
  }
  const mode = drawState.pathMode;

  if (mode === "both" || mode === "lines") {
    ctx.strokeStyle = "#f3b441";
    ctx.lineWidth = 2;
    ctx.beginPath();
    pathNodes.forEach((node, index) => {
      const [sx, sy] = worldToCanvas(node.x, node.y);
      if (index === 0) {
        ctx.moveTo(sx, sy);
      } else {
        ctx.lineTo(sx, sy);
      }
    });
    ctx.stroke();
  }

  if (mode === "both" || mode === "points") {
    ctx.fillStyle = "#f3b441";
    pathNodes.forEach((node) => {
      const [sx, sy] = worldToCanvas(node.x, node.y);
      ctx.beginPath();
      ctx.arc(sx, sy, 4, 0, Math.PI * 2);
      ctx.fill();
    });
  }
}

function drawPoiOverlay() {
  if (!drawState.showPoi) {
    return;
  }
  ctx.fillStyle = "#d94a4a";
  poiNodes.forEach((poi) => {
    const [sx, sy] = worldToCanvas(poi.x, poi.y);
    ctx.beginPath();
    ctx.arc(sx, sy, 5, 0, Math.PI * 2);
    ctx.fill();
  });
}

function drawRobot() {
  if (!drawState.showRobot) {
    return;
  }
  const [x, y] = worldToCanvas(lastPose.x, lastPose.y);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-lastPose.yaw);
  ctx.fillStyle = "#177f7c";
  ctx.fillRect(-10, -6, 20, 12);
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(14, 0);
  ctx.stroke();
  ctx.restore();
}

function drawComm() {
  const comm = {
    wsState: ws ? ws.readyState : -1,
    reconnectAttempts,
    msgTotal: streamHealth.msgTotal,
    checksumErr: streamHealth.checksumErr,
    checksumSkipped: streamHealth.checksumSkipped,
    staleTsErr: streamHealth.staleTsErr,
    gapErr: streamHealth.gapErr,
    lagMs: streamHealth.lastLagMs,
    httpRetries: streamHealth.retriesHttp,
    lastApiError: streamHealth.lastApiError || "-",
    wsClients: streamHealth.lastHealth ? streamHealth.lastHealth.ws_clients : "n/a",
    serverScanActive: streamHealth.lastHealth ? streamHealth.lastHealth.scan_active : "n/a",
  };
  elements.commPanel.innerText = JSON.stringify(comm, null, 2);
  if (streamHealth.lastHealth) {
    elements.statusDetail.innerText = `WS ok | clients ${streamHealth.lastHealth.ws_clients} | scan ${streamHealth.lastHealth.scan_active ? "on" : "off"} | ros ${streamHealth.lastHealth.ros_enabled ? "enabled" : "detected only"}`;
  }
}

function renderCamera() {
  elements.cameraGrid.innerHTML = "";
  for (let i = 1; i <= 4; i += 1) {
    const camera = cameraDisplay.get(i);
    const card = document.createElement("div");
    card.className = "camera-card";
    if (!camera) {
      card.innerHTML = `<strong>Camera ${i}</strong><div>No manual snapshot</div>`;
      elements.cameraGrid.appendChild(card);
      continue;
    }
    const detail = camera.objects.map((obj) => `${obj.label} (${obj.confidence})`).join("<br/>") || "No detections";
    card.innerHTML = `
      <strong>Camera ${i}</strong>
      <div>${detail}</div>
      <time>Latest: ${formatTime(camera.receivedAtMs)} | seq ${camera.seq}</time>
    `;
    elements.cameraGrid.appendChild(card);
  }
}

function syncDrawControls() {
  drawState.pathMode = elements.pathDrawMode.value;
  drawState.showPath = elements.showPathToggle.checked;
  drawState.showPoi = elements.showPoiToggle.checked;
  drawState.showRobot = elements.showRobotToggle.checked;
  const modeLabel = {
    points: "Only Points",
    both: "Point + Line",
    lines: "Only Lines",
  }[drawState.pathMode];
  elements.drawModeBadge.innerText = modeLabel;
}

function draw() {
  ctx.fillStyle = "#8f969c";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawOccupancy();
  drawPathOverlay();
  drawPoiOverlay();
  drawRobot();
  drawComm();
  requestAnimationFrame(draw);
}

function bindUi() {
  document.getElementById("connectBtn").onclick = connect;
  document.getElementById("startScanBtn").onclick = () => startScan().catch((err) => alert(err.message));
  document.getElementById("stopScanBtn").onclick = () => stopScan().catch((err) => alert(err.message));
  document.getElementById("clearAccumBtn").onclick = clearAccumulation;
  document.getElementById("saveMapBtn").onclick = () => saveMap().catch((err) => alert(`Save failed: ${err.message}`));
  document.getElementById("sendPathBtn").onclick = () => sendPath().catch((err) => alert(`Send path failed: ${err.message}`));
  document.getElementById("clearPathBtn").onclick = clearPath;
  document.getElementById("centerViewBtn").onclick = centerViewOnRobot;
  document.getElementById("resetViewBtn").onclick = resetView;
  elements.refreshCameraBtn.onclick = refreshCameraSnapshot;

  elements.pathDrawMode.addEventListener("change", syncDrawControls);
  elements.showPathToggle.addEventListener("change", syncDrawControls);
  elements.showPoiToggle.addEventListener("change", syncDrawControls);
  elements.showRobotToggle.addEventListener("change", syncDrawControls);

  document.querySelectorAll("[data-move]").forEach((button) => {
    button.addEventListener("click", () => {
      const cfg = readMotionConfig();
      const move = button.getAttribute("data-move");
      if (move === "forward") {
        sendMove(cfg.forwardSpeed, 0, cfg.cmdDuration);
      } else if (move === "backward") {
        sendMove(-cfg.reverseSpeed, 0, cfg.cmdDuration);
      } else if (move === "left") {
        sendMove(Math.max(cfg.forwardSpeed * 0.5, 0.2), cfg.turnRate, cfg.cmdDuration);
      } else if (move === "right") {
        sendMove(Math.max(cfg.forwardSpeed * 0.5, 0.2), -cfg.turnRate, cfg.cmdDuration);
      } else if (move === "stop") {
        stopMove();
      }
    });
  });

  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", () => {
    keyState.clear();
    if (driveTimer) {
      clearInterval(driveTimer);
      driveTimer = null;
    }
    stopMove();
    setKeyboardState("Window lost focus");
  });
  window.addEventListener("beforeunload", () => disconnectSocket("beforeunload"));
  window.addEventListener("pagehide", () => disconnectSocket("pagehide"));
}

bindUi();
bindCanvasInteractions();
syncDrawControls();
renderCamera();
renderScanState();
resetView();
updateCameraRefreshStatus();
draw();
