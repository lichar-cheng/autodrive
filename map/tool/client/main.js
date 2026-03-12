let ws;
let pingTimer = null;
let reconnectTimer = null;
let reconnectAttempts = 0;

let lastPose = {x:0,y:0,yaw:0};
let lastGps = {lat:0, lon:0};
let front = [], rear = [], clusters = [];
let lastChassis = {wheel_speed_l:0,wheel_speed_r:0,battery:0,mode:'-'};
let pathNodes = [];
let poiNodes = [];
let trajectory = [];
let cameraState = new Map();

const streamHealth = {
  msgTotal: 0,
  checksumErr: 0,
  staleTsErr: 0,
  gapErr: 0,
  lastSeq: {},
  lastLagMs: 0,
  retriesHttp: 0,
};

const canvas = document.getElementById('lidarCanvas');
const ctx = canvas.getContext('2d');
const posePanel = document.getElementById('posePanel');
const pathList = document.getElementById('pathList');
const cameraGrid = document.getElementById('cameraGrid');
const gpsPanel = document.getElementById('gpsPanel');
const chassisPanel = document.getElementById('chassisPanel');
const commPanel = document.getElementById('commPanel');

const topicHandler = {
  '/robot/pose': (msg) => {
    lastPose = msg.payload;
    trajectory.push({x:lastPose.x, y:lastPose.y});
    if (trajectory.length > 3000) trajectory = trajectory.slice(-3000);
    posePanel.innerText = JSON.stringify(lastPose, null, 2);
  },
  '/robot/gps': (msg) => {
    lastGps = msg.payload;
    gpsPanel.innerText = JSON.stringify(lastGps, null, 2);
  },
  '/chassis/status': (msg) => {
    lastChassis = msg.payload;
    chassisPanel.innerText = JSON.stringify(lastChassis, null, 2);
  },
  '/chassis/odom': (msg) => {
    // reserved for odom-vs-pose consistency check
  },
  '/lidar/front': (msg) => { front = msg.payload.points; },
  '/lidar/rear': (msg) => { rear = msg.payload.points; },
  '/map/grid': (msg) => { clusters = msg.payload.clusters; },
};
for (let i=1; i<=4; i++) {
  topicHandler[`/camera/${i}/compressed`] = (msg) => {
    cameraState.set(i, msg.payload.objects);
    renderCamera();
  };
}

function stableStringify(obj) {
  if (obj === null || typeof obj !== 'object') return JSON.stringify(obj);
  if (Array.isArray(obj)) return `[${obj.map(stableStringify).join(',')}]`;
  const keys = Object.keys(obj).sort();
  return `{${keys.map(k => `${JSON.stringify(k)}:${stableStringify(obj[k])}`).join(',')}}`;
}

async function sha256Hex(text) {
  const enc = new TextEncoder().encode(text);
  const buf = await crypto.subtle.digest('SHA-256', enc);
  return [...new Uint8Array(buf)].map(x => x.toString(16).padStart(2, '0')).join('');
}

async function validateMessage(msg) {
  streamHealth.msgTotal += 1;
  const basis = `${msg.topic}|${Number(msg.stamp).toFixed(6)}|${msg.seq}|${stableStringify(msg.payload)}`;
  const digest = await sha256Hex(basis);
  if (digest !== msg.checksum) streamHealth.checksumErr += 1;

  const lagMs = Date.now() - Number(msg.server_time_ms || Date.now());
  streamHealth.lastLagMs = lagMs;
  if (Math.abs(lagMs) > 5000) streamHealth.staleTsErr += 1;

  const last = streamHealth.lastSeq[msg.topic];
  if (typeof last === 'number' && msg.seq > last + 1) streamHealth.gapErr += (msg.seq - last - 1);
  streamHealth.lastSeq[msg.topic] = msg.seq;
}

function setStatus(txt) {
  document.getElementById('status').innerText = txt;
}

function scheduleReconnect() {
  if (reconnectTimer) return;
  reconnectAttempts += 1;
  const backoff = Math.min(10000, Math.round((2 ** Math.min(reconnectAttempts, 6)) * 200 + Math.random() * 300));
  setStatus(`重连中 ${backoff}ms (第${reconnectAttempts}次)`);
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connect();
  }, backoff);
}

function startPing() {
  clearInterval(pingTimer);
  pingTimer = setInterval(() => {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send('ping');
  }, 5000);
}

function stopPing() {
  clearInterval(pingTimer);
  pingTimer = null;
}

function connect() {
  const url = document.getElementById('serverUrl').value;
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) ws.close();
  ws = new WebSocket(url);

  ws.onopen = () => {
    reconnectAttempts = 0;
    setStatus('已连接');
    startPing();
  };
  ws.onclose = () => {
    stopPing();
    setStatus('已断开');
    scheduleReconnect();
  };
  ws.onerror = () => {
    setStatus('连接异常');
  };
  ws.onmessage = async (event) => {
    if (event.data === 'pong') return;
    const msg = JSON.parse(event.data);
    await validateMessage(msg);
    if (topicHandler[msg.topic]) topicHandler[msg.topic](msg);
  };
}

async function callApi(path, body={}, retries=3) {
  const base = document.getElementById('serverUrl').value.replace('ws://','http://').replace('/ws/stream','');
  let lastErr;
  for (let i=0; i<=retries; i++) {
    try {
      const res = await fetch(base + path, {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return await res.json();
    } catch (err) {
      lastErr = err;
      if (i < retries) {
        streamHealth.retriesHttp += 1;
        await new Promise(r => setTimeout(r, 200 * (2 ** i)));
      }
    }
  }
  throw lastErr;
}

document.getElementById('connectBtn').onclick = connect;
document.getElementById('startScanBtn').onclick = () => callApi('/scan/start').catch(console.error);
document.getElementById('stopScanBtn').onclick = () => callApi('/scan/stop').catch(console.error);
document.getElementById('saveMapBtn').onclick = async () => {
  try {
    const r = await callApi('/map/save', {name:'demo', notes:'wifi mapping session'});
    alert('地图已保存: ' + r.file);
  } catch (e) {
    alert('保存失败: ' + e.message);
  }
};
document.getElementById('sendPathBtn').onclick = async () => {
  await callApi('/path/plan', {nodes:pathNodes});
};

canvas.addEventListener('click', async (e) => {
  const rect = canvas.getBoundingClientRect();
  const x = ((e.clientX - rect.left) - canvas.width/2) / 25;
  const y = (canvas.height/2 - (e.clientY - rect.top)) / 25;

  if (e.shiftKey) {
    const poi = {name:`POI-${poiNodes.length+1}`, x, y, lat:lastGps.lat, lon:lastGps.lon};
    poiNodes.push(poi);
    await callApi('/map/poi', {poi});
    return;
  }

  pathNodes.push({x, y, lat:lastGps.lat, lon:lastGps.lon});
  const li = document.createElement('li');
  li.textContent = `${pathNodes.length}. (${x.toFixed(2)}, ${y.toFixed(2)}) lat=${(lastGps.lat||0).toFixed(6)} lon=${(lastGps.lon||0).toFixed(6)}`;
  pathList.appendChild(li);
});

function sendMove(v, w, d=0.2) { callApi('/control/move', {velocity:v, yaw_rate:w, duration:d}).catch(console.error); }
function stopMove() { callApi('/control/stop').catch(console.error); }
window.sendMove = sendMove;
window.stopMove = stopMove;

const keyState = new Set();
let driveTimer = null;
function startKeyboardControl() {
  window.addEventListener('keydown', (e) => {
    const k = e.key.toLowerCase();
    if (['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright',' '].includes(k)) e.preventDefault();
    if (k === ' ') return stopMove();
    keyState.add(k);
    if (!driveTimer) driveTimer = setInterval(tickKeyboardControl, 120);
  });
  window.addEventListener('keyup', (e) => {
    keyState.delete(e.key.toLowerCase());
    if (keyState.size === 0 && driveTimer) {
      clearInterval(driveTimer);
      driveTimer = null;
      stopMove();
    }
  });
}

function tickKeyboardControl() {
  let v = 0, w = 0;
  if (keyState.has('w') || keyState.has('arrowup')) v += 0.8;
  if (keyState.has('s') || keyState.has('arrowdown')) v -= 0.5;
  if (keyState.has('a') || keyState.has('arrowleft')) w += 1.0;
  if (keyState.has('d') || keyState.has('arrowright')) w -= 1.0;
  if (v !== 0 || w !== 0) sendMove(Math.abs(v), w, 0.15);
}

function draw() {
  ctx.fillStyle = '#05070a';
  ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle = '#1f2a3c';
  for (let i=0; i<canvas.width; i+=50) { ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,canvas.height); ctx.stroke(); }
  for (let i=0; i<canvas.height; i+=50) { ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(canvas.width,i); ctx.stroke(); }

  drawPoints(front, '#00d1ff');
  drawPoints(rear, '#ffae00');
  drawClusters();
  drawTrajectory();
  drawRobot();
  drawPath(pathNodes, '#ffd166');
  drawPoi();
  drawComm();
  requestAnimationFrame(draw);
}

function drawPoints(pts, color) {
  ctx.fillStyle = color;
  pts.forEach(([x,y]) => {
    const sx = canvas.width/2 + x*25;
    const sy = canvas.height/2 - y*25;
    ctx.fillRect(sx, sy, 2, 2);
  });
}
function drawClusters() {
  ctx.fillStyle = '#8df58e';
  clusters.forEach(p => {
    const sx = canvas.width/2 + p.x*25;
    const sy = canvas.height/2 - p.y*25;
    ctx.beginPath(); ctx.arc(sx, sy, 4, 0, Math.PI*2); ctx.fill();
  });
}
function drawRobot() {
  const x = canvas.width/2 + lastPose.x*25;
  const y = canvas.height/2 - lastPose.y*25;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-lastPose.yaw);
  ctx.fillStyle = '#ff4d6d';
  ctx.fillRect(-8, -5, 16, 10);
  ctx.strokeStyle = '#fff';
  ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(12,0); ctx.stroke();
  ctx.restore();
}
function drawPath(nodes, color) {
  if (nodes.length < 1) return;
  ctx.strokeStyle = color;
  ctx.beginPath();
  nodes.forEach((p, idx) => {
    const x = canvas.width/2 + p.x*25;
    const y = canvas.height/2 - p.y*25;
    if (idx===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.stroke();
}
function drawTrajectory() {
  drawPath(trajectory, '#9d7dff');
}
function drawPoi() {
  ctx.fillStyle = '#ff6b6b';
  poiNodes.forEach(p => {
    const sx = canvas.width/2 + p.x*25;
    const sy = canvas.height/2 - p.y*25;
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI*2); ctx.fill();
  });
}
function drawComm() {
  commPanel.innerText = JSON.stringify({
    wsState: ws ? ws.readyState : -1,
    reconnectAttempts,
    msgTotal: streamHealth.msgTotal,
    checksumErr: streamHealth.checksumErr,
    staleTsErr: streamHealth.staleTsErr,
    gapErr: streamHealth.gapErr,
    lagMs: streamHealth.lastLagMs,
    httpRetries: streamHealth.retriesHttp,
  }, null, 2);
}

function renderCamera() {
  cameraGrid.innerHTML = '';
  for (let i=1; i<=4; i++) {
    const objs = cameraState.get(i) || [];
    const card = document.createElement('div');
    card.className = 'camera-card';
    card.innerHTML = `<b>Camera ${i}</b><div>${objs.map(o=>`${o.label}(${o.confidence})`).join('<br/>') || '无目标'}</div>`;
    cameraGrid.appendChild(card);
  }
}

startKeyboardControl();
draw();
