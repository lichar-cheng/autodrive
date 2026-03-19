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
let lastOdom = { x: 0, y: 0, yaw: 0, vx: 0, wz: 0 };
let lastChassis = { wheel_speed_l: 0, wheel_speed_r: 0, battery: 0, mode: "-" };
let lastScanInfo = {
  front: { raw_points: 0, keyframe: false, stamp: 0 },
  rear: { raw_points: 0, keyframe: false, stamp: 0 },
};
let poseHistory = [];
let pathNodes = [];
let pathSegments = [];
let poiNodes = [];
let pendingFreePoint = null;
let pendingPoiDraft = null;
let pendingPoiQueue = [];
let selectedSegmentId = null;
let selectedPoiIds = new Set();
let poiIdSeed = 1;
let segmentIdSeed = 1;
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

const scanMergeConfig = {
  maxPoseHistory: 240,
  turnSkipWz: 0.35,
  saveMinHits: 3,
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

const stcmInspector = {
  file: null,
  bundle: null,
  points: [],
  pgmText: "",
  configJsonText: "",
  summary: null,
};

const drawState = {
  showPath: true,
  showPoi: true,
  showRobot: true,
  showMotionControlCard: true,
  showPoiCard: true,
  showTrajectoryCard: true,
  showStcmInspectorCard: false,
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
  poiList: document.getElementById("poiList"),
  cameraGrid: document.getElementById("cameraGrid"),
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
  poiNameInput: document.getElementById("poiNameInput"),
  poiGeoInput: document.getElementById("poiGeoInput"),
  poiBatchCreateInput: document.getElementById("poiBatchCreateInput"),
  trajectoryToolMode: document.getElementById("trajectoryToolMode"),
  showPathToggle: document.getElementById("showPathToggle"),
  showPoiToggle: document.getElementById("showPoiToggle"),
  showRobotToggle: document.getElementById("showRobotToggle"),
  drawModeBadge: document.getElementById("drawModeBadge"),
  poiStatus: document.getElementById("poiStatus"),
  trajectoryStatus: document.getElementById("trajectoryStatus"),
  motionControlCard: document.getElementById("motionControlCard"),
  poiCard: document.getElementById("poiCard"),
  trajectoryCard: document.getElementById("trajectoryCard"),
  stcmInspectorCard: document.getElementById("stcmInspectorCard"),
  showMotionControlCardToggle: document.getElementById("showMotionControlCardToggle"),
  showPoiCardToggle: document.getElementById("showPoiCardToggle"),
  showTrajectoryCardToggle: document.getElementById("showTrajectoryCardToggle"),
  showStcmInspectorCardToggle: document.getElementById("showStcmInspectorCardToggle"),
  clearPoiBtn: document.getElementById("clearPoiBtn"),
  addPoiBtn: document.getElementById("addPoiBtn"),
  applyPoiGeoBtn: document.getElementById("applyPoiGeoBtn"),
  copyPoiBtn: document.getElementById("copyPoiBtn"),
  poiCountBadge: document.getElementById("poiCountBadge"),
  autoConnectBtn: document.getElementById("autoConnectBtn"),
  connectSelectedPoiBtn: document.getElementById("connectSelectedPoiBtn"),
  deleteSegmentBtn: document.getElementById("deleteSegmentBtn"),
  refreshCameraBtn: document.getElementById("refreshCameraBtn"),
  cameraRefreshStatus: document.getElementById("cameraRefreshStatus"),
  viewMetrics: document.getElementById("viewMetrics"),
  connectBtn: document.getElementById("connectBtn"),
  savePathHint: document.getElementById("savePathHint"),
  stcmFileInput: document.getElementById("stcmFileInput"),
  stcmResolutionInput: document.getElementById("stcmResolutionInput"),
  stcmPaddingInput: document.getElementById("stcmPaddingInput"),
  inspectStcmBtn: document.getElementById("inspectStcmBtn"),
  downloadPgmBtn: document.getElementById("downloadPgmBtn"),
  downloadStcmJsonBtn: document.getElementById("downloadStcmJsonBtn"),
  stcmInspectorStatus: document.getElementById("stcmInspectorStatus"),
  stcmFileMeta: document.getElementById("stcmFileMeta"),
  stcmGridMeta: document.getElementById("stcmGridMeta"),
  stcmSummaryPanel: document.getElementById("stcmSummaryPanel"),
  stcmManifestPanel: document.getElementById("stcmManifestPanel"),
};

const topicHandler = {
  "/robot/pose": (msg) => {
    lastPose = msg.payload;
  },
  "/robot/gps": (msg) => {
    lastGps = msg.payload;
  },
  "/chassis/status": (msg) => {
    lastChassis = msg.payload;
    renderScanState();
  },
  "/chassis/odom": (msg) => {
    lastOdom = msg.payload;
    poseHistory.push({
      stamp: Number(msg.stamp || Date.now() / 1000),
      pose: { ...msg.payload },
    });
    if (poseHistory.length > scanMergeConfig.maxPoseHistory) {
      poseHistory = poseHistory.slice(-scanMergeConfig.maxPoseHistory);
    }
    renderOdomScanPanel();
  },
  "/lidar/front": (msg) => {
    scanSession.frontFrames += 1;
    lastScanInfo.front = {
      raw_points: Number(msg.payload.raw_points || (msg.payload.points || []).length || 0),
      keyframe: Boolean(msg.payload.keyframe),
      stamp: Number(msg.stamp || 0),
    };
    accumulatePoints(msg.payload.points || [], {
      source: "front",
      stamp: Number(msg.stamp || 0),
      keyframe: Boolean(msg.payload.keyframe),
    });
    renderOdomScanPanel();
  },
  "/lidar/rear": (msg) => {
    scanSession.rearFrames += 1;
    lastScanInfo.rear = {
      raw_points: Number(msg.payload.raw_points || (msg.payload.points || []).length || 0),
      keyframe: Boolean(msg.payload.keyframe),
      stamp: Number(msg.stamp || 0),
    };
    accumulatePoints(msg.payload.points || [], {
      source: "rear",
      stamp: Number(msg.stamp || 0),
      keyframe: Boolean(msg.payload.keyframe),
    });
    renderOdomScanPanel();
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

function renderOdomScanPanel() {
  elements.posePanel.innerText = JSON.stringify({
    odom: lastOdom,
    scan: {
      front_frames: scanSession.frontFrames,
      rear_frames: scanSession.rearFrames,
      raw_live_points: scanSession.totalLivePoints,
      front: lastScanInfo.front,
      rear: lastScanInfo.rear,
    },
  }, null, 2);
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

function setInspectorStatus(text, level = "") {
  elements.stcmInspectorStatus.innerText = text;
  elements.stcmInspectorStatus.className = `view-chip ${level}`.trim();
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

async function decompressZipEntry(entry) {
  if (entry.compressionMethod === 0) {
    return entry.data;
  }
  if (entry.compressionMethod !== 8) {
    throw new Error(`Unsupported ZIP compression method: ${entry.compressionMethod}`);
  }
  if (typeof DecompressionStream === "undefined") {
    throw new Error("Browser does not support DecompressionStream for ZIP inflate");
  }
  const stream = new Blob([entry.data]).stream().pipeThrough(new DecompressionStream("deflate-raw"));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

function parseZipEntries(arrayBuffer) {
  const bytes = new Uint8Array(arrayBuffer);
  const view = new DataView(arrayBuffer);
  let eocdOffset = -1;

  for (let i = bytes.length - 22; i >= Math.max(0, bytes.length - 65557); i -= 1) {
    if (view.getUint32(i, true) === 0x06054b50) {
      eocdOffset = i;
      break;
    }
  }
  if (eocdOffset < 0) {
    throw new Error("Invalid STCM/ZIP file: EOCD not found");
  }

  const centralDirectorySize = view.getUint32(eocdOffset + 12, true);
  const centralDirectoryOffset = view.getUint32(eocdOffset + 16, true);
  const entries = new Map();
  let cursor = centralDirectoryOffset;
  const centralEnd = centralDirectoryOffset + centralDirectorySize;

  while (cursor < centralEnd) {
    if (view.getUint32(cursor, true) !== 0x02014b50) {
      throw new Error("Invalid ZIP central directory header");
    }
    const compressionMethod = view.getUint16(cursor + 10, true);
    const compressedSize = view.getUint32(cursor + 20, true);
    const uncompressedSize = view.getUint32(cursor + 24, true);
    const fileNameLength = view.getUint16(cursor + 28, true);
    const extraLength = view.getUint16(cursor + 30, true);
    const commentLength = view.getUint16(cursor + 32, true);
    const localHeaderOffset = view.getUint32(cursor + 42, true);
    const fileNameBytes = bytes.slice(cursor + 46, cursor + 46 + fileNameLength);
    const fileName = new TextDecoder().decode(fileNameBytes);

    if (view.getUint32(localHeaderOffset, true) !== 0x04034b50) {
      throw new Error(`Invalid ZIP local header for ${fileName}`);
    }
    const localNameLength = view.getUint16(localHeaderOffset + 26, true);
    const localExtraLength = view.getUint16(localHeaderOffset + 28, true);
    const dataOffset = localHeaderOffset + 30 + localNameLength + localExtraLength;
    const data = bytes.slice(dataOffset, dataOffset + compressedSize);

    entries.set(fileName, {
      name: fileName,
      compressionMethod,
      compressedSize,
      uncompressedSize,
      data,
    });

    cursor += 46 + fileNameLength + extraLength + commentLength;
  }

  return entries;
}

function parseRadarPoints(bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const points = [];
  for (let offset = 0; offset + 12 <= bytes.byteLength; offset += 12) {
    points.push([
      view.getFloat32(offset, true),
      view.getFloat32(offset + 4, true),
      view.getFloat32(offset + 8, true),
    ]);
  }
  return points;
}

function buildPgmFromPoints(points, resolution, paddingCells) {
  if (!points.length) {
    throw new Error("No radar points in STCM");
  }

  let minX = points[0][0];
  let maxX = points[0][0];
  let minY = points[0][1];
  let maxY = points[0][1];

  points.forEach(([x, y]) => {
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
  });

  const paddedMinX = minX - paddingCells * resolution;
  const paddedMinY = minY - paddingCells * resolution;
  const paddedMaxX = maxX + paddingCells * resolution;
  const paddedMaxY = maxY + paddingCells * resolution;
  const width = Math.max(1, Math.ceil((paddedMaxX - paddedMinX) / resolution) + 1);
  const height = Math.max(1, Math.ceil((paddedMaxY - paddedMinY) / resolution) + 1);
  const grid = new Uint8Array(width * height).fill(205);
  const occupiedSet = new Set();

  points.forEach(([x, y]) => {
    const ix = Math.min(width - 1, Math.max(0, Math.round((x - paddedMinX) / resolution)));
    const iy = Math.min(height - 1, Math.max(0, Math.round((y - paddedMinY) / resolution)));
    const flippedY = height - 1 - iy;
    const idx = flippedY * width + ix;
    grid[idx] = 0;
    occupiedSet.add(`${ix}:${iy}`);
  });

  const rows = [];
  for (let row = 0; row < height; row += 1) {
    const start = row * width;
    const values = [];
    for (let col = 0; col < width; col += 1) {
      values.push(String(grid[start + col]));
    }
    rows.push(values.join(" "));
  }

  const pgmText = `P2\n# Generated from STCM radar_points\n${width} ${height}\n255\n${rows.join("\n")}\n`;
  return {
    pgmText,
    width,
    height,
    origin: [Number(paddedMinX.toFixed(4)), Number(paddedMinY.toFixed(4)), 0],
    bounds: {
      minX: Number(minX.toFixed(4)),
      maxX: Number(maxX.toFixed(4)),
      minY: Number(minY.toFixed(4)),
      maxY: Number(maxY.toFixed(4)),
    },
    occupiedCells: occupiedSet.size,
  };
}

function buildStcmConfig(fileName, manifest, points, resolution, paddingCells) {
  const pgm = buildPgmFromPoints(points, resolution, paddingCells);
  const config = {
    image: fileName.replace(/\.stcm$/i, ".pgm"),
    resolution,
    origin: pgm.origin,
    negate: 0,
    occupied_thresh: 0.65,
    free_thresh: 0.196,
    mode: "trinary",
    stcm_meta: {
      source_file: fileName,
      version: manifest.version || "unknown",
      map_source: manifest.map_source || "unknown",
      created_at: manifest.created_at || null,
      radar_points: points.length,
      occupied_cells: pgm.occupiedCells,
      width: pgm.width,
      height: pgm.height,
      bounds: pgm.bounds,
    },
  };
  return { pgm, configJsonText: JSON.stringify(config, null, 2) };
}

function downloadFile(fileName, content, mimeType) {
  const blob = new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = fileName;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function updateSavePathHint(pathText = "") {
  const localPath = pathText || "No file saved yet";
  elements.savePathHint.innerText = `Local export: ${localPath}. Browser should open a save dialog so you can choose the destination folder and filename.`;
}

async function inspectSelectedStcm() {
  const file = elements.stcmFileInput.files && elements.stcmFileInput.files[0];
  if (!file) {
    throw new Error("Please choose a .stcm file first");
  }

  setInspectorStatus("Parsing", "active");
  elements.stcmFileMeta.innerText = `${file.name} | ${(file.size / 1024).toFixed(1)} KB`;
  const arrayBuffer = await file.arrayBuffer();
  const entries = parseZipEntries(arrayBuffer);
  const manifestEntry = entries.get("manifest.json");
  const radarEntry = entries.get("radar_points.bin");
  if (!manifestEntry || !radarEntry) {
    throw new Error("STCM must contain manifest.json and radar_points.bin");
  }

  const manifestBytes = await decompressZipEntry(manifestEntry);
  const radarBytes = await decompressZipEntry(radarEntry);
  const manifest = JSON.parse(new TextDecoder().decode(manifestBytes));
  const points = parseRadarPoints(radarBytes);
  const resolution = Math.max(0.02, numberFromInput(elements.stcmResolutionInput, 0.1));
  const paddingCells = Math.max(0, Math.round(numberFromInput(elements.stcmPaddingInput, 8)));
  const { pgm, configJsonText } = buildStcmConfig(file.name, manifest, points, resolution, paddingCells);

  stcmInspector.file = file;
  stcmInspector.bundle = manifest;
  stcmInspector.points = points;
  stcmInspector.pgmText = pgm.pgmText;
  stcmInspector.configJsonText = configJsonText;
  stcmInspector.summary = {
    file: file.name,
    manifestVersion: manifest.version || "unknown",
    mapSource: manifest.map_source || "unknown",
    radarPoints: points.length,
    poiCount: Array.isArray(manifest.poi) ? manifest.poi.length : 0,
    pathCount: Array.isArray(manifest.path) ? manifest.path.length : 0,
    trajectoryCount: Array.isArray(manifest.trajectory) ? manifest.trajectory.length : 0,
    width: pgm.width,
    height: pgm.height,
    resolution,
    paddingCells,
    occupiedCells: pgm.occupiedCells,
    origin: pgm.origin,
    bounds: pgm.bounds,
  };

  elements.stcmGridMeta.innerText = `${pgm.width} x ${pgm.height} | ${pgm.occupiedCells} occupied | res ${resolution.toFixed(2)} m/px`;
  elements.stcmSummaryPanel.innerText = JSON.stringify(stcmInspector.summary, null, 2);
  elements.stcmManifestPanel.innerText = JSON.stringify(manifest, null, 2);
  setInspectorStatus("Ready", "active");
}

function encodeUint16LE(value) {
  const bytes = new Uint8Array(2);
  new DataView(bytes.buffer).setUint16(0, value, true);
  return bytes;
}

function encodeUint32LE(value) {
  const bytes = new Uint8Array(4);
  new DataView(bytes.buffer).setUint32(0, value, true);
  return bytes;
}

function crc32(bytes) {
  let crc = 0xffffffff;
  for (let i = 0; i < bytes.length; i += 1) {
    crc ^= bytes[i];
    for (let j = 0; j < 8; j += 1) {
      const mask = -(crc & 1);
      crc = (crc >>> 1) ^ (0xedb88320 & mask);
    }
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function concatUint8Arrays(chunks) {
  const total = chunks.reduce((sum, chunk) => sum + chunk.length, 0);
  const output = new Uint8Array(total);
  let offset = 0;
  chunks.forEach((chunk) => {
    output.set(chunk, offset);
    offset += chunk.length;
  });
  return output;
}

function createStoredZip(files) {
  const encoder = new TextEncoder();
  const localParts = [];
  const centralParts = [];
  let offset = 0;

  files.forEach((file) => {
    const nameBytes = encoder.encode(file.name);
    const dataBytes = file.data instanceof Uint8Array ? file.data : encoder.encode(file.data);
    const checksum = crc32(dataBytes);
    const localHeader = concatUint8Arrays([
      encodeUint32LE(0x04034b50),
      encodeUint16LE(20),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint32LE(checksum),
      encodeUint32LE(dataBytes.length),
      encodeUint32LE(dataBytes.length),
      encodeUint16LE(nameBytes.length),
      encodeUint16LE(0),
      nameBytes,
      dataBytes,
    ]);
    localParts.push(localHeader);

    const centralHeader = concatUint8Arrays([
      encodeUint32LE(0x02014b50),
      encodeUint16LE(20),
      encodeUint16LE(20),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint32LE(checksum),
      encodeUint32LE(dataBytes.length),
      encodeUint32LE(dataBytes.length),
      encodeUint16LE(nameBytes.length),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint16LE(0),
      encodeUint32LE(0),
      encodeUint32LE(offset),
      nameBytes,
    ]);
    centralParts.push(centralHeader);
    offset += localHeader.length;
  });

  const centralDirectory = concatUint8Arrays(centralParts);
  const endOfCentralDirectory = concatUint8Arrays([
    encodeUint32LE(0x06054b50),
    encodeUint16LE(0),
    encodeUint16LE(0),
    encodeUint16LE(files.length),
    encodeUint16LE(files.length),
    encodeUint32LE(centralDirectory.length),
    encodeUint32LE(offset),
    encodeUint16LE(0),
  ]);

  return concatUint8Arrays([...localParts, centralDirectory, endOfCentralDirectory]);
}

function serializeRadarPoints(points) {
  const bytes = new Uint8Array(points.length * 12);
  const view = new DataView(bytes.buffer);
  points.forEach((point, index) => {
    const offset = index * 12;
    view.setFloat32(offset, Number(point[0] || 0), true);
    view.setFloat32(offset + 4, Number(point[1] || 0), true);
    view.setFloat32(offset + 8, Number(point[2] || 0), true);
  });
  return bytes;
}

function buildClientStcmBundle() {
  rebuildPathNodes();
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

  return {
    version: "stcm.v2",
    notes: JSON.stringify(notes, null, 2),
    created_at: Date.now() / 1000,
    source: "browser",
    map_source: "laser_accumulation",
    pose: lastPose,
    gps: lastGps,
    chassis: lastChassis,
    poi: poiNodes.map((poi) => sanitizePoiPayload(poi)),
    path: pathNodes,
    trajectory: pathSegments.map((segment) => ({
      id: segment.id,
      source: segment.source,
      geometry: segment.geometry,
      curveOffset: Number(segment.curveOffset || 0),
      start: sanitizePathNode(segment.start),
      end: sanitizePathNode(segment.end),
    })),
    gps_track: [],
    chassis_track: [],
    scan_summary: {
      scanActive: scanSession.active,
      elapsedSec: scanSession.startedAtMs ? Number(((Date.now() - scanSession.startedAtMs) / 1000).toFixed(1)) : 0,
      obstacleCells: scanSession.occupiedCells.size,
      safeCells: scanSession.freeCells.size,
      rawLidarPoints: scanSession.totalLivePoints,
      frontFrames: scanSession.frontFrames,
      rearFrames: scanSession.rearFrames,
      voxelSize,
    },
    radar_points: pointsToSave.length ? pointsToSave : [[0, 0, 1]],
  };
}

async function saveBlobWithPicker(fileName, blob) {
  if (window.showSaveFilePicker) {
    const handle = await window.showSaveFilePicker({
      suggestedName: fileName,
      types: [
        {
          description: "STCM map",
          accept: { "application/octet-stream": [".stcm"] },
        },
      ],
    });
    const writable = await handle.createWritable();
    await writable.write(blob);
    await writable.close();
    return handle.name || fileName;
  }

  downloadFile(fileName, blob, "application/octet-stream");
  return `${fileName} (downloaded)`;
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

function poseForStamp(stamp) {
  if (!poseHistory.length || !Number.isFinite(stamp) || stamp <= 0) {
    return lastOdom;
  }
  let best = poseHistory[poseHistory.length - 1];
  let bestDelta = Math.abs(best.stamp - stamp);
  for (const item of poseHistory) {
    const delta = Math.abs(item.stamp - stamp);
    if (delta < bestDelta) {
      best = item;
      bestDelta = delta;
    }
  }
  return best.pose;
}

function shouldSkipScanFrame(framePose, keyframe) {
  if (keyframe) {
    return false;
  }
  return Math.abs(Number(framePose.wz || 0)) >= scanMergeConfig.turnSkipWz;
}

function accumulatePoints(points, meta = {}) {
  if (!scanSession.active || !Array.isArray(points) || points.length === 0) {
    renderScanState();
    return;
  }

  scanSession.voxelSize = Math.max(0.02, numberFromInput(elements.voxelSizeInput, 0.12));
  const framePose = poseForStamp(Number(meta.stamp || 0));
  if (shouldSkipScanFrame(framePose, Boolean(meta.keyframe))) {
    renderScanState();
    return;
  }
  scanSession.totalLivePoints += points.length;
  const [robotIx, robotIy] = worldToCell(framePose.x, framePose.y);

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
    .filter((cell) => cell.hits >= scanMergeConfig.saveMinHits)
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

function makeLocalPoint(x, y, meta = {}) {
  return {
    x: Number(x),
    y: Number(y),
    lat: Number.isFinite(meta.lat) ? Number(meta.lat) : null,
    lon: Number.isFinite(meta.lon) ? Number(meta.lon) : null,
    poiId: meta.poiId || meta.clientId || null,
    name: meta.name || "",
  };
}

function sanitizePathNode(node) {
  return {
    x: Number(node.x),
    y: Number(node.y),
    lat: Number.isFinite(node.lat) ? Number(node.lat) : null,
    lon: Number.isFinite(node.lon) ? Number(node.lon) : null,
  };
}

function sanitizePoiPayload(poi) {
  return {
    name: poi.name,
    x: Number(poi.x),
    y: Number(poi.y),
    yaw: Number.isFinite(poi.yaw) ? Number(poi.yaw) : 0,
    lat: Number.isFinite(poi.lat) ? Number(poi.lat) : null,
    lon: Number.isFinite(poi.lon) ? Number(poi.lon) : null,
  };
}

function distanceBetween(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function createSegment(start, end, options = {}) {
  return {
    id: `seg-${segmentIdSeed++}`,
    start: makeLocalPoint(start.x, start.y, start),
    end: makeLocalPoint(end.x, end.y, end),
    geometry: "line",
    curveOffset: 0,
    source: options.source || "free",
  };
}

function sampleSegment(segment) {
  if (!segment) {
    return [];
  }
  return [sanitizePathNode(segment.start), sanitizePathNode(segment.end)];
}

function rebuildPathNodes() {
  pathNodes = pathSegments.flatMap((segment) => sampleSegment(segment));
}

function describePoint(point) {
  return `(${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)})`;
}

function parseOptionalNumber(value) {
  if (value === null || value === undefined) {
    return null;
  }
  const text = String(value).trim();
  if (!text) {
    return null;
  }
  const num = Number(text);
  return Number.isFinite(num) ? num : null;
}

function parseLonLatText(value) {
  const text = String(value || "").trim();
  if (!text) {
    return { ok: true, empty: true, lon: null, lat: null };
  }
  const parts = text.split(",").map((item) => item.trim());
  if (parts.length !== 2) {
    return { ok: false, message: "Geo format must be lon,lat" };
  }
  const lon = Number(parts[0]);
  const lat = Number(parts[1]);
  if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
    return { ok: false, message: "Geo must contain valid numbers" };
  }
  if (lon < -180 || lon > 180) {
    return { ok: false, message: "Longitude must be between -180 and 180" };
  }
  if (lat < -90 || lat > 90) {
    return { ok: false, message: "Latitude must be between -90 and 90" };
  }
  return { ok: true, empty: false, lon, lat };
}

function loadPoiQueueFromInput() {
  const text = elements.poiBatchCreateInput.value.trim();
  if (!text) {
    return [];
  }
  const queue = [];
  const errors = [];
  text.split(/\r?\n/).map((line) => line.trim()).filter(Boolean).forEach((line, idx) => {
    const parts = line.split(",").map((item) => item.trim());
    const name = parts[0];
    if (!name) {
      errors.push(`line ${idx + 1}`);
      return;
    }
    const lon = parts.length >= 3 ? parseOptionalNumber(parts[1]) : null;
    const lat = parts.length >= 3 ? parseOptionalNumber(parts[2]) : null;
    const yaw = parts.length >= 4 ? parseOptionalNumber(parts[3]) : null;
    if (parts.length >= 3 && (lat === null || lon === null)) {
      errors.push(`line ${idx + 1}`);
      return;
    }
    if (parts.length >= 4 && yaw === null) {
      errors.push(`line ${idx + 1}`);
      return;
    }
    queue.push({ name, lat, lon, yaw });
  });
  if (errors.length) {
    alert(`Invalid batch POI input: ${errors.join(", ")}`);
    return null;
  }
  return queue;
}

function readManualGeoInput(requireValue = false) {
  const parsed = parseLonLatText(elements.poiGeoInput.value);
  if (!parsed.ok) {
    alert(parsed.message);
    elements.poiGeoInput.focus();
    return null;
  }
  if (requireValue && parsed.empty) {
    alert("Input geo as lon,lat");
    elements.poiGeoInput.focus();
    return null;
  }
  return parsed;
}

function startNextPoiDraft() {
  if (pendingPoiDraft || !pendingPoiQueue.length) {
    return false;
  }
  pendingPoiDraft = pendingPoiQueue.shift();
  syncTrajectoryPanel();
  return true;
}

function buildPoiCopyText() {
  return poiNodes.map((poi) => [
    poi.name,
    Number(poi.x).toFixed(3),
    Number(poi.y).toFixed(3),
    Number.isFinite(poi.yaw) ? Number(poi.yaw).toFixed(3) : "0.000",
    Number.isFinite(poi.lat) ? Number(poi.lat).toFixed(6) : "",
    Number.isFinite(poi.lon) ? Number(poi.lon).toFixed(6) : "",
  ].join(",")).join("\n");
}

function syncTrajectoryButtons() {
  const selectedSegment = pathSegments.find((segment) => segment.id === selectedSegmentId) || null;
  elements.deleteSegmentBtn.disabled = !selectedSegment;
  elements.connectSelectedPoiBtn.disabled = selectedPoiIds.size !== 2;
  elements.clearPoiBtn.disabled = selectedPoiIds.size === 0;
  elements.applyPoiGeoBtn.disabled = selectedPoiIds.size === 0;
  elements.copyPoiBtn.disabled = poiNodes.length === 0;
  if (pendingPoiDraft) {
    elements.addPoiBtn.innerText = "Cancel Add POI";
  } else if (pendingPoiQueue.length) {
    elements.addPoiBtn.innerText = `Add POI (${pendingPoiQueue.length} queued)`;
  } else {
    elements.addPoiBtn.innerText = "Add POI";
  }
}

function renderPoiList() {
  elements.poiList.innerHTML = "";
  elements.poiCountBadge.innerText = `${poiNodes.length} POI${selectedPoiIds.size ? ` | ${selectedPoiIds.size} Selected` : ""}`;
  if (!poiNodes.length) {
    const li = document.createElement("li");
    li.innerHTML = `<span class="list-main">No POI yet</span><span class="list-meta">Input name, click Add POI, then click canvas to place</span>`;
    elements.poiList.appendChild(li);
    return;
  }
  poiNodes.forEach((poi, index) => {
    const li = document.createElement("li");
    if (selectedPoiIds.has(poi.clientId)) {
      li.classList.add("selected");
    }
    li.innerHTML = `
      <span class="list-main">${index + 1}. ${poi.name} ${describePoint(poi)}</span>
      <span class="list-meta">yaw=${Number.isFinite(poi.yaw) ? poi.yaw.toFixed(3) : "0.000"} | lat=${Number.isFinite(poi.lat) ? poi.lat.toFixed(6) : "n/a"} lon=${Number.isFinite(poi.lon) ? poi.lon.toFixed(6) : "n/a"}</span>
    `;
    li.onclick = () => {
      togglePoiSelection(poi.clientId);
    };
    elements.poiList.appendChild(li);
  });
}

function renderSegmentList() {
  elements.pathList.innerHTML = "";
  if (!pathSegments.length) {
    const li = document.createElement("li");
    if (pendingFreePoint) {
      li.classList.add("pending");
      li.innerHTML = `<span class="list-main">Free-point mode start saved at ${describePoint(pendingFreePoint)}</span><span class="list-meta">Click canvas again to create the segment</span>`;
    } else {
      li.innerHTML = `<span class="list-main">No segments yet</span><span class="list-meta">Auto-connect POI or add one with the current tool</span>`;
    }
    elements.pathList.appendChild(li);
    return;
  }

  pathSegments.forEach((segment, index) => {
    const li = document.createElement("li");
    if (segment.id === selectedSegmentId) {
      li.classList.add("selected");
    }
    li.innerHTML = `
      <span class="list-main">${index + 1}. line | ${segment.source} | ${describePoint(segment.start)} -> ${describePoint(segment.end)}</span>
      <span class="list-meta">Length ${distanceBetween(segment.start, segment.end).toFixed(2)} m</span>
    `;
    li.onclick = () => {
      selectedSegmentId = segment.id;
      syncTrajectoryPanel();
    };
    elements.pathList.appendChild(li);
  });
}

function updateTrajectoryStatus() {
  const toolLabel = elements.trajectoryToolMode.value === "poi" ? "Click two POI to connect" : "Click two free points to connect";
  const pendingText = pendingFreePoint ? ` | Start ${describePoint(pendingFreePoint)}` : "";
  elements.trajectoryStatus.innerText = `Segments ${pathSegments.length} | Nodes ${pathNodes.length} | Tool ${toolLabel}${pendingText}`;
}

function syncTrajectoryPanel() {
  rebuildPathNodes();
  renderPoiList();
  renderSegmentList();
  updateTrajectoryStatus();
  elements.poiStatus.innerText = pendingPoiDraft
    ? `Ready to place "${pendingPoiDraft.name}" on canvas${pendingPoiQueue.length ? ` | ${pendingPoiQueue.length} queued` : ""}`
    : pendingPoiQueue.length
      ? `${pendingPoiQueue.length} POI queued. Click Add POI to start`
      : "POI idle";
  syncTrajectoryButtons();
}

function togglePoiSelection(clientId) {
  if (selectedPoiIds.has(clientId)) {
    selectedPoiIds.delete(clientId);
  } else {
    if (selectedPoiIds.size >= 2) {
      const first = selectedPoiIds.values().next().value;
      selectedPoiIds.delete(first);
    }
    selectedPoiIds.add(clientId);
  }
  syncTrajectoryPanel();
}

function clearPoi() {
  if (!selectedPoiIds.size) {
    alert("Select POI to delete");
    return;
  }
  const removedIds = new Set(selectedPoiIds);
  poiNodes = poiNodes.filter((poi) => !removedIds.has(poi.clientId));
  pathSegments = pathSegments.filter((segment) => !removedIds.has(segment.start.poiId) && !removedIds.has(segment.end.poiId));
  selectedPoiIds = new Set();
  pendingPoiDraft = null;
  pendingPoiQueue = [];
  if (selectedSegmentId && !pathSegments.some((segment) => segment.id === selectedSegmentId)) {
    selectedSegmentId = null;
  }
  syncTrajectoryPanel();
}

function applyGeoToSelectedPoi() {
  if (!selectedPoiIds.size) {
    alert("Select POI first");
    return;
  }
  const geo = readManualGeoInput(true);
  if (!geo) {
    return;
  }
  poiNodes.forEach((poi) => {
    if (selectedPoiIds.has(poi.clientId)) {
      poi.lat = geo.lat;
      poi.lon = geo.lon;
    }
  });
  syncTrajectoryPanel();
}

function togglePoiPlacement() {
  if (pendingPoiDraft) {
    pendingPoiDraft = null;
    pendingPoiQueue = [];
    syncTrajectoryPanel();
    return;
  }
  const queue = loadPoiQueueFromInput();
  if (queue === null) {
    return;
  }
  if (queue.length) {
    pendingPoiQueue = queue;
    startNextPoiDraft();
    return;
  }
  const geo = readManualGeoInput(false);
  if (!geo) {
    return;
  }
  const name = elements.poiNameInput.value.trim();
  if (!name) {
    alert("Input POI name first");
    elements.poiNameInput.focus();
    return;
  }
  pendingPoiDraft = {
    name,
    lat: geo.lat,
    lon: geo.lon,
    yaw: null,
  };
  syncTrajectoryPanel();
}

async function copyPoiData() {
  if (!poiNodes.length) {
    alert("No POI to copy");
    return;
  }
  const text = buildPoiCopyText();
  if (navigator.clipboard && navigator.clipboard.writeText) {
    await navigator.clipboard.writeText(text);
  } else {
    const area = document.createElement("textarea");
    area.value = text;
    document.body.appendChild(area);
    area.select();
    document.execCommand("copy");
    area.remove();
  }
  alert("POI data copied");
}

function addSegment(segment) {
  pathSegments.push(segment);
  selectedSegmentId = segment.id;
  syncTrajectoryPanel();
}

function removeSelectedSegment() {
  if (!selectedSegmentId) {
    return;
  }
  pathSegments = pathSegments.filter((segment) => segment.id !== selectedSegmentId);
  selectedSegmentId = null;
  syncTrajectoryPanel();
}

function buildSegmentFromSelectedPoi() {
  if (selectedPoiIds.size !== 2) {
    alert("Select exactly two POI");
    return;
  }
  const selected = poiNodes.filter((poi) => selectedPoiIds.has(poi.clientId));
  if (selected.length !== 2) {
    alert("POI selection state is invalid");
    return;
  }
  addSegment(createSegment(selected[0], selected[1], {
    source: "poi",
  }));
}

function solveNearestLoop(points) {
  const ordered = [...points];
  ordered.sort((a, b) => a.x - b.x || a.y - b.y);
  const route = [ordered[0]];
  const remaining = ordered.slice(1);
  while (remaining.length) {
    const last = route[route.length - 1];
    let bestIndex = 0;
    let bestDistance = Number.POSITIVE_INFINITY;
    remaining.forEach((poi, index) => {
      const dist = distanceBetween(last, poi);
      if (dist < bestDistance) {
        bestDistance = dist;
        bestIndex = index;
      }
    });
    route.push(remaining.splice(bestIndex, 1)[0]);
  }
  return route;
}

function totalLoopDistance(route) {
  if (route.length < 2) {
    return 0;
  }
  let total = 0;
  for (let i = 0; i < route.length; i += 1) {
    total += distanceBetween(route[i], route[(i + 1) % route.length]);
  }
  return total;
}

function optimizeLoopWithTwoOpt(route) {
  if (route.length < 4) {
    return route;
  }
  let improved = true;
  let best = [...route];
  while (improved) {
    improved = false;
    for (let i = 1; i < best.length - 2; i += 1) {
      for (let j = i + 1; j < best.length - 1; j += 1) {
        const candidate = [
          ...best.slice(0, i),
          ...best.slice(i, j + 1).reverse(),
          ...best.slice(j + 1),
        ];
        if (totalLoopDistance(candidate) + 1e-6 < totalLoopDistance(best)) {
          best = candidate;
          improved = true;
        }
      }
    }
  }
  return best;
}

function autoConnectPoiLoop() {
  if (poiNodes.length < 2) {
    alert("At least two POI are required");
    return;
  }
  const route = optimizeLoopWithTwoOpt(solveNearestLoop(poiNodes));
  pathSegments = pathSegments.filter((segment) => segment.source !== "auto");
  for (let i = 0; i < route.length - 1; i += 1) {
    pathSegments.push(createSegment(route[i], route[i + 1], { source: "auto", geometry: "line" }));
  }
  if (route.length > 2) {
    pathSegments.push(createSegment(route[route.length - 1], route[0], { source: "auto", geometry: "line" }));
  }
  selectedSegmentId = pathSegments.length ? pathSegments[pathSegments.length - 1].id : null;
  syncTrajectoryPanel();
}

function findPoiAt(worldX, worldY, thresholdPx = 12) {
  const thresholdWorld = thresholdPx / viewState.scale;
  return poiNodes.find((poi) => Math.hypot(poi.x - worldX, poi.y - worldY) <= thresholdWorld) || null;
}

function distancePointToSegment(point, start, end) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  if (dx === 0 && dy === 0) {
    return Math.hypot(point.x - start.x, point.y - start.y);
  }
  const t = Math.max(0, Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / (dx * dx + dy * dy)));
  const projX = start.x + t * dx;
  const projY = start.y + t * dy;
  return Math.hypot(point.x - projX, point.y - projY);
}

function findSegmentAt(worldX, worldY, thresholdPx = 10) {
  const thresholdWorld = thresholdPx / viewState.scale;
  const hitPoint = { x: worldX, y: worldY };
  for (let i = pathSegments.length - 1; i >= 0; i -= 1) {
    const segment = pathSegments[i];
    const samples = sampleSegment(segment);
    for (let j = 0; j < samples.length - 1; j += 1) {
      if (distancePointToSegment(hitPoint, samples[j], samples[j + 1]) <= thresholdWorld) {
        return segment;
      }
    }
  }
  return null;
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
  const bundle = buildClientStcmBundle();
  const manifest = { ...bundle };
  delete manifest.radar_points;
  const zipBytes = createStoredZip([
    {
      name: "manifest.json",
      data: JSON.stringify(manifest, null, 2),
    },
    {
      name: "radar_points.bin",
      data: serializeRadarPoints(bundle.radar_points),
    },
  ]);
  const savedName = await saveBlobWithPicker(`${name}.stcm`, new Blob([zipBytes], { type: "application/octet-stream" }));
  scanSession.lastSavedFile = savedName;
  scanSession.savedPointCount = bundle.radar_points.length;
  updateSavePathHint(scanSession.lastSavedFile);
  renderScanState();
  alert(`Map saved: ${savedName}`);
}

async function sendPath() {
  rebuildPathNodes();
  await callApi("/path/plan", { nodes: pathNodes });
}

function clearPath() {
  pathSegments = [];
  pathNodes = [];
  pendingFreePoint = null;
  selectedSegmentId = null;
  syncTrajectoryPanel();
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

function handleCanvasClick(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const [x, y] = canvasToWorld((clientX - rect.left) * scaleX, (clientY - rect.top) * scaleY);

  if (pendingPoiDraft) {
    const manualGeo = readManualGeoInput(false);
    if (!manualGeo) {
      return;
    }
    const draft = pendingPoiDraft;
    const poi = {
      clientId: `poi-${poiIdSeed++}`,
      name: draft.name,
      x,
      y,
      yaw: Number.isFinite(draft.yaw) ? Number(draft.yaw) : Number(lastPose.yaw || 0),
      lat: draft.lat ?? manualGeo.lat ?? lastGps.lat,
      lon: draft.lon ?? manualGeo.lon ?? lastGps.lon,
    };
    poiNodes.push(poi);
    pendingPoiDraft = null;
    elements.poiNameInput.value = "";
    callApi("/map/poi", {
      poi: {
        name: poi.name,
        x: Number(poi.x),
        y: Number(poi.y),
        lat: Number.isFinite(poi.lat) ? Number(poi.lat) : null,
        lon: Number.isFinite(poi.lon) ? Number(poi.lon) : null,
      },
    }).catch((err) => {
      alert(`POI failed: ${err.message}`);
    });
    startNextPoiDraft();
    syncTrajectoryPanel();
    return;
  }

  const hitSegment = findSegmentAt(x, y);
  if (hitSegment) {
    selectedSegmentId = hitSegment.id;
    syncTrajectoryPanel();
    return;
  }

  const hitPoi = findPoiAt(x, y);
  if (hitPoi) {
    togglePoiSelection(hitPoi.clientId);
    return;
  }

  if (elements.trajectoryToolMode.value === "free") {
    const point = makeLocalPoint(x, y, { lat: lastGps.lat, lon: lastGps.lon });
    if (!pendingFreePoint) {
      pendingFreePoint = point;
      selectedSegmentId = null;
      syncTrajectoryPanel();
      return;
    }
    addSegment(createSegment(pendingFreePoint, point, {
      source: "free",
    }));
    pendingFreePoint = null;
    syncTrajectoryPanel();
  }
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
      handleCanvasClick(event.clientX, event.clientY);
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
    if (cell.hits < scanMergeConfig.saveMinHits || cell.hits < freeWeight * 0.9) {
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
  if (!drawState.showPath || (!pathSegments.length && !pendingFreePoint)) {
    return;
  }
  pathSegments.forEach((segment) => {
    const samples = sampleSegment(segment);
    if (!samples.length) {
      return;
    }
    ctx.strokeStyle = segment.id === selectedSegmentId ? "#ff7b54" : "#f3b441";
    ctx.lineWidth = segment.id === selectedSegmentId ? 4 : 2;
    ctx.beginPath();
    samples.forEach((node, index) => {
      const [sx, sy] = worldToCanvas(node.x, node.y);
      if (index === 0) {
        ctx.moveTo(sx, sy);
      } else {
        ctx.lineTo(sx, sy);
      }
    });
    ctx.stroke();
  });

  if (pendingFreePoint) {
    const [sx, sy] = worldToCanvas(pendingFreePoint.x, pendingFreePoint.y);
    ctx.save();
    ctx.strokeStyle = "#4fd1c5";
    ctx.setLineDash([6, 4]);
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(sx, sy, 8, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }
}

function drawPoiOverlay() {
  if (!drawState.showPoi) {
    return;
  }
  poiNodes.forEach((poi) => {
    const [sx, sy] = worldToCanvas(poi.x, poi.y);
    ctx.fillStyle = selectedPoiIds.has(poi.clientId) ? "#7c3aed" : "#d94a4a";
    ctx.beginPath();
    ctx.arc(sx, sy, 5, 0, Math.PI * 2);
    ctx.fill();
    if (selectedPoiIds.has(poi.clientId)) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(sx, sy, 9, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.fillStyle = "#ffffff";
    ctx.font = "12px Segoe UI";
    ctx.fillText(poi.name, sx + 8, sy - 8);
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
  ctx.fillStyle = "#13766e";
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
  drawState.showPath = elements.showPathToggle.checked;
  drawState.showPoi = elements.showPoiToggle.checked;
  drawState.showRobot = elements.showRobotToggle.checked;
  drawState.showMotionControlCard = elements.showMotionControlCardToggle.checked;
  drawState.showPoiCard = elements.showPoiCardToggle.checked;
  drawState.showTrajectoryCard = elements.showTrajectoryCardToggle.checked;
  drawState.showStcmInspectorCard = elements.showStcmInspectorCardToggle.checked;
  elements.motionControlCard.hidden = !drawState.showMotionControlCard;
  elements.poiCard.hidden = !drawState.showPoiCard;
  elements.trajectoryCard.hidden = !drawState.showTrajectoryCard;
  elements.stcmInspectorCard.hidden = !drawState.showStcmInspectorCard;
  elements.drawModeBadge.innerText = "Line";
  updateTrajectoryStatus();
  syncTrajectoryButtons();
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
  elements.addPoiBtn.onclick = togglePoiPlacement;
  elements.applyPoiGeoBtn.onclick = applyGeoToSelectedPoi;
  elements.copyPoiBtn.onclick = () => copyPoiData().catch((err) => alert(`Copy failed: ${err.message}`));
  elements.clearPoiBtn.onclick = clearPoi;
  elements.autoConnectBtn.onclick = autoConnectPoiLoop;
  elements.connectSelectedPoiBtn.onclick = buildSegmentFromSelectedPoi;
  elements.deleteSegmentBtn.onclick = removeSelectedSegment;
  document.getElementById("centerViewBtn").onclick = centerViewOnRobot;
  document.getElementById("resetViewBtn").onclick = resetView;
  elements.refreshCameraBtn.onclick = refreshCameraSnapshot;
  elements.inspectStcmBtn.onclick = () => {
    inspectSelectedStcm().catch((err) => {
      setInspectorStatus("Error", "error");
      elements.stcmSummaryPanel.innerText = JSON.stringify({ error: err.message }, null, 2);
      elements.stcmManifestPanel.innerText = "-";
    });
  };
  elements.downloadPgmBtn.onclick = () => {
    if (!stcmInspector.pgmText || !stcmInspector.file) {
      alert("Inspect a STCM file first");
      return;
    }
    downloadFile(stcmInspector.file.name.replace(/\.stcm$/i, ".pgm"), stcmInspector.pgmText, "image/x-portable-graymap");
  };
  elements.downloadStcmJsonBtn.onclick = () => {
    if (!stcmInspector.configJsonText || !stcmInspector.file) {
      alert("Inspect a STCM file first");
      return;
    }
    downloadFile(stcmInspector.file.name.replace(/\.stcm$/i, ".json"), stcmInspector.configJsonText, "application/json");
  };
  elements.stcmFileInput.addEventListener("change", () => {
    const file = elements.stcmFileInput.files && elements.stcmFileInput.files[0];
    stcmInspector.file = file || null;
    stcmInspector.bundle = null;
    stcmInspector.points = [];
    stcmInspector.pgmText = "";
    stcmInspector.configJsonText = "";
    stcmInspector.summary = null;
    setInspectorStatus("Idle");
    elements.stcmFileMeta.innerText = file ? `${file.name} | ${(file.size / 1024).toFixed(1)} KB` : "No file selected";
    elements.stcmGridMeta.innerText = "No grid generated";
    elements.stcmSummaryPanel.innerText = "-";
    elements.stcmManifestPanel.innerText = "-";
  });

  elements.trajectoryToolMode.addEventListener("change", () => {
    pendingFreePoint = null;
    updateTrajectoryStatus();
    syncTrajectoryButtons();
  });
  elements.poiBatchCreateInput.addEventListener("input", syncTrajectoryButtons);
  elements.poiGeoInput.addEventListener("input", syncTrajectoryPanel);
  elements.showPathToggle.addEventListener("change", syncDrawControls);
  elements.showPoiToggle.addEventListener("change", syncDrawControls);
  elements.showRobotToggle.addEventListener("change", syncDrawControls);
  elements.showMotionControlCardToggle.addEventListener("change", syncDrawControls);
  elements.showPoiCardToggle.addEventListener("change", syncDrawControls);
  elements.showTrajectoryCardToggle.addEventListener("change", syncDrawControls);
  elements.showStcmInspectorCardToggle.addEventListener("change", syncDrawControls);

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
syncTrajectoryPanel();
renderCamera();
renderScanState();
renderOdomScanPanel();
resetView();
updateCameraRefreshStatus();
updateSavePathHint();
setInspectorStatus("Idle");
draw();
