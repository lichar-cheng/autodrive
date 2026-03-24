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
  yamlText: "",
  exportJsonText: "",
  summary: null,
  loadedFileName: "",
};

const drawState = {
  showPath: true,
  showPoi: true,
  showRobot: true,
  showMotionControlCard: true,
  showPoiCard: true,
  showTrajectoryCard: true,
  showStcmInspectorCard: true,
};

const editState = {
  tool: "view",
  pendingObstacleStart: null,
  erasing: false,
  loadedFromStcm: false,
  loadedMapName: "",
};

const pathValidation = {
  checked: false,
  ok: null,
  invalidSegmentIds: new Set(),
  message: "",
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
  pathSafetyMarginInput: document.getElementById("pathSafetyMarginInput"),
  pathStartPoiField: document.getElementById("pathStartPoiField"),
  pathEndPoiField: document.getElementById("pathEndPoiField"),
  pathApplyField: document.getElementById("pathApplyField"),
  pathStartPoiInput: document.getElementById("pathStartPoiInput"),
  pathEndPoiInput: document.getElementById("pathEndPoiInput"),
  freeConnectHint: document.getElementById("freeConnectHint"),
  clearSelectionBtn: document.getElementById("clearSelectionBtn"),
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
  connectNamedPoiBtn: document.getElementById("connectNamedPoiBtn"),
  validatePathBtn: document.getElementById("validatePathBtn"),
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
  loadStcmToCanvasBtn: document.getElementById("loadStcmToCanvasBtn"),
  downloadPgmBtn: document.getElementById("downloadPgmBtn"),
  downloadYamlBtn: document.getElementById("downloadYamlBtn"),
  downloadStcmJsonBtn: document.getElementById("downloadStcmJsonBtn"),
  mapEditToolMode: document.getElementById("mapEditToolMode"),
  editBrushRadiusInput: document.getElementById("editBrushRadiusInput"),
  autoClearNoiseBtn: document.getElementById("autoClearNoiseBtn"),
  clearEditorMapBtn: document.getElementById("clearEditorMapBtn"),
  mapEditStatus: document.getElementById("mapEditStatus"),
  mapLoadedBadge: document.getElementById("mapLoadedBadge"),
  editorToolBadge: document.getElementById("editorToolBadge"),
  editorStatsBadge: document.getElementById("editorStatsBadge"),
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
    throw new Error("Invalid SLAM/ZIP file: EOCD not found");
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

function buildPgmFromData(manifest, points, resolution, paddingCells) {
  const browserOccupancy = manifest.browser_occupancy;
  const occupancyVoxel = Math.max(0.02, Number(browserOccupancy?.voxel_size || resolution));
  const occupiedCells = Array.isArray(browserOccupancy?.occupied_cells) ? browserOccupancy.occupied_cells : null;

  let minCellX = Number.POSITIVE_INFINITY;
  let maxCellX = Number.NEGATIVE_INFINITY;
  let minCellY = Number.POSITIVE_INFINITY;
  let maxCellY = Number.NEGATIVE_INFINITY;
  const occupiedSet = new Set();

  if (occupiedCells && occupiedCells.length) {
    occupiedCells.forEach((cell) => {
      const ix = Math.round(Number(cell.ix || 0));
      const iy = Math.round(Number(cell.iy || 0));
      minCellX = Math.min(minCellX, ix);
      maxCellX = Math.max(maxCellX, ix);
      minCellY = Math.min(minCellY, iy);
      maxCellY = Math.max(maxCellY, iy);
      occupiedSet.add(`${ix}:${iy}`);
    });
  } else {
    if (!points.length) {
      throw new Error("No radar points in SLAM");
    }
    points.forEach(([x, y]) => {
      const ix = Math.round(Number(x) / resolution);
      const iy = Math.round(Number(y) / resolution);
      minCellX = Math.min(minCellX, ix);
      maxCellX = Math.max(maxCellX, ix);
      minCellY = Math.min(minCellY, iy);
      maxCellY = Math.max(maxCellY, iy);
      occupiedSet.add(`${ix}:${iy}`);
    });
  }

  const paddedMinCellX = minCellX - paddingCells;
  const paddedMinCellY = minCellY - paddingCells;
  const paddedMaxCellX = maxCellX + paddingCells;
  const paddedMaxCellY = maxCellY + paddingCells;
  const width = Math.max(1, paddedMaxCellX - paddedMinCellX + 1);
  const height = Math.max(1, paddedMaxCellY - paddedMinCellY + 1);
  const grid = new Uint8Array(width * height).fill(205);

  occupiedSet.forEach((key) => {
    const [cellXText, cellYText] = key.split(":");
    const ix = Number(cellXText) - paddedMinCellX;
    const iy = Number(cellYText) - paddedMinCellY;
    const flippedY = height - 1 - iy;
    grid[flippedY * width + ix] = 0;
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

  const origin = [
    Number((paddedMinCellX * occupancyVoxel).toFixed(3)),
    Number((paddedMinCellY * occupancyVoxel).toFixed(3)),
    0,
  ];
  return {
    pgmText: `P2\n# Generated from SLAM occupancy\n${width} ${height}\n255\n${rows.join("\n")}\n`,
    width,
    height,
    origin,
    occupiedCells: occupiedSet.size,
    bounds: {
      minX: Number((minCellX * occupancyVoxel).toFixed(3)),
      maxX: Number((maxCellX * occupancyVoxel).toFixed(3)),
      minY: Number((minCellY * occupancyVoxel).toFixed(3)),
      maxY: Number((maxCellY * occupancyVoxel).toFixed(3)),
    },
  };
}

function buildYamlText(fileName, resolution, origin) {
  return [
    `image: ${fileName.replace(/\.slam$/i, ".pgm")}`,
    "mode: trinary",
    `resolution: ${resolution.toFixed(3)}`,
    `origin: [${origin[0].toFixed(3)}, ${origin[1].toFixed(3)}, ${Number(origin[2] || 0).toFixed(0)}]`,
    "negate: 0",
    "occupied_thresh: 0.65",
    "free_thresh: 0.196",
  ].join("\n");
}

function buildStcmExports(fileName, manifest, points, resolution, paddingCells) {
  const pgm = buildPgmFromData(manifest, points, resolution, paddingCells);
  const yamlText = buildYamlText(fileName, resolution, pgm.origin);
  const exportManifest = { ...manifest };
  delete exportManifest.browser_occupancy;
  delete exportManifest.trajectory;
  const exportJson = {
    source_file: fileName,
    map_yaml: {
      image: fileName.replace(/\.slam$/i, ".pgm"),
      mode: "trinary",
      resolution,
      origin: pgm.origin,
      negate: 0,
      occupied_thresh: 0.65,
      free_thresh: 0.196,
    },
    pgm_meta: {
      width: pgm.width,
      height: pgm.height,
      occupied_cells: pgm.occupiedCells,
      bounds: pgm.bounds,
    },
    manifest: exportManifest,
  };
  return {
    pgm,
    yamlText,
    exportJsonText: JSON.stringify(exportJson, null, 2),
  };
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

function setInspectorBundleState(fileName, manifest, points, summary = {}) {
  const resolution = Math.max(0.02, numberFromInput(elements.stcmResolutionInput, 0.1));
  const paddingCells = Math.max(0, Math.round(numberFromInput(elements.stcmPaddingInput, 8)));
  const { pgm, yamlText, exportJsonText } = buildStcmExports(fileName, manifest, points, resolution, paddingCells);

  stcmInspector.file = { name: fileName };
  stcmInspector.bundle = manifest;
  stcmInspector.points = points;
  stcmInspector.pgmText = pgm.pgmText;
  stcmInspector.yamlText = yamlText;
  stcmInspector.exportJsonText = exportJsonText;
  stcmInspector.summary = {
    file: fileName,
    manifestVersion: manifest.version || "unknown",
    mapSource: manifest.map_source || "unknown",
    radarPoints: points.length,
    hasBrowserOccupancy: Boolean(manifest.browser_occupancy),
    restoredFreeCells: Array.isArray(manifest.browser_occupancy?.free_cells) ? manifest.browser_occupancy.free_cells.length : 0,
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
    ...summary,
  };
  elements.stcmFileMeta.innerText = fileName;
  elements.stcmGridMeta.innerText = `${pgm.width} x ${pgm.height} | ${pgm.occupiedCells} occupied | res ${resolution.toFixed(2)} m/px`;
  elements.stcmSummaryPanel.innerText = JSON.stringify(stcmInspector.summary, null, 2);
  elements.stcmManifestPanel.innerText = JSON.stringify(manifest, null, 2);
  setInspectorStatus("Ready", "active");
}

async function inspectSelectedStcm() {
  const file = elements.stcmFileInput.files && elements.stcmFileInput.files[0];
  if (!file) {
    throw new Error("Please choose a .slam file first");
  }

  setInspectorStatus("Parsing", "active");
  elements.stcmFileMeta.innerText = `${file.name} | ${(file.size / 1024).toFixed(1)} KB`;
  const arrayBuffer = await file.arrayBuffer();
  const entries = parseZipEntries(arrayBuffer);
  const manifestEntry = entries.get("manifest.json");
  const radarEntry = entries.get("radar_points.bin");
  if (!manifestEntry || !radarEntry) {
    throw new Error("SLAM must contain manifest.json and radar_points.bin");
  }

  const manifestBytes = await decompressZipEntry(manifestEntry);
  const radarBytes = await decompressZipEntry(radarEntry);
  const manifest = JSON.parse(new TextDecoder().decode(manifestBytes));
  const points = parseRadarPoints(radarBytes);
  setInspectorBundleState(file.name, manifest, points);
}

function resetEditorToolState(nextTool = editState.tool) {
  editState.tool = nextTool;
  editState.pendingObstacleStart = null;
  editState.erasing = false;
}

function brushRadiusInMeters() {
  return Math.max(0.05, numberFromInput(elements.editBrushRadiusInput, 0.25));
}

function setMapEditStatus(text, level = "") {
  elements.mapEditStatus.innerText = text;
  elements.mapEditStatus.className = `hint ${level}`.trim();
}

function updateEditorBadges() {
  const sourceLabel = editState.loadedFromStcm
    ? `Loaded: ${editState.loadedMapName || stcmInspector.loadedFileName || "SLAM map"}`
    : "Scan Session";
  elements.mapLoadedBadge.innerText = sourceLabel;

  const toolLabel = editState.tool === "erase"
    ? "Tool: Erase Noise"
    : editState.tool === "obstacle"
      ? `Tool: Draw Obstacle${editState.pendingObstacleStart ? " | Pick end point" : ""}`
      : "Tool: View / Select";
  elements.editorToolBadge.innerText = toolLabel;
  elements.editorStatsBadge.innerText = `${scanSession.occupiedCells.size} obstacle cells | ${poiNodes.length} POI | ${pathSegments.length} paths`;
}

function setOccupiedCell(ix, iy, hits = scanMergeConfig.saveMinHits, intensity = 1) {
  scanSession.occupiedCells.set(cellKey(ix, iy), { ix, iy, hits, intensity });
}

function removeCellsInRadius(worldX, worldY, radiusMeters) {
  const radiusCells = Math.max(1, Math.round(radiusMeters / scanSession.voxelSize));
  const [cx, cy] = worldToCell(worldX, worldY);
  for (let dx = -radiusCells; dx <= radiusCells; dx += 1) {
    for (let dy = -radiusCells; dy <= radiusCells; dy += 1) {
      const dist = Math.hypot(dx, dy) * scanSession.voxelSize;
      if (dist > radiusMeters) {
        continue;
      }
      const key = cellKey(cx + dx, cy + dy);
      scanSession.occupiedCells.delete(key);
      scanSession.freeCells.delete(key);
    }
  }
  renderScanState();
  updateEditorBadges();
}

function rasterizeObstacleLine(startPoint, endPoint) {
  const [startIx, startIy] = worldToCell(startPoint.x, startPoint.y);
  const [endIx, endIy] = worldToCell(endPoint.x, endPoint.y);
  const cells = rasterizeLineCells(startIx, startIy, endIx, endIy);
  cells.forEach(({ ix, iy }) => {
    setOccupiedCell(ix, iy);
  });
  renderScanState();
  updateEditorBadges();
}

function autoClearNoise() {
  const removable = [];
  for (const cell of scanSession.occupiedCells.values()) {
    if (cell.hits > scanMergeConfig.saveMinHits) {
      continue;
    }
    let neighbors = 0;
    for (let dx = -1; dx <= 1; dx += 1) {
      for (let dy = -1; dy <= 1; dy += 1) {
        if (dx === 0 && dy === 0) {
          continue;
        }
        if (scanSession.occupiedCells.has(cellKey(cell.ix + dx, cell.iy + dy))) {
          neighbors += 1;
        }
      }
    }
    if (neighbors <= 1) {
      removable.push(cellKey(cell.ix, cell.iy));
    }
  }
  removable.forEach((key) => scanSession.occupiedCells.delete(key));
  renderScanState();
  updateEditorBadges();
  setMapEditStatus(removable.length ? `Auto cleared ${removable.length} noisy cells` : "No isolated noise found", removable.length ? "active" : "");
}

function rebuildSegmentsFromBundle(manifest) {
  segmentIdSeed = 1;
  pathSegments = [];
  const paths = Array.isArray(manifest.path) ? manifest.path : [];
  paths.forEach((item) => {
    const points = Array.isArray(item.points) ? item.points : [];
    const start = item.start || points[0];
    const end = item.end || points[points.length - 1];
    if (!start || !end) {
      return;
    }
    pathSegments.push(createSegment(start, end, {
      source: item.source || "slam",
      clearance: Number(item.clearance || 0),
      points: points.length ? points : [start, end],
    }));
  });
  if (!pathSegments.length) {
    const nodes = Array.isArray(manifest.path) ? manifest.path : [];
    for (let i = 0; i < nodes.length - 1; i += 1) {
      pathSegments.push(createSegment(nodes[i], nodes[i + 1], { source: "slam" }));
    }
  }
  segmentIdSeed = pathSegments.length + 1;
}

function loadStcmIntoEditor() {
  if (!stcmInspector.bundle || !Array.isArray(stcmInspector.points)) {
    alert("Inspect a SLAM file first");
    return;
  }

  clearAccumulation();
  scanSession.active = false;
  scanSession.voxelSize = Math.max(0.02, numberFromInput(elements.voxelSizeInput, 0.12));
  const browserOccupancy = stcmInspector.bundle.browser_occupancy;
  if (browserOccupancy && Array.isArray(browserOccupancy.occupied_cells)) {
    scanSession.voxelSize = Math.max(0.02, Number(browserOccupancy.voxel_size || scanSession.voxelSize));
    browserOccupancy.occupied_cells.forEach((cell) => {
      setOccupiedCell(
        Number(cell.ix || 0),
        Number(cell.iy || 0),
        Number(cell.hits || scanMergeConfig.saveMinHits),
        Number(cell.intensity || 1),
      );
    });
    if (Array.isArray(browserOccupancy.free_cells)) {
      browserOccupancy.free_cells.forEach((cell) => {
        scanSession.freeCells.set(cellKey(Number(cell.ix || 0), Number(cell.iy || 0)), {
          ix: Number(cell.ix || 0),
          iy: Number(cell.iy || 0),
          hits: Number(cell.hits || 1),
        });
      });
    }
  } else {
    stcmInspector.points.forEach((point) => {
      const [ix, iy] = worldToCell(Number(point[0] || 0), Number(point[1] || 0));
      setOccupiedCell(ix, iy, scanMergeConfig.saveMinHits, Number(point[2] || 1));
    });
  }

  poiIdSeed = 1;
  poiNodes = (Array.isArray(stcmInspector.bundle.poi) ? stcmInspector.bundle.poi : []).map((poi) => ({
    clientId: `poi-${poiIdSeed++}`,
    name: poi.name || `POI ${poiIdSeed}`,
    x: Number(poi.x || 0),
    y: Number(poi.y || 0),
    yaw: poi.yaw === null || poi.yaw === undefined || poi.yaw === "" ? 0 : Number(poi.yaw),
    lat: poi.lat === null || poi.lat === undefined || poi.lat === "" ? null : Number(poi.lat),
    lon: poi.lon === null || poi.lon === undefined || poi.lon === "" ? null : Number(poi.lon),
  }));
  selectedPoiIds = new Set();
  pendingPoiDraft = null;
  pendingPoiQueue = [];
  rebuildSegmentsFromBundle(stcmInspector.bundle);
  selectedSegmentId = null;

  editState.loadedFromStcm = true;
  editState.loadedMapName = stcmInspector.file ? stcmInspector.file.name : "slam";
  stcmInspector.loadedFileName = editState.loadedMapName;
  if (elements.mapEditToolMode) {
    elements.mapEditToolMode.value = "view";
  }
  resetEditorToolState("view");
  elements.mapNameInput.value = editState.loadedMapName.replace(/\.slam$/i, "") || elements.mapNameInput.value;
  if (typeof stcmInspector.bundle.notes === "string" && stcmInspector.bundle.notes.trim()) {
    elements.mapNotesInput.value = stcmInspector.bundle.notes;
  }
  if (stcmInspector.summary && Number.isFinite(Number(stcmInspector.summary.resolution))) {
    elements.stcmResolutionInput.value = Number(stcmInspector.summary.resolution).toFixed(2);
  }

  syncTrajectoryPanel();
  renderScanState();
  centerViewOnLoadedMap();
  updateEditorBadges();
  setMapEditStatus(`Loaded ${editState.loadedMapName} into main map view`, "active");
}

function centerViewOnLoadedMap() {
  const points = Array.from(scanSession.occupiedCells.values());
  if (!points.length) {
    centerViewOnRobot();
    return;
  }
  let minX = points[0].ix * scanSession.voxelSize;
  let maxX = minX;
  let minY = points[0].iy * scanSession.voxelSize;
  let maxY = minY;
  points.forEach((cell) => {
    const x = cell.ix * scanSession.voxelSize;
    const y = cell.iy * scanSession.voxelSize;
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
  });
  viewState.panX = -((minX + maxX) / 2) * viewState.scale;
  viewState.panY = ((minY + maxY) / 2) * viewState.scale;
  updateViewMetrics();
}

function clearEditorMap() {
  clearAccumulation();
  poiNodes = [];
  pathSegments = [];
  pathNodes = [];
  selectedPoiIds = new Set();
  selectedSegmentId = null;
  editState.loadedFromStcm = false;
  editState.loadedMapName = "";
  if (elements.mapEditToolMode) {
    elements.mapEditToolMode.value = "view";
  }
  resetEditorToolState("view");
  syncTrajectoryPanel();
  updateEditorBadges();
  setMapEditStatus("Loaded map cleared", "warning");
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
    loadedFromStcm: editState.loadedFromStcm,
    loadedMapName: editState.loadedMapName || null,
    editTool: editState.tool,
  };
  const browserOccupancy = {
    voxel_size: voxelSize,
    occupied_cells: Array.from(scanSession.occupiedCells.values()).map((cell) => ({
      ix: Number(cell.ix),
      iy: Number(cell.iy),
      hits: Number(cell.hits || scanMergeConfig.saveMinHits),
      intensity: Number(cell.intensity || 1),
    })),
    free_cells: Array.from(scanSession.freeCells.values()).map((cell) => ({
      ix: Number(cell.ix),
      iy: Number(cell.iy),
      hits: Number(cell.hits || 1),
    })),
  };

  return {
    version: "stcm.v2",
    notes: JSON.stringify(notes, null, 2),
    created_at: Date.now() / 1000,
    source: "browser",
    map_source: editState.loadedFromStcm ? "stcm_editor" : "laser_accumulation",
    browser_occupancy: browserOccupancy,
    pose: lastPose,
    gps: lastGps,
    chassis: lastChassis,
    poi: poiNodes.map((poi) => sanitizePoiPayload(poi)),
    path: pathSegments.map((segment) => ({
      id: segment.id,
      source: segment.source,
      clearance: Number(segment.clearance || 0),
      start: sanitizePathNode(segment.start),
      end: sanitizePathNode(segment.end),
      points: sampleSegment(segment),
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
          description: "SLAM map",
          accept: { "application/octet-stream": [".slam"] },
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
  updateEditorBadges();
}

function startScanSession() {
  clearAccumulation();
  editState.loadedFromStcm = false;
  editState.loadedMapName = "";
  scanSession.active = true;
  scanSession.startedAtMs = Date.now();
  renderScanState();
  updateEditorBadges();
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
    mapSource: editState.loadedFromStcm ? "loaded_stcm" : "live_scan",
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
    updateEditorBadges();
    return;
  }
  if (occupiedCount > 0 || freeCount > 0) {
    setScanState(`Stopped | ${occupiedCount} obs / ${freeCount} safe`, "warning");
    updateEditorBadges();
    return;
  }
  setScanState("Idle");
  updateEditorBadges();
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
    poiId: node.poiId || null,
    name: node.name || "",
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

function polylineLength(points) {
  let total = 0;
  for (let i = 0; i < points.length - 1; i += 1) {
    total += distanceBetween(points[i], points[i + 1]);
  }
  return total;
}

function readPathSafetyMargin() {
  return Math.max(0, numberFromInput(elements.pathSafetyMarginInput, 0.3));
}

function simplifyCellPath(path) {
  if (path.length <= 2) {
    return path;
  }
  const simplified = [path[0]];
  let prevDx = path[1].ix - path[0].ix;
  let prevDy = path[1].iy - path[0].iy;
  for (let i = 1; i < path.length - 1; i += 1) {
    const dx = path[i + 1].ix - path[i].ix;
    const dy = path[i + 1].iy - path[i].iy;
    if (dx !== prevDx || dy !== prevDy) {
      simplified.push(path[i]);
      prevDx = dx;
      prevDy = dy;
    }
  }
  simplified.push(path[path.length - 1]);
  return simplified;
}

function isCellBlocked(ix, iy, clearanceCells = 0, allowStartKey = "", allowEndKey = "") {
  for (let dx = -clearanceCells; dx <= clearanceCells; dx += 1) {
    for (let dy = -clearanceCells; dy <= clearanceCells; dy += 1) {
      if (Math.hypot(dx, dy) > clearanceCells + 1e-6) {
        continue;
      }
      const key = cellKey(ix + dx, iy + dy);
      if (key === allowStartKey || key === allowEndKey) {
        continue;
      }
      if (scanSession.occupiedCells.has(key)) {
        return true;
      }
    }
  }
  return false;
}

function planPathCells(start, end, clearanceMeters) {
  const [startIx, startIy] = worldToCell(start.x, start.y);
  const [endIx, endIy] = worldToCell(end.x, end.y);
  const startKey = cellKey(startIx, startIy);
  const endKey = cellKey(endIx, endIy);
  const clearanceCells = Math.max(0, Math.ceil(clearanceMeters / scanSession.voxelSize));
  const occupiedCells = Array.from(scanSession.occupiedCells.values());
  const occupiedMinX = occupiedCells.length ? Math.min(...occupiedCells.map((cell) => cell.ix)) : Math.min(startIx, endIx);
  const occupiedMaxX = occupiedCells.length ? Math.max(...occupiedCells.map((cell) => cell.ix)) : Math.max(startIx, endIx);
  const occupiedMinY = occupiedCells.length ? Math.min(...occupiedCells.map((cell) => cell.iy)) : Math.min(startIy, endIy);
  const occupiedMaxY = occupiedCells.length ? Math.max(...occupiedCells.map((cell) => cell.iy)) : Math.max(startIy, endIy);

  if (!isCellBlocked(startIx, startIy, clearanceCells, startKey, endKey)
    && !isCellBlocked(endIx, endIy, clearanceCells, startKey, endKey)) {
    const linePath = rasterizeLineCells(startIx, startIy, endIx, endIy);
    const lineBlocked = linePath.some((cell) => isCellBlocked(cell.ix, cell.iy, clearanceCells, startKey, endKey));
    if (!lineBlocked) {
      return simplifyCellPath(linePath);
    }
  }

  const minX = Math.min(startIx, endIx, occupiedMinX) - clearanceCells - 20;
  const maxX = Math.max(startIx, endIx, occupiedMaxX) + clearanceCells + 20;
  const minY = Math.min(startIy, endIy, occupiedMinY) - clearanceCells - 20;
  const maxY = Math.max(startIy, endIy, occupiedMaxY) + clearanceCells + 20;
  const open = [{ ix: startIx, iy: startIy, key: startKey, g: 0, f: Math.hypot(endIx - startIx, endIy - startIy) }];
  const best = new Map([[startKey, open[0]]]);
  const cameFrom = new Map();
  const closed = new Set();
  const neighbors = [
    { dx: 1, dy: 0, cost: 1 },
    { dx: -1, dy: 0, cost: 1 },
    { dx: 0, dy: 1, cost: 1 },
    { dx: 0, dy: -1, cost: 1 },
    { dx: 1, dy: 1, cost: Math.SQRT2 },
    { dx: 1, dy: -1, cost: Math.SQRT2 },
    { dx: -1, dy: 1, cost: Math.SQRT2 },
    { dx: -1, dy: -1, cost: Math.SQRT2 },
  ];

  while (open.length) {
    open.sort((a, b) => a.f - b.f);
    const current = open.shift();
    if (!current || closed.has(current.key)) {
      continue;
    }
    if (current.key === endKey) {
      const path = [{ ix: endIx, iy: endIy }];
      let cursor = endKey;
      while (cameFrom.has(cursor)) {
        cursor = cameFrom.get(cursor);
        const [ixText, iyText] = cursor.split(":");
        path.push({ ix: Number(ixText), iy: Number(iyText) });
      }
      path.reverse();
      return simplifyCellPath(path);
    }
    closed.add(current.key);

    neighbors.forEach((step) => {
      const nextIx = current.ix + step.dx;
      const nextIy = current.iy + step.dy;
      if (nextIx < minX || nextIx > maxX || nextIy < minY || nextIy > maxY) {
        return;
      }
      const nextKey = cellKey(nextIx, nextIy);
      if (closed.has(nextKey)) {
        return;
      }
      if (isCellBlocked(nextIx, nextIy, clearanceCells, startKey, endKey)) {
        return;
      }
      if (step.dx !== 0 && step.dy !== 0) {
        if (isCellBlocked(current.ix + step.dx, current.iy, clearanceCells, startKey, endKey)
          || isCellBlocked(current.ix, current.iy + step.dy, clearanceCells, startKey, endKey)) {
          return;
        }
      }
      const g = current.g + step.cost;
      const prev = best.get(nextKey);
      if (prev && g >= prev.g) {
        return;
      }
      const node = {
        ix: nextIx,
        iy: nextIy,
        key: nextKey,
        g,
        f: g + Math.hypot(endIx - nextIx, endIy - nextIy),
      };
      cameFrom.set(nextKey, current.key);
      best.set(nextKey, node);
      open.push(node);
    });
  }
  return null;
}

function cellPathToWorldPoints(pathCells, start, end) {
  if (!pathCells || !pathCells.length) {
    return [makeLocalPoint(start.x, start.y, start), makeLocalPoint(end.x, end.y, end)];
  }
  return pathCells.map((cell, index) => {
    if (index === 0) {
      return makeLocalPoint(start.x, start.y, start);
    }
    if (index === pathCells.length - 1) {
      return makeLocalPoint(end.x, end.y, end);
    }
    return makeLocalPoint(cell.ix * scanSession.voxelSize, cell.iy * scanSession.voxelSize);
  });
}

function rasterizeLineCells(startIx, startIy, endIx, endIy) {
  const points = [];
  const pushCell = (ix, iy) => {
    const prev = points[points.length - 1];
    if (!prev || prev.ix !== ix || prev.iy !== iy) {
      points.push({ ix, iy });
    }
  };

  let x = startIx;
  let y = startIy;
  const dx = endIx - startIx;
  const dy = endIy - startIy;
  const nx = Math.abs(dx);
  const ny = Math.abs(dy);
  const signX = Math.sign(dx);
  const signY = Math.sign(dy);
  let ix = 0;
  let iy = 0;

  pushCell(x, y);
  while (ix < nx || iy < ny) {
    const nextHorizontal = (0.5 + ix) / Math.max(nx, 1);
    const nextVertical = (0.5 + iy) / Math.max(ny, 1);
    if (nextHorizontal < nextVertical) {
      x += signX;
      ix += 1;
    } else if (nextVertical < nextHorizontal) {
      y += signY;
      iy += 1;
    } else {
      x += signX;
      y += signY;
      ix += 1;
      iy += 1;
    }
    pushCell(x, y);
  }
  return points;
}

function buildPlannedSegment(start, end, options = {}) {
  const clearance = options.clearance ?? readPathSafetyMargin();
  const cellPath = planPathCells(start, end, clearance);
  if (!cellPath) {
    throw new Error(`No obstacle-free path found from ${describePoint(start)} to ${describePoint(end)} with clearance ${clearance.toFixed(2)} m`);
  }
  return createSegment(start, end, {
    ...options,
    points: cellPathToWorldPoints(cellPath, start, end),
    clearance,
  });
}

function createSegment(start, end, options = {}) {
  const points = Array.isArray(options.points) && options.points.length
    ? options.points.map((point, index) => {
      if (index === 0) {
        return makeLocalPoint(start.x, start.y, { ...start, ...point });
      }
      if (index === options.points.length - 1) {
        return makeLocalPoint(end.x, end.y, { ...end, ...point });
      }
      return makeLocalPoint(point.x, point.y, point);
    })
    : [makeLocalPoint(start.x, start.y, start), makeLocalPoint(end.x, end.y, end)];
  return {
    id: `seg-${segmentIdSeed++}`,
    start: makeLocalPoint(start.x, start.y, start),
    end: makeLocalPoint(end.x, end.y, end),
    geometry: "line",
    curveOffset: 0,
    source: options.source || "free",
    clearance: Number(options.clearance || 0),
    points,
  };
}

function sampleSegment(segment) {
  if (!segment) {
    return [];
  }
  const points = Array.isArray(segment.points) && segment.points.length
    ? segment.points
    : [segment.start, segment.end];
  return points.map((point) => sanitizePathNode(point));
}

function rebuildPathNodes() {
  pathNodes = [];
  pathSegments.forEach((segment) => {
    const samples = sampleSegment(segment);
    samples.forEach((point, index) => {
      const prev = pathNodes[pathNodes.length - 1];
      if (index > 0 && prev && pathNodeKey(prev) === pathNodeKey(point)) {
        return;
      }
      pathNodes.push(point);
    });
  });
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

function clearPendingPoiPlacement() {
  pendingPoiDraft = null;
  pendingPoiQueue = [];
}

function startManualPoiDraft(name, geo) {
  clearPendingPoiPlacement();
  pendingPoiDraft = {
    name,
    lat: geo.lat,
    lon: geo.lon,
    yaw: null,
  };
  syncTrajectoryPanel();
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

function resetPathValidation() {
  pathValidation.checked = false;
  pathValidation.ok = null;
  pathValidation.invalidSegmentIds = new Set();
  pathValidation.message = "";
}

function pathNodeKey(point) {
  return `${Number(point.x).toFixed(3)},${Number(point.y).toFixed(3)}`;
}

function pathValidationTolerance() {
  return Math.max(0.15, scanSession.voxelSize * 1.5);
}

function describeValidationPoint(point) {
  if (point.name) {
    return `${point.name} ${describePoint(point)}`;
  }
  return describePoint(point);
}

function resolveValidationNode(point, clusters) {
  const poiKey = point.poiId ? `poi:${point.poiId}` : "";
  if (poiKey) {
    const match = clusters.find((cluster) => cluster.poiKey === poiKey);
    if (match) {
      return match;
    }
  }

  const tolerance = pathValidationTolerance();
  const match = clusters.find((cluster) => {
    if (poiKey || cluster.poiKey) {
      return false;
    }
    return Math.hypot(cluster.x - point.x, cluster.y - point.y) <= tolerance;
  });
  if (match) {
    return match;
  }

  const cluster = {
    id: poiKey || `node:${clusters.length + 1}`,
    poiKey,
    x: Number(point.x),
    y: Number(point.y),
    labels: new Set(),
  };
  clusters.push(cluster);
  return cluster;
}

function normalizePoiName(name) {
  return String(name || "").trim().toLowerCase();
}

function findPoiByName(name) {
  const target = normalizePoiName(name);
  if (!target) {
    return { poi: null, error: "Input both POI names first" };
  }
  const matches = poiNodes.filter((poi) => normalizePoiName(poi.name) === target);
  if (!matches.length) {
    return { poi: null, error: `POI "${name}" not found` };
  }
  if (matches.length > 1) {
    return { poi: null, error: `POI name "${name}" is duplicated` };
  }
  return { poi: matches[0], error: "" };
}

function computePathClosedLoopValidation() {
  resetPathValidation();
  pathValidation.checked = true;

  if (pathSegments.length < 3) {
    pathValidation.ok = false;
    pathValidation.invalidSegmentIds = new Set(pathSegments.map((segment) => segment.id));
    pathValidation.message = "Closed-loop check failed: at least 3 path segments are required.";
    return false;
  }

  const clusters = [];
  const endpointMap = new Map();
  const adjacency = new Map();

  pathSegments.forEach((segment) => {
    const startNode = resolveValidationNode(segment.start, clusters);
    const endNode = resolveValidationNode(segment.end, clusters);
    startNode.labels.add(describeValidationPoint(segment.start));
    endNode.labels.add(describeValidationPoint(segment.end));
    const startKey = startNode.id;
    const endKey = endNode.id;
    if (!endpointMap.has(startKey)) {
      endpointMap.set(startKey, []);
    }
    if (!endpointMap.has(endKey)) {
      endpointMap.set(endKey, []);
    }
    endpointMap.get(startKey).push(segment.id);
    endpointMap.get(endKey).push(segment.id);

    if (!adjacency.has(startKey)) {
      adjacency.set(startKey, new Set());
    }
    if (!adjacency.has(endKey)) {
      adjacency.set(endKey, new Set());
    }
    adjacency.get(startKey).add(endKey);
    adjacency.get(endKey).add(startKey);
  });

  const invalidSegmentIds = new Set();
  const badNodes = [];
  endpointMap.forEach((segmentIds, key) => {
    if (segmentIds.length !== 2) {
      const node = clusters.find((item) => item.id === key);
      badNodes.push({
        key,
        degree: segmentIds.length,
        labels: node ? Array.from(node.labels) : [key],
      });
      segmentIds.forEach((id) => invalidSegmentIds.add(id));
    }
  });

  const keys = Array.from(adjacency.keys());
  const visited = new Set();
  const components = [];
  keys.forEach((startKey) => {
    if (visited.has(startKey)) {
      return;
    }
    const stack = [startKey];
    const component = new Set();
    while (stack.length) {
      const node = stack.pop();
      if (visited.has(node)) {
        continue;
      }
      visited.add(node);
      component.add(node);
      (adjacency.get(node) || []).forEach((nextNode) => {
        if (!visited.has(nextNode)) {
          stack.push(nextNode);
        }
      });
    }
    components.push(component);
  });

  if (components.length !== 1) {
    pathSegments.forEach((segment) => invalidSegmentIds.add(segment.id));
  }

  pathValidation.ok = invalidSegmentIds.size === 0;
  pathValidation.invalidSegmentIds = invalidSegmentIds;

  if (pathValidation.ok) {
    pathValidation.message = "Closed-loop check passed.";
  } else {
    const reasonParts = [];
    if (badNodes.length) {
      const details = badNodes
        .slice(0, 8)
        .map((node) => `${node.labels[0]} degree=${node.degree}`)
        .join("; ");
      reasonParts.push(`${badNodes.length} endpoint(s) do not have degree 2: ${details}`);
    }
    if (components.length !== 1) {
      reasonParts.push(`path is split into ${components.length} disconnected component(s)`);
    }
    pathValidation.message = `Closed-loop check failed: ${reasonParts.join("; ")}.`;
  }

  return pathValidation.ok;
}

function validatePathClosedLoop(showAlert = true) {
  computePathClosedLoopValidation();
  selectedSegmentId = null;
  selectedPoiIds = new Set();

  if (showAlert) {
    alert(pathValidation.message);
  }
  resetPathValidation();
  syncTrajectoryPanel();
  return pathValidation.ok;
}

function clearSelections() {
  selectedSegmentId = null;
  selectedPoiIds = new Set();
  pendingFreePoint = null;
  resetPathValidation();
  syncTrajectoryPanel();
}

function syncTrajectoryButtons() {
  const selectedSegment = pathSegments.find((segment) => segment.id === selectedSegmentId) || null;
  const idleMode = elements.trajectoryToolMode.value === "idle";
  const nameMode = elements.trajectoryToolMode.value === "poi";
  const hasNamedPoiPair = elements.pathStartPoiInput.value.trim() && elements.pathEndPoiInput.value.trim();
  elements.pathStartPoiField.hidden = !nameMode;
  elements.pathEndPoiField.hidden = !nameMode;
  elements.pathApplyField.hidden = !nameMode;
  elements.pathStartPoiInput.disabled = !nameMode;
  elements.pathEndPoiInput.disabled = !nameMode;
  elements.deleteSegmentBtn.disabled = !selectedSegment;
  elements.connectNamedPoiBtn.disabled = !nameMode || !hasNamedPoiPair;
  elements.connectNamedPoiBtn.hidden = !nameMode;
  elements.clearSelectionBtn.disabled = !selectedSegment && selectedPoiIds.size === 0 && !pendingFreePoint;
  elements.validatePathBtn.disabled = pathSegments.length === 0;
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
      li.innerHTML = `<span class="list-main">No path segments yet</span><span class="list-meta">Auto-connect POI or add one with the current tool</span>`;
    }
    elements.pathList.appendChild(li);
    return;
  }

  pathSegments.forEach((segment, index) => {
    const li = document.createElement("li");
    const samples = sampleSegment(segment);
    if (segment.id === selectedSegmentId) {
      li.classList.add("selected");
    }
    if (pathValidation.invalidSegmentIds.has(segment.id)) {
      li.classList.add("invalid");
    }
    li.innerHTML = `
      <span class="list-main">${index + 1}. line | ${segment.source} | ${describePoint(segment.start)} -> ${describePoint(segment.end)}</span>
      <span class="list-meta">Length ${polylineLength(samples).toFixed(2)} m | key points ${samples.length}${segment.clearance ? ` | clearance ${segment.clearance.toFixed(2)} m` : ""}${pathValidation.invalidSegmentIds.has(segment.id) ? " | closed-loop error" : ""}</span>
    `;
    li.onclick = () => {
      selectedSegmentId = segment.id;
      syncTrajectoryPanel();
    };
    elements.pathList.appendChild(li);
  });
}

function updateTrajectoryStatus() {
  const toolLabel = elements.trajectoryToolMode.value === "idle"
    ? "Browse only"
    : elements.trajectoryToolMode.value === "poi"
    ? `Use POI${elements.pathStartPoiInput.value.trim() || elements.pathEndPoiInput.value.trim() ? ` (${elements.pathStartPoiInput.value.trim() || "?"} -> ${elements.pathEndPoiInput.value.trim() || "?"})` : ""}`
    : "Free points on canvas";
  const pendingText = pendingFreePoint ? ` | Start ${describePoint(pendingFreePoint)}` : "";
  const safetyText = ` | Clearance ${readPathSafetyMargin().toFixed(2)} m`;
  const validationText = !pathValidation.checked
    ? " | Loop unchecked"
    : pathValidation.ok
      ? " | Loop OK"
      : ` | Loop error ${pathValidation.invalidSegmentIds.size} segment(s)`;
  elements.trajectoryStatus.innerText = `Path segments ${pathSegments.length} | Nodes ${pathNodes.length} | Tool ${toolLabel}${safetyText}${pendingText}${validationText}`;
}

function syncTrajectoryPanel() {
  rebuildPathNodes();
  if (pathValidation.checked) {
    computePathClosedLoopValidation();
  }
  renderPoiList();
  renderSegmentList();
  updateTrajectoryStatus();
  elements.freeConnectHint.innerText = elements.trajectoryToolMode.value === "idle"
    ? "Browse only mode: clicking the map will only select or clear selection."
    : elements.trajectoryToolMode.value === "free"
    ? pendingFreePoint
      ? `Free points mode: second click will apply the path. Start point ${describePoint(pendingFreePoint)}.`
      : "Free points mode: click any two map points to apply a path."
    : "Use POI mode: input start/end POI then click Apply Path.";
  elements.poiStatus.innerText = pendingPoiDraft
    ? `Ready to place "${pendingPoiDraft.name}" on canvas${pendingPoiQueue.length ? ` | ${pendingPoiQueue.length} queued` : ""}`
    : pendingPoiQueue.length
      ? `${pendingPoiQueue.length} POI queued. Click Add POI to start`
      : "POI idle";
  syncTrajectoryButtons();
  updateEditorBadges();
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
  const name = elements.poiNameInput.value.trim();
  if (pendingPoiDraft || pendingPoiQueue.length) {
    if (name) {
      const geo = readManualGeoInput(false);
      if (!geo) {
        return;
      }
      startManualPoiDraft(name, geo);
      return;
    }
    clearPendingPoiPlacement();
    syncTrajectoryPanel();
    return;
  }
  if (name) {
    const geo = readManualGeoInput(false);
    if (!geo) {
      return;
    }
    startManualPoiDraft(name, geo);
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
  alert("Input POI name first, or provide batch POI input.");
  elements.poiNameInput.focus();
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

function connectNamedPoi() {
  const startName = elements.pathStartPoiInput.value.trim();
  const endName = elements.pathEndPoiInput.value.trim();
  if (!startName || !endName) {
    alert("Input both POI names first");
    return;
  }
  const startResult = findPoiByName(startName);
  if (!startResult.poi) {
    alert(startResult.error);
    return;
  }
  const endResult = findPoiByName(endName);
  if (!endResult.poi) {
    alert(endResult.error);
    return;
  }
  if (startResult.poi.clientId === endResult.poi.clientId) {
    alert("Start and end POI cannot be the same");
    return;
  }
  try {
    addSegment(buildPlannedSegment(startResult.poi, endResult.poi, {
      source: "poi-name",
    }));
  } catch (err) {
    alert(err.message);
  }
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
  const preserved = pathSegments.filter((segment) => segment.source !== "auto");
  const nextAutoSegments = [];
  try {
    for (let i = 0; i < route.length - 1; i += 1) {
      nextAutoSegments.push(buildPlannedSegment(route[i], route[i + 1], { source: "auto", geometry: "line" }));
    }
    if (route.length > 2) {
      nextAutoSegments.push(buildPlannedSegment(route[route.length - 1], route[0], { source: "auto", geometry: "line" }));
    }
  } catch (err) {
    alert(err.message);
    return;
  }
  pathSegments = [...preserved, ...nextAutoSegments];
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
  const savedName = await saveBlobWithPicker(`${name}.slam`, new Blob([zipBytes], { type: "application/octet-stream" }));
  scanSession.lastSavedFile = savedName;
  scanSession.savedPointCount = bundle.radar_points.length;
  setInspectorBundleState(savedName.endsWith(".slam") ? savedName : `${name}.slam`, manifest, bundle.radar_points, {
    source: "current_map_save",
  });
  updateSavePathHint(scanSession.lastSavedFile);
  renderScanState();
  alert(`Map saved: ${savedName}`);
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

function clientToWorld(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return canvasToWorld((clientX - rect.left) * scaleX, (clientY - rect.top) * scaleY);
}

function handleCanvasClick(clientX, clientY) {
  const [x, y] = clientToWorld(clientX, clientY);

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
    if (ws && ws.readyState === WebSocket.OPEN) {
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
    }
    startNextPoiDraft();
    syncTrajectoryPanel();
    return;
  }

  if (editState.tool === "erase") {
    removeCellsInRadius(x, y, brushRadiusInMeters());
    setMapEditStatus(`Erased map cells near (${x.toFixed(2)}, ${y.toFixed(2)})`, "active");
    return;
  }

  if (editState.tool === "obstacle") {
    if (!editState.pendingObstacleStart) {
      editState.pendingObstacleStart = { x, y };
      updateEditorBadges();
      setMapEditStatus(`Obstacle start fixed at (${x.toFixed(2)}, ${y.toFixed(2)}). Click end point next.`, "active");
      return;
    }
    rasterizeObstacleLine(editState.pendingObstacleStart, { x, y });
    setMapEditStatus(`Added obstacle line from (${editState.pendingObstacleStart.x.toFixed(2)}, ${editState.pendingObstacleStart.y.toFixed(2)}) to (${x.toFixed(2)}, ${y.toFixed(2)})`, "active");
    editState.pendingObstacleStart = null;
    updateEditorBadges();
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
    try {
      addSegment(buildPlannedSegment(pendingFreePoint, point, {
        source: "free",
      }));
      pendingFreePoint = null;
      syncTrajectoryPanel();
    } catch (err) {
      alert(err.message);
    }
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

  if (selectedSegmentId || selectedPoiIds.size || pendingFreePoint) {
    clearSelections();
  }
}

function bindCanvasInteractions() {
  canvas.addEventListener("pointerdown", (event) => {
    if (editState.tool === "erase") {
      canvas.setPointerCapture(event.pointerId);
      viewState.dragging = false;
      editState.erasing = true;
      const [x, y] = clientToWorld(event.clientX, event.clientY);
      removeCellsInRadius(x, y, brushRadiusInMeters());
      canvas.classList.remove("dragging");
      return;
    }

    if (editState.tool !== "view") {
      canvas.setPointerCapture(event.pointerId);
      viewState.dragging = true;
      viewState.moved = false;
      return;
    }

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
    if (editState.erasing) {
      const [x, y] = clientToWorld(event.clientX, event.clientY);
      removeCellsInRadius(x, y, brushRadiusInMeters());
      return;
    }
    if (editState.tool !== "view") {
      return;
    }
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
    if (editState.erasing) {
      editState.erasing = false;
      canvas.releasePointerCapture(event.pointerId);
      setMapEditStatus("Noise eraser finished", "active");
      return;
    }
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
    if (editState.erasing) {
      editState.erasing = false;
    }
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
    if (scanSession.occupiedCells.has(cellKey(cell.ix, cell.iy))) {
      continue;
    }
    const worldX = cell.ix * scanSession.voxelSize;
    const worldY = cell.iy * scanSession.voxelSize;
    const [sx, sy] = worldToCanvas(worldX, worldY);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(sx - sizePx / 2, sy - sizePx / 2, sizePx, sizePx);
  }

  for (const cell of scanSession.occupiedCells.values()) {
    const worldX = cell.ix * scanSession.voxelSize;
    const worldY = cell.iy * scanSession.voxelSize;
    const [sx, sy] = worldToCanvas(worldX, worldY);
    ctx.fillStyle = "#0c0f12";
    ctx.fillRect(sx - sizePx / 2, sy - sizePx / 2, sizePx, sizePx);
  }
}

function drawEditorOverlay() {
  if (editState.tool !== "obstacle" || !editState.pendingObstacleStart) {
    return;
  }
  const [sx, sy] = worldToCanvas(editState.pendingObstacleStart.x, editState.pendingObstacleStart.y);
  ctx.save();
  ctx.strokeStyle = "#101214";
  ctx.fillStyle = "#101214";
  ctx.lineWidth = 3;
  ctx.setLineDash([8, 6]);
  ctx.beginPath();
  ctx.arc(sx, sy, 9, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(sx, sy, 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
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
    const isInvalid = pathValidation.invalidSegmentIds.has(segment.id);
    ctx.strokeStyle = isInvalid ? "#cc4b37" : segment.id === selectedSegmentId ? "#ff7b54" : "#f3b441";
    ctx.lineWidth = isInvalid ? 4 : segment.id === selectedSegmentId ? 4 : 2;
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
    ctx.font = "15px Segoe UI";
    const textWidth = ctx.measureText(poi.name).width;
    ctx.fillStyle = "rgba(255, 245, 214, 0.92)";
    ctx.fillRect(sx + 5, sy - 24, textWidth + 12, 20);
    ctx.fillStyle = "#182833";
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
  elements.drawModeBadge.innerText = "Path Line";
  updateTrajectoryStatus();
  syncTrajectoryButtons();
  updateEditorBadges();
}

function draw() {
  ctx.fillStyle = "#8f969c";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawOccupancy();
  drawEditorOverlay();
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
  elements.addPoiBtn.onclick = togglePoiPlacement;
  elements.applyPoiGeoBtn.onclick = applyGeoToSelectedPoi;
  elements.copyPoiBtn.onclick = () => copyPoiData().catch((err) => alert(`Copy failed: ${err.message}`));
  elements.clearPoiBtn.onclick = clearPoi;
  elements.autoConnectBtn.onclick = autoConnectPoiLoop;
  elements.connectNamedPoiBtn.onclick = connectNamedPoi;
  elements.validatePathBtn.onclick = () => validatePathClosedLoop(true);
  elements.clearSelectionBtn.onclick = clearSelections;
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
  elements.loadStcmToCanvasBtn.onclick = loadStcmIntoEditor;
  elements.autoClearNoiseBtn.onclick = autoClearNoise;
  elements.clearEditorMapBtn.onclick = clearEditorMap;
  elements.downloadPgmBtn.onclick = () => {
    if (!stcmInspector.pgmText || !stcmInspector.file) {
      alert("Inspect a SLAM file first");
      return;
    }
    downloadFile(stcmInspector.file.name.replace(/\.slam$/i, ".pgm"), stcmInspector.pgmText, "image/x-portable-graymap");
  };
  elements.downloadYamlBtn.onclick = () => {
    if (!stcmInspector.yamlText || !stcmInspector.file) {
      alert("Inspect a SLAM file first");
      return;
    }
    downloadFile(stcmInspector.file.name.replace(/\.slam$/i, ".yaml"), stcmInspector.yamlText, "application/x-yaml");
  };
  elements.downloadStcmJsonBtn.onclick = () => {
    if (!stcmInspector.exportJsonText || !stcmInspector.file) {
      alert("Inspect a SLAM file first");
      return;
    }
    downloadFile(stcmInspector.file.name.replace(/\.slam$/i, ".json"), stcmInspector.exportJsonText, "application/json");
  };
  elements.stcmFileInput.addEventListener("change", () => {
    const file = elements.stcmFileInput.files && elements.stcmFileInput.files[0];
    stcmInspector.file = file || null;
    stcmInspector.bundle = null;
    stcmInspector.points = [];
    stcmInspector.pgmText = "";
    stcmInspector.yamlText = "";
    stcmInspector.exportJsonText = "";
    stcmInspector.summary = null;
    stcmInspector.loadedFileName = "";
    setInspectorStatus("Idle");
    elements.stcmFileMeta.innerText = file ? `${file.name} | ${(file.size / 1024).toFixed(1)} KB` : "No file selected";
    elements.stcmGridMeta.innerText = "No grid generated";
    elements.stcmSummaryPanel.innerText = "-";
    elements.stcmManifestPanel.innerText = "-";
  });
  elements.mapEditToolMode.addEventListener("change", () => {
    resetEditorToolState(elements.mapEditToolMode.value);
    updateEditorBadges();
    if (editState.tool === "erase") {
      setMapEditStatus(`Erase Noise mode active. Brush radius ${brushRadiusInMeters().toFixed(2)} m`, "active");
    } else if (editState.tool === "obstacle") {
      setMapEditStatus("Draw Obstacle Line mode active. Click two points on the map.", "active");
    } else {
      setMapEditStatus("View / Select mode active. You can pan, zoom, and select POI or path.", "active");
    }
  });
  elements.editBrushRadiusInput.addEventListener("input", () => {
    if (editState.tool === "erase") {
      setMapEditStatus(`Erase Noise brush radius ${brushRadiusInMeters().toFixed(2)} m`, "active");
    }
  });

  elements.trajectoryToolMode.addEventListener("change", () => {
    pendingFreePoint = null;
    selectedSegmentId = null;
    resetPathValidation();
    syncTrajectoryPanel();
  });
  elements.pathStartPoiInput.addEventListener("input", syncTrajectoryPanel);
  elements.pathEndPoiInput.addEventListener("input", syncTrajectoryPanel);
  elements.pathSafetyMarginInput.addEventListener("input", syncTrajectoryPanel);
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
  window.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      resetPathValidation();
      clearSelections();
    }
  });
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
resetEditorToolState(elements.mapEditToolMode ? elements.mapEditToolMode.value : "view");
updateEditorBadges();
setMapEditStatus("View / Select mode active. Load a SLAM file to start second-stage map editing.");
draw();
