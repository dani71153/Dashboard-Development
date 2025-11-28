document.addEventListener('DOMContentLoaded', () => {
// ========================
// 1. Estado e integración con ROS (roslibjs)
// ========================
const rosStatus   = document.getElementById('ros-status');
const topicStatus = document.getElementById('topic-status');
const datosRos    = document.getElementById('datos-ros');
const slamStatusDot   = document.getElementById('slam-status-dot');
const slamStatusText  = document.getElementById('slam-status-text');
const slamLastUpdate  = document.getElementById('slam-last-update');
const mapaCanvas      = document.getElementById('mapa-canvas');
const slamMapMeta     = document.getElementById('slam-map-meta');
const slamPoseOutput  = document.getElementById('slam-pose-output');
const slamServiceFeedback = document.getElementById('slam-service-feedback');
const dynamicMapButton = document.getElementById('btn-request-dynamic-map');
const xyPoiForm = document.getElementById('xy-poi-form');
const xyPoiList = document.getElementById('xy-poi-list');
const xyUseCurrentBtn = document.getElementById('xy-poi-use-current');
const xyNameInput = document.getElementById('xy-poi-name');
const xyXInput = document.getElementById('xy-poi-x');
const xyYInput = document.getElementById('xy-poi-y');
const xyYawInput = document.getElementById('xy-poi-yaw');
const xyvizCanvas = document.getElementById('xyviz-canvas');
const xyvizPlaceholder = document.getElementById('xyviz-placeholder');
const xyvizAxisToggle = document.getElementById('xyviz-axis-toggle');
const xyvizResetViewBtn = document.getElementById('xyviz-reset-view');
const xyvizLandmarkForm = document.getElementById('xyviz-landmark-form');
const xyvizZoomInBtn = document.getElementById('xyviz-zoom-in');
const xyvizZoomOutBtn = document.getElementById('xyviz-zoom-out');
const xyvizLandmarkNameInput = document.getElementById('xyviz-landmark-name');
const xyvizLandmarkXInput = document.getElementById('xyviz-landmark-x');
const xyvizLandmarkYInput = document.getElementById('xyviz-landmark-y');
const xyvizLandmarkYawInput = document.getElementById('xyviz-landmark-yaw');
const xyvizUseCurrentBtn = document.getElementById('xyviz-use-current');
const xyvizLandmarkList = document.getElementById('xyviz-landmark-list');
const xyvizSelectedName = document.getElementById('xyviz-selected-name');
const xyvizSelectedDistance = document.getElementById('xyviz-selected-distance');
const xyvizSelectedDx = document.getElementById('xyviz-selected-dx');
const xyvizSelectedDy = document.getElementById('xyviz-selected-dy');
const xyvizSelectedDyaw = document.getElementById('xyviz-selected-dyaw');
const xyvizSelectedDxError = document.getElementById('xyviz-selected-dx-error');
const xyvizSelectedDyError = document.getElementById('xyviz-selected-dy-error');
const xyvizSelectedDyawError = document.getElementById('xyviz-selected-dyaw-error');
const xyvizRobotPoseEl = document.getElementById('xyviz-robot-pose');
const xyvizLastUpdateEl = document.getElementById('xyviz-last-update');
const navImuTemp = document.getElementById('nav-imu-temp');
const lidarStatusChip = document.getElementById('lidar-status-chip');
const lidarStatusText = document.getElementById('lidar-status-text');
const lidarStartBtn = document.getElementById('btn-lidar-start');
const lidarStopBtn = document.getElementById('btn-lidar-stop');
const calibDesiredXInput = document.getElementById('calib-desired-x');
const calibDesiredYInput = document.getElementById('calib-desired-y');
const calibDesiredThetaInput = document.getElementById('calib-desired-theta');
const calibOffsetXInput = document.getElementById('calib-offset-x');
const calibOffsetYInput = document.getElementById('calib-offset-y');
const calibOffsetThetaInput = document.getElementById('calib-offset-theta');
const calibToggleAngleModeBtn = document.getElementById('calib-toggle-angle-mode');
const calibAngleModeHint = document.getElementById('calib-angle-mode-hint');
const calibDesiredThetaLabel = document.getElementById('calib-desired-theta-label');
const calibOffsetThetaLabel = document.getElementById('calib-offset-theta-label');
const calibEncodersStatusChip = document.getElementById('calib-encoders-status');
const calibFilteredStatusChip = document.getElementById('calib-filtered-status');
const calibImuStatusChip = document.getElementById('calib-imu-status');
const navxSection = document.getElementById('navegacion');
const navxAsideButtons = document.querySelectorAll('.navx-io-btn');
const navxSalidasPanel = document.getElementById('nav-io-salidas');
const navxEntradasPanel = document.getElementById('nav-io-entradas');
const navxSalidasTabs = document.querySelectorAll('.navx-tab-btn');
const navxPanelMap = {
  velocidades: document.getElementById('nav-salidas-velocidades'),
  frecuencia: document.getElementById('nav-salidas-frecuencia'),
  aceleraciones: document.getElementById('nav-salidas-aceleraciones'),
  consola: document.getElementById('nav-salidas-consola')
};
const navxAccelToggleHighBtn = document.getElementById('nav-accel-toggle-high');
const navxFreqToggleBtn = document.getElementById('nav-freq-toggle-mode');
const navxVelModeLinearBtn = document.getElementById('nav-vel-mode-linear');
const navxVelModeAngularBtn = document.getElementById('nav-vel-mode-angular');

const SLAM_HEARTBEAT_TIMEOUT_MS = 6000;
let slamHeartbeatTimer = null;
let slamMapMonitor = null;
let slamPoseMonitor = null;
let dynamicMapService = null;
const XY_SCALE_PX_PER_M = 80;
const XY_GRID_STEP_METERS = 1;
const XY_MAJOR_GRID_EVERY = 5;
const XY_POI_ARROW_SIZE_M = 0.45;
let xyCanvas = null;
let xyCtx = null;
let xyResizeObserver = null;
let xyOrigin = { x: 0, y: 0 };
let xyPose = { x: 0, y: 0, yaw: 0, hasPose: false };
let xyPois = [];
let xyPoiId = 0;
let xyWindowResizeHandlerRegistered = false;
let xyCanvasWidth = 0;
let xyCanvasHeight = 0;
const XYVIZ_SCALE_PX_PER_M = 80;
const XYVIZ_GRID_STEP_METERS = 1;
const XYVIZ_MAJOR_GRID_EVERY = 5;
const XYVIZ_TRAIL_LIMIT = 400;
const XYVIZ_COLORS = ['#38bdf8', '#f97316', '#a855f7', '#10b981', '#f87171', '#eab308'];
const LIDAR_TOPIC_NAME = '/scan';
const LIDAR_HEARTBEAT_TIMEOUT_MS = 6000;
const IMU_TEMP_TOPIC_NAME = '/imu/temp';
const IMU_TEMP_TIMEOUT_MS = 7000;
let lidarScanMonitor = null;
let lidarHeartbeatTimer = null;
let lidarStartService = null;
let lidarStopService = null;
let lidarState = 'off';
let lidarActionInFlight = false;
let imuTempMonitor = null;
let imuTempTimer = null;
let navImuLastTemp = null;
let calibEncodersMonitor = null;
let calibFilteredMonitor = null;
let calibImuMonitor = null;
const CALIB_HEARTBEAT_TIMEOUT_MS = 6000;
const xyvizState = {
  canvas: xyvizCanvas,
  ctx: xyvizCanvas ? xyvizCanvas.getContext('2d') : null,
  wrap: xyvizCanvas ? xyvizCanvas.parentElement : null,
  resizeObserver: null,
  windowResizeHandlerRegistered: false,
  axisMode: 'standard',
  scale: XYVIZ_SCALE_PX_PER_M,
  width: 0,
  height: 0,
  origin: { x: 0, y: 0 },
  panOffset: { x: 0, y: 0 },
  robotPose: { x: 0, y: 0, yaw: 0, hasPose: false },
  trail: [],
  landmarks: [],
  nextLandmarkId: 1,
  selectedId: null,
  isPanning: false,
  panStart: { x: 0, y: 0 },
  panOriginSnapshot: { x: 0, y: 0 }
};
let xyvizOdomMonitor = null;
let xyvizInitialized = false;
const calibracionData = {
  encoders: { x: null, y: null, theta: null, stamp: null },
  filtered: { x: null, y: null, theta: null, stamp: null },
  imu: {
    theta: null,
    angVel: { x: null, y: null, z: null },
    linAcc: { x: null, y: null, z: null },
    stamp: null
  }
};
const calibElements = {
  encoders: {
    stamp: document.getElementById('calib-encoders-stamp'),
    xValue: document.getElementById('calib-encoders-x'),
    xDiff: document.getElementById('calib-encoders-x-diff'),
    xError: document.getElementById('calib-encoders-x-error'),
    yValue: document.getElementById('calib-encoders-y'),
    yDiff: document.getElementById('calib-encoders-y-diff'),
    yError: document.getElementById('calib-encoders-y-error'),
    thetaValue: document.getElementById('calib-encoders-theta'),
    thetaDiff: document.getElementById('calib-encoders-theta-diff'),
    thetaError: document.getElementById('calib-encoders-theta-error'),
    distance: document.getElementById('calib-encoders-distance')
  },
  filtered: {
    stamp: document.getElementById('calib-filtered-stamp'),
    xValue: document.getElementById('calib-filtered-x'),
    xDiff: document.getElementById('calib-filtered-x-diff'),
    xError: document.getElementById('calib-filtered-x-error'),
    yValue: document.getElementById('calib-filtered-y'),
    yDiff: document.getElementById('calib-filtered-y-diff'),
    yError: document.getElementById('calib-filtered-y-error'),
    thetaValue: document.getElementById('calib-filtered-theta'),
    thetaDiff: document.getElementById('calib-filtered-theta-diff'),
    thetaError: document.getElementById('calib-filtered-theta-error'),
    distance: document.getElementById('calib-filtered-distance')
  },
  imu: {
    stamp: document.getElementById('calib-imu-stamp'),
    thetaValue: document.getElementById('calib-imu-theta'),
    thetaDiff: document.getElementById('calib-imu-theta-diff'),
    thetaError: document.getElementById('calib-imu-theta-error'),
    angX: document.getElementById('calib-imu-ang-x'),
    angY: document.getElementById('calib-imu-ang-y'),
    angZ: document.getElementById('calib-imu-ang-z'),
    linX: document.getElementById('calib-imu-lin-x'),
    linY: document.getElementById('calib-imu-lin-y'),
    linZ: document.getElementById('calib-imu-lin-z')
  }
};
const CALIB_EPSILON = 1e-6;
let calibAngleMode = 'rad';
const calibStatus = {
  encoders: { el: calibEncodersStatusChip, timer: null },
  filtered: { el: calibFilteredStatusChip, timer: null },
  imu: { el: calibImuStatusChip, timer: null }
};
const NAVX_MAX_POINTS = 120;
const NAVX_ODOM_MAX_POINTS = 180;
const NAVX_ODOM_JUMP_THRESHOLD = 0.15;
const NAVX_ACCEL_MAX_POINTS = 180;
const NAVX_VEL_TOPIC_CONFIGS = [
  {
    key: 'cmdVel',
    canvasId: 'nav-chart-cmd-vel',
    topic: '/cmd_vel',
    colorLinear: '#2563eb',
    colorAngular: '#f97316',
    typeHint: 'geometry_msgs/msg/Twist'
  },
  {
    key: 'cmdVelSmoothed',
    canvasId: 'nav-chart-cmd-vel-smoothed',
    topic: '/cmd_vel_smoothed',
    colorLinear: '#0ea5e9',
    colorAngular: '#f97316',
    typeHint: 'geometry_msgs/msg/TwistStamped'
  },
  {
    key: 'cmdVelTeleop',
    canvasId: 'nav-chart-cmd-vel-teleop',
    topic: '/cmd_vel_teleop',
    colorLinear: '#22c55e',
    colorAngular: '#f97316',
    typeHint: 'geometry_msgs/msg/TwistStamped'
  },
  {
    key: 'diffCmdVel',
    canvasId: 'nav-chart-diff-cmd-vel',
    topic: '/diff_drive_controller/cmd_vel',
    colorLinear: '#a855f7',
    colorAngular: '#f97316',
    typeHint: 'geometry_msgs/msg/TwistStamped'
  }
];
const navxState = {
  salidasActive: false,
  activePanel: 'velocidades',
  velocity: {
    charts: {},
    topics: {},
    startTimes: {},
    topicTypes: {},
    mode: 'linear'
  },
  frequency: {
    chart: null,
    topic: null,
    startStamp: null,
    lastStamp: null,
    mode: 'delta', // 'delta' (segundos) o 'hz'
    history: []
  },
  acceleration: {
    charts: {},
    topic: null,
    startStamp: null,
    lastStamp: null,
    wheelNames: [],
    wheelState: [],
    highWheelsEnabled: true
  }
};

if (navxAsideButtons.length) {
  navxAsideButtons.forEach(button => {
    button.addEventListener('click', () => {
      const target = button.dataset.target;
      navxAsideButtons.forEach(btn => btn.classList.remove('active'));
      button.classList.add('active');

      if (navxSalidasPanel) navxSalidasPanel.style.display = target === 'salidas' ? 'flex' : 'none';
      if (navxEntradasPanel) navxEntradasPanel.style.display = target === 'entradas' ? 'flex' : 'none';

      updateNavegacionMonitoring();
    });
  });
}

if (navxSalidasTabs.length) {
  navxSalidasTabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const panel = tab.dataset.panel;
      navxSalidasTabs.forEach(btn => btn.classList.remove('active'));
      tab.classList.add('active');
      Object.entries(navxPanelMap).forEach(([key, panelEl]) => {
        if (!panelEl) return;
        panelEl.classList.toggle('active', key === panel);
      });
      navxState.activePanel = panel;
      updateNavPanelSubscriptions();
    });
  });
}

if (navxVelModeLinearBtn && navxVelModeAngularBtn) {
  navxVelModeLinearBtn.addEventListener('click', () => navxSetVelocityMode('linear'));
  navxVelModeAngularBtn.addEventListener('click', () => navxSetVelocityMode('angular'));
  navxSetVelocityMode(navxState.velocity.mode);
}

if (navxFreqToggleBtn) {
  navxFreqToggleBtn.addEventListener('click', () => {
    navxState.frequency.mode = navxState.frequency.mode === 'delta' ? 'hz' : 'delta';
    navxApplyFrequencyMode();
  });
  navxApplyFrequencyMode();
}

if (navxAccelToggleHighBtn) {
  navxAccelToggleHighBtn.addEventListener('click', () => {
    navxState.acceleration.highWheelsEnabled = !navxState.acceleration.highWheelsEnabled;
    navxApplyAccelerationVisibility();
  });
  navxApplyAccelerationVisibility();
}

if (window.Chart && window['chartjsPluginAnnotation']) {
  Chart.register(window['chartjsPluginAnnotation']);
}

if (rosStatus)   rosStatus.textContent = '🔴 Desconectado';
if (topicStatus) topicStatus.textContent = '—';
if (slamStatusDot) {
  slamStatusDot.classList.remove('status-running', 'status-disconnected', 'status-idle');
  slamStatusDot.classList.add('status-unknown');
}
if (slamStatusText) slamStatusText.textContent = 'SLAMToolbox: Sin datos';
if (slamLastUpdate) slamLastUpdate.textContent = 'Última actualización: —';
if (slamServiceFeedback) slamServiceFeedback.style.color = '#9e9e9e';

function setSlamIndicator(state, message) {
  if (slamStatusDot) {
    slamStatusDot.classList.remove('status-running', 'status-disconnected', 'status-idle', 'status-unknown');
    let cssClass = 'status-unknown';
    if (state === 'connected' || state === 'idle') {
      cssClass = 'status-running';
    } else if (state === 'disconnected') {
      cssClass = 'status-disconnected';
    }
    slamStatusDot.classList.add(cssClass);
  }
  if (slamStatusText && typeof message === 'string') {
    slamStatusText.textContent = message;
  }
}

function setSlamLastUpdate(date) {
  if (!slamLastUpdate) return;
  if (!date) {
    slamLastUpdate.textContent = 'Última actualización: —';
    return;
  }
  const formato = date.toLocaleTimeString('es-ES', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
  });
  slamLastUpdate.textContent = `Última actualización: ${formato}`;
}

function markSlamHeartbeat(origen) {
  setSlamIndicator('connected', `SLAMToolbox: Conectado (${origen})`);
  setSlamLastUpdate(new Date());
  if (slamHeartbeatTimer) clearTimeout(slamHeartbeatTimer);
  slamHeartbeatTimer = setTimeout(() => {
    setSlamIndicator('idle', 'SLAMToolbox: Conectado (sin datos recientes)');
  }, SLAM_HEARTBEAT_TIMEOUT_MS);
}

function startSlamMonitoring() {
  if (!window.ros || !window.ros.isConnected) return;
  if (!slamMapMonitor) {
    slamMapMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
      queue_length: 1,
      throttle_rate: 2000
    });
    slamMapMonitor.subscribe(msg => {
      markSlamHeartbeat('mapa');
      updateMapMetadata(msg);
    });
  }
  if (!slamPoseMonitor) {
    slamPoseMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: '/slam_toolbox/pose',
      messageType: 'geometry_msgs/PoseStamped',
      queue_length: 1,
      throttle_rate: 1500
    });
    slamPoseMonitor.subscribe(msg => {
      markSlamHeartbeat('pose');
      updatePosePanel(msg);
    });
  }
  setSlamIndicator('idle', 'SLAMToolbox: Conectado (esperando datos)');
  setSlamLastUpdate(null);
}

function stopSlamMonitoring() {
  if (slamHeartbeatTimer) {
    clearTimeout(slamHeartbeatTimer);
    slamHeartbeatTimer = null;
  }
  if (slamMapMonitor) {
    slamMapMonitor.unsubscribe();
    slamMapMonitor = null;
  }
  if (slamPoseMonitor) {
    slamPoseMonitor.unsubscribe();
    slamPoseMonitor = null;
  }
  if (slamMapMeta) {
    slamMapMeta.textContent = 'Dimensiones: — · Resolución: — · Origen: —';
  }
  if (slamPoseOutput) {
    slamPoseOutput.textContent = 'Sin datos';
  }
  clearXYRobotPose();
}

function renderMapaPlaceholder(texto) {
  if (!mapaCanvas) return;
  let placeholder = mapaCanvas.querySelector('.mapa-placeholder');
  if (!placeholder) {
    placeholder = document.createElement('div');
    placeholder.className = 'mapa-placeholder';
    mapaCanvas.appendChild(placeholder);
  } else {
    mapaCanvas.appendChild(placeholder);
  }
  placeholder.textContent = texto;
  placeholder.style.display = 'flex';
}

function hideMapaPlaceholder() {
  if (!mapaCanvas) return;
  const placeholder = mapaCanvas.querySelector('.mapa-placeholder');
  if (placeholder) {
    placeholder.style.display = 'none';
  }
}

function quaternionToYaw(q) {
  const x = q?.x || 0;
  const y = q?.y || 0;
  const z = q?.z || 0;
  const w = q?.w ?? 1;
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  return Math.atan2(siny_cosp, cosy_cosp);
}

function updateMapMetadata(msg) {
  if (!slamMapMeta || !msg || !msg.info) return;
  const info = msg.info;
  const width = Number(info.width) || 0;
  const height = Number(info.height) || 0;
  const resolution = typeof info.resolution === 'number' ? info.resolution : 0;
  const sizeX = (width * resolution).toFixed(2);
  const sizeY = (height * resolution).toFixed(2);
  const origin = info.origin && info.origin.position ? info.origin.position : { x: 0, y: 0 };
  const originX = Number(origin.x || 0).toFixed(2);
  const originY = Number(origin.y || 0).toFixed(2);
  slamMapMeta.textContent = `Dimensiones: ${width}×${height} celdas (${sizeX}×${sizeY} m) · Resolución: ${resolution.toFixed(3)} m/celda · Origen: [${originX}, ${originY}]`;
}

function updatePosePanel(msg) {
  if (!slamPoseOutput) return;
  const poseStamped = msg?.pose?.pose ? msg.pose.pose : msg?.pose ? msg.pose : msg;
  if (!poseStamped || !poseStamped.position) {
    slamPoseOutput.textContent = 'Sin datos';
    clearXYRobotPose();
    return;
  }
  const pos = poseStamped.position;
  const orient = poseStamped.orientation || {};
  const yaw = quaternionToYaw(orient);
  const yawDeg = (yaw * 180 / Math.PI).toFixed(1);

  let covInfo = 'Covarianza: no disponible';
  const cov = msg?.pose?.covariance;
  if (Array.isArray(cov) && cov.length >= 36) {
    const sigmaX = Math.sqrt(Math.max(cov[0], 0)).toFixed(3);
    const sigmaY = Math.sqrt(Math.max(cov[7], 0)).toFixed(3);
    const sigmaTheta = Math.sqrt(Math.max(cov[35], 0)).toFixed(3);
    covInfo = `σx: ${sigmaX} m · σy: ${sigmaY} m · σθ: ${sigmaTheta} rad`;
  }

  slamPoseOutput.textContent =
    `x: ${(Number(pos.x) || 0).toFixed(3)} m\n` +
    `y: ${(Number(pos.y) || 0).toFixed(3)} m\n` +
    `z: ${(Number(pos.z) || 0).toFixed(3)} m\n` +
    `yaw: ${yawDeg}°\n` +
    covInfo;

  updateXYRobotPose(Number(pos.x) || 0, Number(pos.y) || 0, yaw);
}

function ensureRos2dViewer() {
  if (!mapaCanvas) return;

  if (!xyCanvas) {
    xyCanvas = document.createElement('canvas');
    xyCanvas.id = 'mapa-xy-canvas';
    xyCanvas.className = 'mapa-xy-canvas';
    xyCanvas.style.width = '100%';
    xyCanvas.style.height = '100%';
    mapaCanvas.insertBefore(xyCanvas, mapaCanvas.firstChild);
    xyCtx = xyCanvas.getContext('2d');
    resizeXYCanvas();

    if ('ResizeObserver' in window) {
      xyResizeObserver = new ResizeObserver(() => resizeXYCanvas());
      xyResizeObserver.observe(mapaCanvas);
    } else if (!xyWindowResizeHandlerRegistered) {
      window.addEventListener('resize', resizeXYCanvas);
      xyWindowResizeHandlerRegistered = true;
    }
  }

  const placeholder = mapaCanvas.querySelector('.mapa-placeholder');
  if (placeholder) {
    mapaCanvas.appendChild(placeholder);
  }

  hideMapaPlaceholder();
  drawXYScene();
}

function resizeXYCanvas() {
  if (!xyCanvas || !xyCtx || !mapaCanvas) return;
  const width = mapaCanvas.clientWidth || mapaCanvas.offsetWidth || 0;
  const height = mapaCanvas.clientHeight || mapaCanvas.offsetHeight || 0;
  if (!width || !height) return;

  const ratio = window.devicePixelRatio || 1;
  xyCanvasWidth = width;
  xyCanvasHeight = height;

  xyCanvas.width = Math.round(width * ratio);
  xyCanvas.height = Math.round(height * ratio);
  xyCanvas.style.width = `${width}px`;
  xyCanvas.style.height = `${height}px`;

  xyCtx.setTransform(ratio, 0, 0, ratio, 0, 0);
  xyCtx.imageSmoothingEnabled = true;

  xyOrigin = { x: width / 2, y: height / 2 };

  drawXYScene();
}

function drawXYScene() {
  if (!xyCtx || !xyCanvasWidth || !xyCanvasHeight) return;

  xyCtx.clearRect(0, 0, xyCanvasWidth, xyCanvasHeight);
  drawXYGrid();
  drawXYAxes();
  drawXYPois();
  drawXYRobotPose();
}

function drawXYGrid() {
  if (!xyCtx || XY_SCALE_PX_PER_M <= 0) return;
  const step = XY_GRID_STEP_METERS;
  if (step <= 0) return;

  const halfWidthM = xyCanvasWidth / (2 * XY_SCALE_PX_PER_M);
  const halfHeightM = xyCanvasHeight / (2 * XY_SCALE_PX_PER_M);
  const majorEvery = Math.max(1, XY_MAJOR_GRID_EVERY);
  const maxStepsX = Math.ceil(halfWidthM / step);
  const maxStepsY = Math.ceil(halfHeightM / step);

  xyCtx.save();
  xyCtx.lineWidth = 1;

  const drawVertical = (meters, isMajor) => {
    const { x } = worldToScreen(meters, 0);
    if (x < 0 || x > xyCanvasWidth) return;
    xyCtx.beginPath();
    xyCtx.moveTo(Math.round(x) + 0.5, 0);
    xyCtx.lineTo(Math.round(x) + 0.5, xyCanvasHeight);
    xyCtx.strokeStyle = isMajor ? 'rgba(59, 130, 246, 0.28)' : 'rgba(148, 163, 184, 0.14)';
    xyCtx.stroke();
  };

  const drawHorizontal = (meters, isMajor) => {
    const { y } = worldToScreen(0, meters);
    if (y < 0 || y > xyCanvasHeight) return;
    xyCtx.beginPath();
    xyCtx.moveTo(0, Math.round(y) + 0.5);
    xyCtx.lineTo(xyCanvasWidth, Math.round(y) + 0.5);
    xyCtx.strokeStyle = isMajor ? 'rgba(59, 130, 246, 0.28)' : 'rgba(148, 163, 184, 0.14)';
    xyCtx.stroke();
  };

  for (let i = 1; i <= maxStepsX; i += 1) {
    const meters = i * step;
    const isMajor = i % majorEvery === 0;
    drawVertical(meters, isMajor);
    drawVertical(-meters, isMajor);
  }

  for (let j = 1; j <= maxStepsY; j += 1) {
    const meters = j * step;
    const isMajor = j % majorEvery === 0;
    drawHorizontal(meters, isMajor);
    drawHorizontal(-meters, isMajor);
  }

  xyCtx.restore();
}

function drawXYAxes() {
  if (!xyCtx) return;
  xyCtx.save();
  xyCtx.lineWidth = 1.4;
  xyCtx.strokeStyle = 'rgba(148, 197, 255, 0.5)';

  xyCtx.beginPath();
  xyCtx.moveTo(Math.round(xyOrigin.x) + 0.5, 0);
  xyCtx.lineTo(Math.round(xyOrigin.x) + 0.5, xyCanvasHeight);
  xyCtx.moveTo(0, Math.round(xyOrigin.y) + 0.5);
  xyCtx.lineTo(xyCanvasWidth, Math.round(xyOrigin.y) + 0.5);
  xyCtx.stroke();

  xyCtx.fillStyle = 'rgba(191, 219, 254, 0.75)';
  xyCtx.font = '11px "Fira Code", monospace';
  xyCtx.fillText('+Y', xyOrigin.x + 6, 16);
  xyCtx.fillText('-Y', xyOrigin.x + 6, xyCanvasHeight - 8);
  xyCtx.fillText('+X', xyCanvasWidth - 30, xyOrigin.y - 6);
  xyCtx.fillText('-X', 8, xyOrigin.y - 6);

  xyCtx.restore();
}

function drawXYRobotPose() {
  if (!xyCtx || !xyPose.hasPose) return;
  const { x, y } = worldToScreen(xyPose.x, xyPose.y);
  if (x < -40 || x > xyCanvasWidth + 40 || y < -40 || y > xyCanvasHeight + 40) return;

  const arrowSize = Math.max(22, XY_POI_ARROW_SIZE_M * XY_SCALE_PX_PER_M * 1.4);

  xyCtx.save();
  xyCtx.translate(x, y);
  xyCtx.rotate(-xyPose.yaw);
  xyCtx.globalAlpha = 0.95;

  xyCtx.beginPath();
  xyCtx.moveTo(0, -arrowSize * 0.7);
  xyCtx.lineTo(arrowSize * 0.5, arrowSize * 0.6);
  xyCtx.lineTo(-arrowSize * 0.5, arrowSize * 0.6);
  xyCtx.closePath();
  xyCtx.fillStyle = 'rgba(96, 165, 250, 0.95)';
  xyCtx.fill();
  xyCtx.lineWidth = 2.4;
  xyCtx.strokeStyle = 'rgba(15, 23, 42, 0.85)';
  xyCtx.stroke();

  xyCtx.restore();

  xyCtx.save();
  xyCtx.globalAlpha = 0.9;
  xyCtx.lineWidth = 1.4;
  xyCtx.strokeStyle = 'rgba(147, 197, 253, 0.85)';
  xyCtx.beginPath();
  xyCtx.arc(x, y, Math.max(10, arrowSize * 0.35), 0, Math.PI * 2);
  xyCtx.stroke();
  xyCtx.restore();

  xyCtx.save();
  xyCtx.fillStyle = 'rgba(191, 219, 254, 0.85)';
  xyCtx.font = '12px "Fira Code", monospace';
  xyCtx.fillText(`(${xyPose.x.toFixed(2)}, ${xyPose.y.toFixed(2)}) m`, x + 12, y - 10);
  xyCtx.restore();
}

function drawXYPois() {
  if (!xyCtx || !xyPois?.length) return;

  xyPois.forEach(poi => {
    const { x, y } = worldToScreen(poi.x, poi.y);
    if (x < -40 || x > xyCanvasWidth + 40 || y < -40 || y > xyCanvasHeight + 40) return;

    xyCtx.save();
    xyCtx.globalAlpha = 0.9;
    xyCtx.fillStyle = poi.color;
    xyCtx.beginPath();
    xyCtx.arc(x, y, 6, 0, Math.PI * 2);
    xyCtx.fill();
    xyCtx.lineWidth = 1.2;
    xyCtx.strokeStyle = 'rgba(15, 23, 42, 0.8)';
    xyCtx.stroke();
    xyCtx.restore();

    if (poi.hasYaw) {
      drawOrientationArrow(poi.x, poi.y, poi.yaw, poi.color, XY_POI_ARROW_SIZE_M);
    }

    xyCtx.save();
    xyCtx.fillStyle = 'rgba(226, 232, 240, 0.9)';
    xyCtx.font = '11px "Fira Code", monospace';
    xyCtx.fillText(`${poi.name} (${poi.x.toFixed(2)}, ${poi.y.toFixed(2)})`, x + 10, y - 8);
    xyCtx.restore();
  });
}

function drawOrientationArrow(wx, wy, yaw, color, sizeMeters = XY_POI_ARROW_SIZE_M) {
  if (!xyCtx) return;
  const { x, y } = worldToScreen(wx, wy);
  const sizePx = Math.max(16, sizeMeters * XY_SCALE_PX_PER_M);

  xyCtx.save();
  xyCtx.translate(x, y);
  xyCtx.rotate(-yaw);
  xyCtx.globalAlpha = 0.9;
  xyCtx.beginPath();
  xyCtx.moveTo(0, -sizePx * 0.6);
  xyCtx.lineTo(sizePx * 0.4, sizePx * 0.5);
  xyCtx.lineTo(-sizePx * 0.4, sizePx * 0.5);
  xyCtx.closePath();
  xyCtx.fillStyle = color;
  xyCtx.fill();
  xyCtx.lineWidth = 1.6;
  xyCtx.strokeStyle = 'rgba(15, 23, 42, 0.75)';
  xyCtx.stroke();
  xyCtx.restore();
}

function worldToScreen(wx, wy) {
  return {
    x: xyOrigin.x + (wx * XY_SCALE_PX_PER_M),
    y: xyOrigin.y - (wy * XY_SCALE_PX_PER_M)
  };
}

function generatePoiColor(index) {
  const hue = (index * 53) % 360;
  return `hsl(${hue}, 82%, 64%)`;
}

function addPoi({ name, x, y, yaw }) {
  ensureRos2dViewer();
  const labelBase = typeof name === 'string' ? name.trim() : '';
  const label = labelBase || `Punto ${xyPois.length + 1}`;
  const valueX = Number.isFinite(x) ? x : 0;
  const valueY = Number.isFinite(y) ? y : 0;
  const hasYaw = Number.isFinite(yaw);
  const poi = {
    id: `poi-${Date.now()}-${xyPoiId++}`,
    name: label,
    x: valueX,
    y: valueY,
    yaw: hasYaw ? yaw : 0,
    hasYaw,
    color: generatePoiColor(xyPoiId)
  };
  xyPois = [...xyPois, poi];
  refreshPoiList();
  drawXYScene();
}

function removePoi(id) {
  xyPois = xyPois.filter(poi => poi.id !== id);
  refreshPoiList();
  drawXYScene();
}

function refreshPoiList() {
  if (!xyPoiList) return;
  xyPoiList.innerHTML = '';

  if (!xyPois.length) {
    const empty = document.createElement('div');
    empty.className = 'xy-poi-empty';
    empty.textContent = 'Agrega puntos para destacarlos en el plano.';
    xyPoiList.appendChild(empty);
    return;
  }

  xyPois.forEach(poi => {
    const item = document.createElement('div');
    item.className = 'xy-poi-item';

    const meta = document.createElement('div');
    meta.className = 'xy-poi-meta';

    const colorDot = document.createElement('span');
    colorDot.className = 'xy-poi-color';
    colorDot.style.background = poi.color;

    const nameEl = document.createElement('strong');
    nameEl.textContent = poi.name;

    const coords = document.createElement('span');
    coords.className = 'xy-poi-coords';
    const yawText = poi.hasYaw ? ` · θ:${radToDeg(poi.yaw).toFixed(1)}°` : '';
    coords.textContent = `x:${poi.x.toFixed(2)}m · y:${poi.y.toFixed(2)}m${yawText}`;

    meta.appendChild(colorDot);
    meta.appendChild(nameEl);
    meta.appendChild(coords);

    const removeBtn = document.createElement('button');
    removeBtn.type = 'button';
    removeBtn.className = 'xy-remove-btn';
    removeBtn.textContent = 'Quitar';
    removeBtn.addEventListener('click', () => removePoi(poi.id));

    item.appendChild(meta);
    item.appendChild(removeBtn);
    xyPoiList.appendChild(item);
  });
}

function updateXYRobotPose(x, y, yaw) {
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    clearXYRobotPose();
    return;
  }
  xyPose = {
    x,
    y,
    yaw: Number.isFinite(yaw) ? yaw : 0,
    hasPose: true
  };
  if (xyUseCurrentBtn) {
    xyUseCurrentBtn.disabled = false;
  }
  hideMapaPlaceholder();
  drawXYScene();
}

function clearXYRobotPose() {
  xyPose = { x: 0, y: 0, yaw: 0, hasPose: false };
  if (xyUseCurrentBtn) {
    xyUseCurrentBtn.disabled = true;
  }
  drawXYScene();
}

function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

function radToDeg(rad) {
  return (rad * 180) / Math.PI;
}

function destroyRos2dViewer() {
  if (xyResizeObserver) {
    xyResizeObserver.disconnect();
    xyResizeObserver = null;
  }
  if (xyWindowResizeHandlerRegistered) {
    window.removeEventListener('resize', resizeXYCanvas);
    xyWindowResizeHandlerRegistered = false;
  }
  clearXYRobotPose();
}

// --- LiDAR monitoring & motor control ---
function refreshLidarControls() {
  const rosConnected = !!(window.ros && window.ros.isConnected);
  const disableAll = !rosConnected || lidarActionInFlight;
  if (lidarStartBtn) {
    lidarStartBtn.disabled = disableAll || lidarState === 'running';
  }
  if (lidarStopBtn) {
    lidarStopBtn.disabled = disableAll || lidarState !== 'running';
  }
}

function setLidarState(state, message) {
  lidarState = state;
  if (lidarStatusChip) {
    lidarStatusChip.classList.remove('lidar-status-off', 'lidar-status-stopped', 'lidar-status-running');
    let chipClass = 'lidar-status-off';
    let label = 'Apagado';
    if (state === 'running') {
      chipClass = 'lidar-status-running';
      label = 'Funcionando';
    } else if (state === 'stopped') {
      chipClass = 'lidar-status-stopped';
      label = 'Detenido';
    }
    lidarStatusChip.classList.add(chipClass);
    lidarStatusChip.textContent = label;
  }
  if (lidarStatusText) {
    if (typeof message === 'string' && message.length) {
      lidarStatusText.textContent = message;
    } else {
      let defaultMsg = 'Sin conexión con ROS.';
      if (state === 'running') {
        defaultMsg = 'Recibiendo datos del LiDAR.';
      } else if (state === 'stopped') {
        defaultMsg = 'LiDAR disponible, sin datos recientes.';
      }
      lidarStatusText.textContent = defaultMsg;
    }
  }
  refreshLidarControls();
}

function rosTimeToDate(stamp) {
  if (!stamp) return null;
  const sec = Number(stamp.sec ?? stamp.secs ?? 0);
  const nanosec = Number(stamp.nanosec ?? stamp.nsecs ?? 0);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) return null;
  return new Date(sec * 1000 + Math.floor(nanosec / 1e6));
}

function markLidarHeartbeat(msg) {
  const timestamp = rosTimeToDate(msg?.header?.stamp) ?? new Date();
  const hora = timestamp.toLocaleTimeString('es-ES', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
  });
  setLidarState('running', `Última lectura /scan: ${hora}`);
  if (lidarHeartbeatTimer) clearTimeout(lidarHeartbeatTimer);
  lidarHeartbeatTimer = setTimeout(() => {
    setLidarState('stopped', `Sin datos recientes de /scan (> ${Math.round(LIDAR_HEARTBEAT_TIMEOUT_MS / 1000)} s).`);
  }, LIDAR_HEARTBEAT_TIMEOUT_MS);
}

function startLidarMonitoring() {
  if (!window.ros || !window.ros.isConnected) return;
  if (!lidarScanMonitor) {
    lidarScanMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: LIDAR_TOPIC_NAME,
      messageType: 'sensor_msgs/LaserScan',
      queue_length: 1,
      throttle_rate: 0
    });
    lidarScanMonitor.subscribe(msg => {
      markLidarHeartbeat(msg);
    });
  }
  setLidarState('stopped', 'Sin lecturas recientes de /scan.');
}

function stopLidarMonitoring() {
  if (lidarHeartbeatTimer) {
    clearTimeout(lidarHeartbeatTimer);
    lidarHeartbeatTimer = null;
  }
  if (lidarScanMonitor) {
    lidarScanMonitor.unsubscribe();
    lidarScanMonitor = null;
  }
  lidarStartService = null;
  lidarStopService = null;
  lidarActionInFlight = false;
  setLidarState('off', 'Sin conexión con ROS.');
}

function ensureLidarServices() {
  if (!window.ros || !window.ros.isConnected) return false;
  if (!lidarStartService) {
    lidarStartService = new ROSLIB.Service({
      ros: window.ros,
      name: '/start_motor',
      serviceType: 'std_srvs/srv/Empty'
    });
  }
  if (!lidarStopService) {
    lidarStopService = new ROSLIB.Service({
      ros: window.ros,
      name: '/stop_motor',
      serviceType: 'std_srvs/srv/Empty'
    });
  }
  return true;
}

function handleLidarStartClick() {
  if (!window.ros || !window.ros.isConnected) {
    setLidarState('off', 'Sin conexión con ROS. Conéctate antes de encender el LiDAR.');
    return;
  }
  const previousState = lidarState;
  if (!ensureLidarServices()) {
    setLidarState('off', 'No se pudo inicializar el cliente ROS. Verifica la conexión.');
    return;
  }
  lidarActionInFlight = true;
  refreshLidarControls();
  setLidarState('stopped', 'Solicitando arranque del LiDAR...');
  lidarStartService.callService(new ROSLIB.ServiceRequest({}), () => {
    lidarActionInFlight = false;
    setLidarState('stopped', 'Motor arrancado. Esperando lecturas de /scan...');
  }, error => {
    lidarActionInFlight = false;
    setLidarState(previousState, `Error al encender el LiDAR: ${error}`);
  });
}

function handleLidarStopClick() {
  if (!window.ros || !window.ros.isConnected) {
    setLidarState('off', 'Sin conexión con ROS. Conéctate antes de detener el LiDAR.');
    return;
  }
  const previousState = lidarState;
  if (!ensureLidarServices()) {
    setLidarState('off', 'No se pudo inicializar el cliente ROS. Verifica la conexión.');
    return;
  }
  lidarActionInFlight = true;
  refreshLidarControls();
  setLidarState('running', 'Solicitando parada del LiDAR...');
  lidarStopService.callService(new ROSLIB.ServiceRequest({}), () => {
    lidarActionInFlight = false;
    setLidarState('stopped', 'Motor detenido. El LiDAR no está emitiendo escaneos.');
  }, error => {
    lidarActionInFlight = false;
    setLidarState(previousState, `Error al detener el LiDAR: ${error}`);
  });
}

if (lidarStartBtn) {
  lidarStartBtn.addEventListener('click', handleLidarStartClick);
}
if (lidarStopBtn) {
  lidarStopBtn.addEventListener('click', handleLidarStopClick);
}

refreshLidarControls();

// --- Temperatura IMU en navegación ---
function setNavImuTempDisplay(state, tempValue = navImuLastTemp) {
  if (!navImuTemp) return;
  navImuTemp.classList.remove(
    'nav-imu-temp-off',
    'nav-imu-temp-waiting',
    'nav-imu-temp-active',
    'nav-imu-temp-stale'
  );

  let label = 'Temp: — °C';

  switch (state) {
    case 'waiting':
      navImuTemp.classList.add('nav-imu-temp-waiting');
      label = 'Temp: esperando…';
      break;
    case 'active':
      navImuTemp.classList.add('nav-imu-temp-active');
      if (Number.isFinite(tempValue)) {
        label = `Temp: ${tempValue.toFixed(1)} °C`;
      }
      break;
    case 'stale':
      navImuTemp.classList.add('nav-imu-temp-stale');
      if (Number.isFinite(tempValue)) {
        label = `Temp: ${tempValue.toFixed(1)} °C · sin datos recientes`;
      } else {
        label = 'Temp: sin datos recientes';
      }
      break;
    case 'off':
    default:
      navImuTemp.classList.add('nav-imu-temp-off');
      label = 'Temp: — °C';
      break;
  }

  navImuTemp.textContent = label;
}

function clearImuTempTimer() {
  if (imuTempTimer) {
    clearTimeout(imuTempTimer);
    imuTempTimer = null;
  }
}

function scheduleImuTempStale() {
  clearImuTempTimer();
  imuTempTimer = setTimeout(() => {
    setNavImuTempDisplay('stale');
  }, IMU_TEMP_TIMEOUT_MS);
}

function handleImuTempMessage(msg) {
  if (!navImuTemp) return;
  let temp = null;

  if (typeof msg === 'number') {
    temp = msg;
  } else if (msg) {
    if (typeof msg.data === 'number') {
      temp = msg.data;
    } else if (typeof msg.temperature === 'number') {
      temp = msg.temperature;
    } else if (typeof msg.temp === 'number') {
      temp = msg.temp;
    }
  }

  if (!Number.isFinite(temp)) return;

  navImuLastTemp = temp;
  setNavImuTempDisplay('active', temp);
  scheduleImuTempStale();
}

function startImuTempMonitoring() {
  if (!navImuTemp) return;
  if (!window.ros || !window.ros.isConnected) return;
  if (imuTempMonitor) return;

  setNavImuTempDisplay('waiting');

  imuTempMonitor = new ROSLIB.Topic({
    ros: window.ros,
    name: IMU_TEMP_TOPIC_NAME,
    messageType: 'sensor_msgs/Temperature',
    queue_length: 1,
    throttle_rate: 0
  });

  imuTempMonitor.subscribe(handleImuTempMessage);
}

function stopImuTempMonitoring(clearDisplay = true) {
  if (imuTempMonitor) {
    imuTempMonitor.unsubscribe();
    imuTempMonitor = null;
  }
  clearImuTempTimer();
  if (clearDisplay) {
    navImuLastTemp = null;
    setNavImuTempDisplay('off');
  } else if (navImuTemp) {
    setNavImuTempDisplay('waiting');
  }
}

setNavImuTempDisplay('off');

function xyvizInit() {
  if (xyvizInitialized) return;
  if (!xyvizState.canvas || !xyvizState.ctx) return;
  xyvizInitialized = true;
  if (typeof ResizeObserver !== 'undefined' && xyvizState.wrap) {
    xyvizState.resizeObserver = new ResizeObserver(() => xyvizResizeCanvas());
    xyvizState.resizeObserver.observe(xyvizState.wrap);
  } else if (!xyvizState.windowResizeHandlerRegistered) {
    window.addEventListener('resize', xyvizResizeCanvas);
    xyvizState.windowResizeHandlerRegistered = true;
  }
  xyvizResizeCanvas();
  xyvizUpdateAxisButton();
  xyvizUpdateLandmarkList();
  xyvizUpdateSelectedMetrics();
  xyvizUpdateUseCurrentAvailability();
  xyvizRegisterCanvasEvents();
}

function xyvizResizeCanvas() {
  if (!xyvizState.canvas || !xyvizState.ctx) return;
  const container = xyvizState.wrap || xyvizState.canvas.parentElement;
  if (!container) return;
  const rect = container.getBoundingClientRect();
  const width = Math.max(rect.width, 240);
  const height = Math.max(rect.height, 240);
  const ratio = window.devicePixelRatio || 1;
  xyvizState.width = width;
  xyvizState.height = height;
  xyvizState.canvas.width = Math.round(width * ratio);
  xyvizState.canvas.height = Math.round(height * ratio);
  xyvizState.canvas.style.width = `${width}px`;
  xyvizState.canvas.style.height = `${height}px`;
  xyvizState.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  xyvizResetOrigin();
  xyvizState.panOffset = { x: 0, y: 0 };
  xyvizDrawScene();
}

function xyvizResetOrigin() {
  xyvizState.origin = {
    x: xyvizState.width / 2 + xyvizState.panOffset.x,
    y: xyvizState.height / 2 + xyvizState.panOffset.y
  };
}

function xyvizSetPlaceholder(message, visible = true) {
  if (!xyvizPlaceholder) return;
  if (typeof message === 'string') {
    xyvizPlaceholder.textContent = message;
  }
  xyvizPlaceholder.style.display = visible ? 'flex' : 'none';
}

function xyvizUpdateAxisButton() {
  if (!xyvizAxisToggle) return;
  if (xyvizState.axisMode === 'standard') {
    xyvizAxisToggle.textContent = 'Ejes: Horizontal = X · Vertical = Y';
  } else {
    xyvizAxisToggle.textContent = 'Ejes: Horizontal = Y · Vertical = X';
  }
}

function xyvizToggleAxisMode() {
  xyvizState.axisMode = xyvizState.axisMode === 'standard' ? 'swapped' : 'standard';
  xyvizUpdateAxisButton();
  xyvizDrawScene();
  xyvizUpdateSelectedMetrics();
}

function xyvizResetView() {
  xyvizState.scale = XYVIZ_SCALE_PX_PER_M;
  xyvizState.panOffset = { x: 0, y: 0 };
  xyvizResetOrigin();
  xyvizDrawScene();
}

function xyvizUpdateUseCurrentAvailability() {
  if (!xyvizUseCurrentBtn) return;
  xyvizUseCurrentBtn.disabled = !xyvizState.robotPose.hasPose;
}

function xyvizWorldToScreen(wx, wy) {
  const scale = xyvizState.scale;
  if (xyvizState.axisMode === 'standard') {
    return {
      x: xyvizState.origin.x + wx * scale,
      y: xyvizState.origin.y - wy * scale
    };
  }
  return {
    x: xyvizState.origin.x + wy * scale,
    y: xyvizState.origin.y - wx * scale
  };
}

function xyvizDrawScene() {
  if (!xyvizState.ctx) return;
  xyvizState.ctx.clearRect(0, 0, xyvizState.width, xyvizState.height);
  xyvizDrawGrid();
  xyvizDrawAxes();
  xyvizDrawTrail();
  xyvizDrawLandmarks();
  xyvizDrawRobot();
}

function xyvizDrawGrid() {
  if (!xyvizState.ctx) return;
  const step = XYVIZ_GRID_STEP_METERS;
  const scale = xyvizState.scale;
  if (scale <= 0 || step <= 0) return;

  const halfWidthM = xyvizState.width / (2 * scale);
  const halfHeightM = xyvizState.height / (2 * scale);
  const extra = step;
  const maxStepsX = Math.ceil(halfWidthM / step);
  const maxStepsY = Math.ceil(halfHeightM / step);

  xyvizState.ctx.save();
  xyvizState.ctx.lineWidth = 1;

  for (let i = 1; i <= maxStepsX; i += 1) {
    const meters = i * step;
    const isMajor = i % XYVIZ_MAJOR_GRID_EVERY === 0;
    const color = isMajor ? 'rgba(59, 130, 246, 0.28)' : 'rgba(148, 163, 184, 0.14)';
    xyvizStrokeWorldLine(meters, -(halfHeightM + extra), meters, halfHeightM + extra, color);
    xyvizStrokeWorldLine(-meters, -(halfHeightM + extra), -meters, halfHeightM + extra, color);
  }

  for (let j = 1; j <= maxStepsY; j += 1) {
    const meters = j * step;
    const isMajor = j % XYVIZ_MAJOR_GRID_EVERY === 0;
    const color = isMajor ? 'rgba(59, 130, 246, 0.28)' : 'rgba(148, 163, 184, 0.14)';
    xyvizStrokeWorldLine(-(halfWidthM + extra), meters, halfWidthM + extra, meters, color);
    xyvizStrokeWorldLine(-(halfWidthM + extra), -meters, halfWidthM + extra, -meters, color);
  }

  xyvizState.ctx.restore();
}

function xyvizDrawAxes() {
  if (!xyvizState.ctx) return;
  const scale = xyvizState.scale;
  const halfWidthM = xyvizState.width / (2 * scale);
  const halfHeightM = xyvizState.height / (2 * scale);
  const extendX = halfWidthM + XYVIZ_GRID_STEP_METERS;
  const extendY = halfHeightM + XYVIZ_GRID_STEP_METERS;

  xyvizState.ctx.save();
  xyvizState.ctx.lineWidth = 1.4;
  xyvizState.ctx.strokeStyle = 'rgba(148, 197, 255, 0.5)';

  if (xyvizState.axisMode === 'standard') {
    xyvizStrokeWorldLine(0, -extendY, 0, extendY, 'rgba(148, 197, 255, 0.55)', 1.4);
    xyvizStrokeWorldLine(-extendX, 0, extendX, 0, 'rgba(148, 197, 255, 0.55)', 1.4);
  } else {
    xyvizStrokeWorldLine(-extendY, 0, extendY, 0, 'rgba(148, 197, 255, 0.55)', 1.4);
    xyvizStrokeWorldLine(0, -extendX, 0, extendX, 'rgba(148, 197, 255, 0.55)', 1.4);
  }

  xyvizState.ctx.restore();

  xyvizState.ctx.save();
  xyvizState.ctx.fillStyle = 'rgba(191, 219, 254, 0.85)';
  xyvizState.ctx.font = '11px "Fira Code", monospace';

  const verticalRange = halfHeightM * 0.9;
  const horizontalRange = halfWidthM * 0.9;

  const verticalPositive = xyvizState.axisMode === 'standard'
    ? xyvizWorldToScreen(0, verticalRange)
    : xyvizWorldToScreen(verticalRange, 0);
  const verticalNegative = xyvizState.axisMode === 'standard'
    ? xyvizWorldToScreen(0, -verticalRange)
    : xyvizWorldToScreen(-verticalRange, 0);
  const horizontalPositive = xyvizState.axisMode === 'standard'
    ? xyvizWorldToScreen(horizontalRange, 0)
    : xyvizWorldToScreen(0, horizontalRange);
  const horizontalNegative = xyvizState.axisMode === 'standard'
    ? xyvizWorldToScreen(-horizontalRange, 0)
    : xyvizWorldToScreen(0, -horizontalRange);

  const verticalLabel = xyvizState.axisMode === 'standard' ? 'Y' : 'X';
  const horizontalLabel = xyvizState.axisMode === 'standard' ? 'X' : 'Y';

  xyvizState.ctx.fillText(`+${verticalLabel}`, verticalPositive.x + 6, verticalPositive.y - 6);
  xyvizState.ctx.fillText(`-${verticalLabel}`, verticalNegative.x + 6, verticalNegative.y + 12);
  xyvizState.ctx.fillText(`+${horizontalLabel}`, horizontalPositive.x - 24, horizontalPositive.y - 6);
  xyvizState.ctx.fillText(`-${horizontalLabel}`, horizontalNegative.x + 6, horizontalNegative.y - 6);

  xyvizState.ctx.restore();
}

function xyvizStrokeWorldLine(x1, y1, x2, y2, strokeStyle, lineWidth = 1) {
  if (!xyvizState.ctx) return;
  const p1 = xyvizWorldToScreen(x1, y1);
  const p2 = xyvizWorldToScreen(x2, y2);
  xyvizState.ctx.beginPath();
  xyvizState.ctx.moveTo(p1.x, p1.y);
  xyvizState.ctx.lineTo(p2.x, p2.y);
  xyvizState.ctx.strokeStyle = strokeStyle;
  xyvizState.ctx.lineWidth = lineWidth;
  xyvizState.ctx.stroke();
}

function xyvizDrawTrail() {
  if (!xyvizState.ctx || !xyvizState.trail.length) return;
  xyvizState.ctx.save();
  xyvizState.ctx.lineWidth = 1.4;
  xyvizState.ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
  xyvizState.ctx.beginPath();
  xyvizState.trail.forEach((point, index) => {
    const { x, y } = xyvizWorldToScreen(point.x, point.y);
    if (index === 0) {
      xyvizState.ctx.moveTo(x, y);
    } else {
      xyvizState.ctx.lineTo(x, y);
    }
  });
  xyvizState.ctx.stroke();
  xyvizState.ctx.restore();
}

function xyvizDrawLandmarks() {
  if (!xyvizState.ctx) return;
  if (!xyvizState.landmarks.length) return;

  xyvizState.landmarks.forEach(landmark => {
    const screen = xyvizWorldToScreen(landmark.x, landmark.y);
    if (screen.x < -60 || screen.x > xyvizState.width + 60 || screen.y < -60 || screen.y > xyvizState.height + 60) {
      return;
    }

    xyvizState.ctx.save();
    xyvizState.ctx.globalAlpha = xyvizState.selectedId === landmark.id ? 0.95 : 0.75;
    xyvizState.ctx.fillStyle = landmark.color;
    xyvizState.ctx.beginPath();
    xyvizState.ctx.arc(screen.x, screen.y, 7, 0, Math.PI * 2);
    xyvizState.ctx.fill();
    xyvizState.ctx.lineWidth = xyvizState.selectedId === landmark.id ? 2 : 1.2;
    xyvizState.ctx.strokeStyle = 'rgba(15, 23, 42, 0.85)';
    xyvizState.ctx.stroke();
    xyvizState.ctx.restore();

    if (landmark.hasYaw) {
      xyvizDrawOrientationArrow(landmark.x, landmark.y, landmark.yaw, landmark.color, 0.35);
    }

    xyvizState.ctx.save();
    xyvizState.ctx.fillStyle = 'rgba(226, 232, 240, 0.88)';
    xyvizState.ctx.font = '11px "Fira Code", monospace';
    const yawText = landmark.hasYaw ? ` · yaw:${radToDeg(landmark.yaw).toFixed(1)}°` : '';
    xyvizState.ctx.fillText(`${landmark.name} (${landmark.x.toFixed(2)}, ${landmark.y.toFixed(2)})${yawText}`, screen.x + 10, screen.y - 8);
    xyvizState.ctx.restore();
  });
}

function xyvizDrawRobot() {
  if (!xyvizState.ctx) return;
  if (!xyvizState.robotPose.hasPose) return;
  const pose = xyvizState.robotPose;
  const screen = xyvizWorldToScreen(pose.x, pose.y);
  if (screen.x < -80 || screen.x > xyvizState.width + 80 || screen.y < -80 || screen.y > xyvizState.height + 80) {
    return;
  }

  const arrowSize = Math.max(22, 0.6 * xyvizState.scale);
  const angle = xyvizComputeScreenAngle(pose.x, pose.y, pose.yaw);

  xyvizState.ctx.save();
  xyvizState.ctx.translate(screen.x, screen.y);
  xyvizState.ctx.rotate(angle);
  xyvizState.ctx.globalAlpha = 0.96;
  xyvizState.ctx.beginPath();
  xyvizState.ctx.moveTo(0, -arrowSize * 0.7);
  xyvizState.ctx.lineTo(arrowSize * 0.5, arrowSize * 0.6);
  xyvizState.ctx.lineTo(-arrowSize * 0.5, arrowSize * 0.6);
  xyvizState.ctx.closePath();
  xyvizState.ctx.fillStyle = 'rgba(96, 165, 250, 0.95)';
  xyvizState.ctx.fill();
  xyvizState.ctx.lineWidth = 2.4;
  xyvizState.ctx.strokeStyle = 'rgba(15, 23, 42, 0.85)';
  xyvizState.ctx.stroke();
  xyvizState.ctx.restore();

  xyvizState.ctx.save();
  xyvizState.ctx.globalAlpha = 0.88;
  xyvizState.ctx.lineWidth = 1.6;
  xyvizState.ctx.strokeStyle = 'rgba(147, 197, 253, 0.85)';
  xyvizState.ctx.beginPath();
  xyvizState.ctx.arc(screen.x, screen.y, Math.max(10, arrowSize * 0.35), 0, Math.PI * 2);
  xyvizState.ctx.stroke();
  xyvizState.ctx.restore();
}

function xyvizComputeScreenAngle(wx, wy, yaw) {
  const base = xyvizWorldToScreen(wx, wy);
  const dir = xyvizWorldToScreen(wx + Math.cos(yaw), wy + Math.sin(yaw));
  return Math.atan2(dir.y - base.y, dir.x - base.x);
}

function xyvizDrawOrientationArrow(wx, wy, yaw, color, sizeMeters = 0.45) {
  if (!xyvizState.ctx) return;
  if (!Number.isFinite(yaw)) return;
  const base = xyvizWorldToScreen(wx, wy);
  const angle = xyvizComputeScreenAngle(wx, wy, yaw);
  const sizePx = Math.max(16, sizeMeters * xyvizState.scale);

  xyvizState.ctx.save();
  xyvizState.ctx.translate(base.x, base.y);
  xyvizState.ctx.rotate(angle);
  xyvizState.ctx.globalAlpha = 0.85;
  xyvizState.ctx.beginPath();
  xyvizState.ctx.moveTo(0, -sizePx * 0.6);
  xyvizState.ctx.lineTo(sizePx * 0.45, sizePx * 0.5);
  xyvizState.ctx.lineTo(-sizePx * 0.45, sizePx * 0.5);
  xyvizState.ctx.closePath();
  xyvizState.ctx.fillStyle = color;
  xyvizState.ctx.fill();
  xyvizState.ctx.restore();
}

function xyvizAddTrailPoint(x, y) {
  xyvizState.trail.push({ x, y });
  if (xyvizState.trail.length > XYVIZ_TRAIL_LIMIT) {
    xyvizState.trail.splice(0, xyvizState.trail.length - XYVIZ_TRAIL_LIMIT);
  }
}

function xyvizScreenToWorld(sx, sy) {
  const scale = xyvizState.scale;
  if (xyvizState.axisMode === 'standard') {
    return {
      x: (sx - xyvizState.origin.x) / scale,
      y: (xyvizState.origin.y - sy) / scale
    };
  }
  return {
    x: (xyvizState.origin.y - sy) / scale,
    y: (sx - xyvizState.origin.x) / scale
  };
}

function xyvizRegisterCanvasEvents() {
  if (!xyvizState.canvas) return;
  xyvizState.canvas.addEventListener('wheel', xyvizHandleWheel, { passive: false });
  xyvizState.canvas.addEventListener('mousedown', xyvizHandlePointerDown);
  xyvizState.canvas.addEventListener('mousemove', xyvizHandlePointerMove);
  xyvizState.canvas.addEventListener('mouseup', xyvizHandlePointerUp);
  xyvizState.canvas.addEventListener('mouseleave', xyvizHandlePointerUp);
  xyvizState.canvas.addEventListener('touchstart', xyvizHandleTouchStart, { passive: false });
  xyvizState.canvas.addEventListener('touchmove', xyvizHandleTouchMove, { passive: false });
  xyvizState.canvas.addEventListener('touchend', xyvizHandlePointerUp, { passive: false });
  window.addEventListener('keydown', xyvizHandleKeyPan);
}

function xyvizHandleWheel(event) {
  event.preventDefault();
  const delta = Math.sign(event.deltaY);
  const factor = delta > 0 ? 0.9 : 1.1;
  xyvizApplyZoom(factor, event.offsetX, event.offsetY);
}

function xyvizApplyZoom(factor, centerX, centerY) {
  const previousScale = xyvizState.scale;
  const newScale = Math.min(Math.max(previousScale * factor, 10), 400);
  if (newScale === previousScale) return;
  const worldBefore = xyvizScreenToWorld(centerX, centerY);
  xyvizState.scale = newScale;
  const newScreen = xyvizWorldToScreen(worldBefore.x, worldBefore.y);
  xyvizState.panOffset.x += centerX - newScreen.x;
  xyvizState.panOffset.y += centerY - newScreen.y;
  xyvizResetOrigin();
  xyvizDrawScene();
}

function xyvizHandlePointerDown(event) {
  event.preventDefault();
  xyvizState.isPanning = true;
  xyvizState.panStart = { x: event.clientX, y: event.clientY };
  xyvizState.panOriginSnapshot = { ...xyvizState.panOffset };
}

function xyvizHandlePointerMove(event) {
  if (!xyvizState.isPanning) return;
  event.preventDefault();
  const dx = event.clientX - xyvizState.panStart.x;
  const dy = event.clientY - xyvizState.panStart.y;
  xyvizState.panOffset = {
    x: xyvizState.panOriginSnapshot.x + dx,
    y: xyvizState.panOriginSnapshot.y + dy
  };
  xyvizResetOrigin();
  xyvizDrawScene();
}

function xyvizHandlePointerUp() {
  xyvizState.isPanning = false;
}

function xyvizHandleTouchStart(event) {
  if (event.touches.length !== 1) return;
  const touch = event.touches[0];
  xyvizState.isPanning = true;
  xyvizState.panStart = { x: touch.clientX, y: touch.clientY };
  xyvizState.panOriginSnapshot = { ...xyvizState.panOffset };
}

function xyvizHandleTouchMove(event) {
  if (!xyvizState.isPanning || event.touches.length !== 1) return;
  event.preventDefault();
  const touch = event.touches[0];
  const dx = touch.clientX - xyvizState.panStart.x;
  const dy = touch.clientY - xyvizState.panStart.y;
  xyvizState.panOffset = {
    x: xyvizState.panOriginSnapshot.x + dx,
    y: xyvizState.panOriginSnapshot.y + dy
  };
  xyvizResetOrigin();
  xyvizDrawScene();
}

function xyvizHandleKeyPan(event) {
  if (activeTab !== 'plano_xy') return;
  const stepPixels = 40;
  let moved = false;
  switch (event.key.toLowerCase()) {
    case 'w':
      xyvizState.panOffset.y += stepPixels;
      moved = true;
      break;
    case 's':
      xyvizState.panOffset.y -= stepPixels;
      moved = true;
      break;
    case 'a':
      xyvizState.panOffset.x += stepPixels;
      moved = true;
      break;
    case 'd':
      xyvizState.panOffset.x -= stepPixels;
      moved = true;
      break;
  }
  if (moved) {
    xyvizResetOrigin();
    xyvizDrawScene();
  }
}

function xyvizAddLandmark({ name, x, y, yaw }) {
  const trimmed = (name || '').trim();
  const id = xyvizState.nextLandmarkId++;
  const yawRad = Number.isFinite(yaw) ? yaw : null;
  const color = XYVIZ_COLORS[(id - 1) % XYVIZ_COLORS.length];
  xyvizState.landmarks.push({
    id,
    name: trimmed || `Landmark ${id}`,
    x,
    y,
    yaw: yawRad,
    hasYaw: Number.isFinite(yawRad),
    color
  });
  xyvizState.selectedId = id;
  xyvizUpdateLandmarkList();
  xyvizUpdateSelectedMetrics();
  xyvizDrawScene();
}

function xyvizRemoveLandmark(id) {
  xyvizState.landmarks = xyvizState.landmarks.filter(lm => lm.id !== id);
  if (xyvizState.selectedId === id) {
    xyvizState.selectedId = xyvizState.landmarks.length ? xyvizState.landmarks[0].id : null;
  }
  xyvizUpdateLandmarkList();
  xyvizUpdateSelectedMetrics();
  xyvizDrawScene();
}

function xyvizSelectLandmark(id) {
  xyvizState.selectedId = id;
  xyvizUpdateLandmarkList();
  xyvizUpdateSelectedMetrics();
}

function xyvizUpdateLandmarkList() {
  if (!xyvizLandmarkList) return;
  xyvizLandmarkList.innerHTML = '';
  if (!xyvizState.landmarks.length) {
    const empty = document.createElement('div');
    empty.className = 'xyviz-landmark-empty';
    empty.textContent = 'Agrega landmarks para monitorear posiciones de referencia.';
    xyvizLandmarkList.appendChild(empty);
    return;
  }

  xyvizState.landmarks.forEach(landmark => {
    const item = document.createElement('div');
    item.className = 'xyviz-landmark-item';
    if (xyvizState.selectedId === landmark.id) {
      item.classList.add('selected');
    }

    const meta = document.createElement('span');
    const yawText = landmark.hasYaw ? ` · yaw:${radToDeg(landmark.yaw).toFixed(1)}°` : '';
    meta.textContent = `${landmark.name} (${landmark.x.toFixed(2)}, ${landmark.y.toFixed(2)})${yawText}`;

    const actions = document.createElement('div');
    actions.className = 'xyviz-landmark-actions';

    const selectBtn = document.createElement('button');
    selectBtn.type = 'button';
    selectBtn.className = 'xyviz-landmark-select';
    selectBtn.textContent = 'Seleccionar';
    selectBtn.addEventListener('click', event => {
      event.preventDefault();
      event.stopPropagation();
      xyvizSelectLandmark(landmark.id);
    });

    const removeBtn = document.createElement('button');
    removeBtn.type = 'button';
    removeBtn.className = 'xyviz-landmark-remove';
    removeBtn.textContent = 'Quitar';
    removeBtn.addEventListener('click', event => {
      event.preventDefault();
      event.stopPropagation();
      xyvizRemoveLandmark(landmark.id);
    });

    actions.appendChild(selectBtn);
    actions.appendChild(removeBtn);
    item.appendChild(meta);
    item.appendChild(actions);
    item.addEventListener('click', () => xyvizSelectLandmark(landmark.id));
    xyvizLandmarkList.appendChild(item);
  });
}

function xyvizUpdateSelectedMetrics() {
  if (!xyvizSelectedName || !xyvizState.landmarks.length) {
    return;
  }
  const landmark = xyvizState.landmarks.find(lm => lm.id === xyvizState.selectedId);
  if (!landmark || !xyvizState.robotPose.hasPose) {
    xyvizSelectedName.textContent = landmark ? landmark.name : '—';
    xyvizSelectedDistance.textContent = '—';
    xyvizSelectedDx.textContent = '—';
    xyvizSelectedDy.textContent = '—';
    xyvizSelectedDyaw.textContent = landmark && landmark.hasYaw ? '—' : 'N/A';
    xyvizSelectedDxError.textContent = '—';
    xyvizSelectedDyError.textContent = '—';
    xyvizSelectedDyawError.textContent = landmark && landmark.hasYaw ? '—' : 'N/A';
    return;
  }

  const pose = xyvizState.robotPose;
  const dx = pose.x - landmark.x;
  const dy = pose.y - landmark.y;
  const distance = Math.hypot(dx, dy);
  const yawDesired = landmark.hasYaw ? landmark.yaw : null;
  const yawDiff = landmark.hasYaw ? normalizeAngle(pose.yaw - yawDesired) : null;

  const errX = computePercentError(landmark.x, pose.x);
  const errY = computePercentError(landmark.y, pose.y);
  const errYaw = Number.isFinite(yawDesired) ? computePercentError(yawDesired, pose.yaw) : null;

  xyvizSelectedName.textContent = landmark.name;
  xyvizSelectedDistance.textContent = Number.isFinite(distance) ? formatLinear(distance) : '—';
  xyvizSelectedDx.textContent = formatLinearDiff(dx);
  xyvizSelectedDy.textContent = formatLinearDiff(dy);
  xyvizSelectedDyaw.textContent = landmark.hasYaw ? formatAngleDiff(yawDiff) : 'N/A';
  xyvizSelectedDxError.textContent = Number.isFinite(errX) ? formatPercent(errX) : '—';
  xyvizSelectedDyError.textContent = Number.isFinite(errY) ? formatPercent(errY) : '—';
  xyvizSelectedDyawError.textContent = landmark.hasYaw && Number.isFinite(errYaw) ? formatPercent(errYaw) : (landmark.hasYaw ? '—' : 'N/A');
}

function xyvizHandleLandmarkSubmit(event) {
  event.preventDefault();
  if (!xyvizLandmarkNameInput || !xyvizLandmarkXInput || !xyvizLandmarkYInput) return;
  const rawName = xyvizLandmarkNameInput.value;
  const rawX = Number.parseFloat(xyvizLandmarkXInput.value);
  const rawY = Number.parseFloat(xyvizLandmarkYInput.value);
  const rawYawDeg = Number.parseFloat(xyvizLandmarkYawInput?.value ?? '');
  const x = Number.isFinite(rawX) ? rawX : 0;
  const y = Number.isFinite(rawY) ? rawY : 0;
  const yaw = Number.isFinite(rawYawDeg) ? degToRad(rawYawDeg) : null;
  xyvizAddLandmark({ name: rawName, x, y, yaw });
  if (xyvizLandmarkNameInput) xyvizLandmarkNameInput.value = '';
  if (xyvizLandmarkXInput) xyvizLandmarkXInput.value = '0';
  if (xyvizLandmarkYInput) xyvizLandmarkYInput.value = '0';
  if (xyvizLandmarkYawInput) xyvizLandmarkYawInput.value = '0';
}

function xyvizHandleUseCurrent() {
  if (!xyvizUseCurrentBtn) return;
  if (!xyvizState.robotPose.hasPose) return;
  const pose = xyvizState.robotPose;
  if (xyvizLandmarkNameInput) {
    const timestamp = new Date().toLocaleTimeString('es-ES', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    xyvizLandmarkNameInput.value = `Pose ${timestamp}`;
  }
  if (xyvizLandmarkXInput) xyvizLandmarkXInput.value = pose.x.toFixed(3);
  if (xyvizLandmarkYInput) xyvizLandmarkYInput.value = pose.y.toFixed(3);
  if (xyvizLandmarkYawInput) xyvizLandmarkYawInput.value = radToDeg(pose.yaw).toFixed(1);
}

function xyvizUpdateRobotTelemetry(stamp) {
  if (!xyvizRobotPoseEl || !xyvizLastUpdateEl) return;
  if (!xyvizState.robotPose.hasPose) {
    xyvizRobotPoseEl.textContent = '—';
    xyvizLastUpdateEl.textContent = '—';
    return;
  }
  const pose = xyvizState.robotPose;
  xyvizRobotPoseEl.textContent = `x:${pose.x.toFixed(3)} m · y:${pose.y.toFixed(3)} m · yaw:${radToDeg(pose.yaw).toFixed(1)}°`;
  xyvizLastUpdateEl.textContent = formatTimestamp(stamp);
}

function startPlanoMonitoring() {
  if (!xyvizState.canvas || !xyvizState.ctx) return;
  xyvizInit();
  if (!window.ros || !window.ros.isConnected) return;
  if (xyvizOdomMonitor) return;
  xyvizSetPlaceholder('Esperando datos de /odometry/filtered…', true);
  xyvizOdomMonitor = new ROSLIB.Topic({
    ros: window.ros,
    name: '/odometry/filtered',
    messageType: 'nav_msgs/Odometry',
    queue_length: 1,
    throttle_rate: 0
  });
  xyvizOdomMonitor.subscribe(handlePlanoOdom);
}

function stopPlanoMonitoring(clearData = false) {
  if (xyvizOdomMonitor) {
    xyvizOdomMonitor.unsubscribe();
    xyvizOdomMonitor = null;
  }
  if (clearData) {
    xyvizState.robotPose = { x: 0, y: 0, yaw: 0, hasPose: false };
    xyvizState.trail = [];
    xyvizState.panOffset = { x: 0, y: 0 };
    xyvizResetOrigin();
    xyvizUpdateRobotTelemetry(null);
    xyvizUpdateUseCurrentAvailability();
    xyvizUpdateSelectedMetrics();
    xyvizDrawScene();
    xyvizSetPlaceholder('Conéctate a ROS para visualizar el plano.', true);
  } else if (xyvizState.canvas && document.getElementById('plano_xy')) {
    const rosConnected = !!(window.ros && window.ros.isConnected);
    const message = rosConnected
      ? 'Pestaña inactiva. Selecciona Plano XY para reanudar.'
      : 'Conéctate a ROS para visualizar el plano.';
    xyvizSetPlaceholder(message, true);
  }
}

function updatePlanoMonitoring() {
  if (!window.ros || !window.ros.isConnected) {
    stopPlanoMonitoring(false);
    return;
  }
  if (activeTab === 'plano_xy') {
    startPlanoMonitoring();
  } else {
    stopPlanoMonitoring(false);
  }
}

function handlePlanoOdom(msg) {
  if (!msg) return;
  const position = msg?.pose?.pose?.position;
  const orientation = msg?.pose?.pose?.orientation || {};
  const stamp = rosTimeToDate(msg?.header?.stamp) ?? new Date();

  const x = Number(position?.x);
  const y = Number(position?.y);
  const yaw = quaternionToYaw(orientation);

  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yaw)) return;

  xyvizState.robotPose = { x, y, yaw, hasPose: true };
  xyvizAddTrailPoint(x, y);
  xyvizUpdateRobotTelemetry(stamp);
  xyvizUpdateUseCurrentAvailability();
  xyvizUpdateSelectedMetrics();
  xyvizDrawScene();
  xyvizSetPlaceholder('', false);
}

xyvizSetPlaceholder('Conéctate a ROS para visualizar el plano.', true);
xyvizUpdateUseCurrentAvailability();
xyvizUpdateAxisButton();

// --- Calibración (odometría e IMU) ---
function clearCalibStatusTimer(key) {
  const entry = calibStatus[key];
  if (entry?.timer) {
    clearTimeout(entry.timer);
    entry.timer = null;
  }
}

function setCalibStatus(key, state, customLabel) {
  const entry = calibStatus[key];
  if (!entry?.el) return;
  const el = entry.el;
  el.classList.remove(
    'calib-status-off',
    'calib-status-waiting',
    'calib-status-active',
    'calib-status-stale',
    'calib-status-paused'
  );
  let label = customLabel;
  switch (state) {
    case 'waiting':
      el.classList.add('calib-status-waiting');
      label = label || 'Esperando datos';
      break;
    case 'active':
      el.classList.add('calib-status-active');
      label = label || 'Recibiendo';
      break;
    case 'stale':
      el.classList.add('calib-status-stale');
      label = label || 'Sin datos recientes';
      break;
    case 'paused':
      el.classList.add('calib-status-paused');
      label = label || 'En pausa';
      break;
    case 'off':
    default:
      el.classList.add('calib-status-off');
      label = label || 'Sin conexión';
      break;
  }
  el.textContent = label;
}

function scheduleCalibStale(key) {
  clearCalibStatusTimer(key);
  const entry = calibStatus[key];
  if (!entry) return;
  entry.timer = setTimeout(() => {
    setCalibStatus(key, 'stale');
    entry.timer = null;
  }, CALIB_HEARTBEAT_TIMEOUT_MS);
}

function markCalibHeartbeat(key) {
  setCalibStatus(key, 'active');
  scheduleCalibStale(key);
}

function resetCalibracionData() {
  calibracionData.encoders.x = null;
  calibracionData.encoders.y = null;
  calibracionData.encoders.theta = null;
  calibracionData.encoders.stamp = null;

  calibracionData.filtered.x = null;
  calibracionData.filtered.y = null;
  calibracionData.filtered.theta = null;
  calibracionData.filtered.stamp = null;

  calibracionData.imu.theta = null;
  calibracionData.imu.angVel.x = null;
  calibracionData.imu.angVel.y = null;
  calibracionData.imu.angVel.z = null;
  calibracionData.imu.linAcc.x = null;
  calibracionData.imu.linAcc.y = null;
  calibracionData.imu.linAcc.z = null;
  calibracionData.imu.stamp = null;
}

function setElementText(element, text) {
  if (!element) return;
  element.textContent = text;
}

function formatTimestamp(date) {
  if (!(date instanceof Date) || Number.isNaN(date.getTime())) return '—';
  return date.toLocaleTimeString('es-ES', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
  });
}

function formatLinear(value) {
  if (!Number.isFinite(value)) return '—';
  return `${value.toFixed(3)} m`;
}

function formatLinearDiff(value) {
  if (!Number.isFinite(value)) return '—';
  const sign = value >= 0 ? '+' : '';
  return `${sign}${value.toFixed(3)} m`;
}

function formatScalar(value, decimals = 3) {
  if (!Number.isFinite(value)) return '—';
  return value.toFixed(decimals);
}

function formatAngle(valueRad) {
  if (!Number.isFinite(valueRad)) return '—';
  if (calibAngleMode === 'deg') {
    return `${radToDeg(valueRad).toFixed(2)}°`;
  }
  return `${valueRad.toFixed(3)} rad`;
}

function formatAngleDiff(valueRad) {
  if (!Number.isFinite(valueRad)) return '—';
  if (calibAngleMode === 'deg') {
    const deg = radToDeg(valueRad);
    const sign = deg >= 0 ? '+' : '';
    return `${sign}${deg.toFixed(2)}°`;
  }
  const sign = valueRad >= 0 ? '+' : '';
  return `${sign}${valueRad.toFixed(3)} rad`;
}

function formatPercent(value) {
  if (!Number.isFinite(value)) return '—';
  return `${value.toFixed(1)}%`;
}

function computePercentError(desired, actual) {
  if (!Number.isFinite(desired) || Math.abs(desired) < CALIB_EPSILON) return null;
  if (!Number.isFinite(actual)) return null;
  return Math.abs((actual - desired) / desired) * 100;
}

function normalizeAngle(angle) {
  if (!Number.isFinite(angle)) return angle;
  let a = angle;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function getThetaInputValue(input) {
  const raw = Number.parseFloat(input?.value ?? '');
  if (!Number.isFinite(raw)) return 0;
  return calibAngleMode === 'deg' ? degToRad(raw) : raw;
}

function getDesiredValues() {
  const x = Number.parseFloat(calibDesiredXInput?.value ?? '');
  const y = Number.parseFloat(calibDesiredYInput?.value ?? '');
  return {
    x: Number.isFinite(x) ? x : 0,
    y: Number.isFinite(y) ? y : 0,
    theta: getThetaInputValue(calibDesiredThetaInput)
  };
}

function getOffsetValues() {
  const x = Number.parseFloat(calibOffsetXInput?.value ?? '');
  const y = Number.parseFloat(calibOffsetYInput?.value ?? '');
  return {
    x: Number.isFinite(x) ? x : 0,
    y: Number.isFinite(y) ? y : 0,
    theta: getThetaInputValue(calibOffsetThetaInput)
  };
}

function updateCalibracionDisplays() {
  const desired = getDesiredValues();
  const offsets = getOffsetValues();

  updateCalibCard('encoders', desired, offsets);
  updateCalibCard('filtered', desired, offsets);
  updateImuCard(desired, offsets);
}

function updateCalibCard(key, desired, offsets) {
  const data = calibracionData[key];
  const elements = calibElements[key];
  if (!data || !elements) return;

  const hasX = Number.isFinite(data.x);
  const hasY = Number.isFinite(data.y);
  const hasTheta = Number.isFinite(data.theta);

  const adjustedX = hasX ? data.x + offsets.x : null;
  const adjustedY = hasY ? data.y + offsets.y : null;
  const adjustedTheta = hasTheta ? normalizeAngle(data.theta + offsets.theta) : null;

  const diffX = Number.isFinite(adjustedX) ? adjustedX - desired.x : null;
  const diffY = Number.isFinite(adjustedY) ? adjustedY - desired.y : null;
  const diffTheta = Number.isFinite(adjustedTheta) ? normalizeAngle(adjustedTheta - desired.theta) : null;

  const percentX = Number.isFinite(adjustedX) ? computePercentError(desired.x, adjustedX) : null;
  const percentY = Number.isFinite(adjustedY) ? computePercentError(desired.y, adjustedY) : null;
  const percentTheta = Number.isFinite(adjustedTheta) ? computePercentError(desired.theta, adjustedTheta) : null;

  const distanceDiff = Number.isFinite(adjustedX) && Number.isFinite(adjustedY)
    ? Math.hypot(adjustedX - desired.x, adjustedY - desired.y)
    : null;

  setElementText(elements.stamp, formatTimestamp(data.stamp));
  setElementText(elements.xValue, hasX ? formatLinear(data.x) : '—');
  setElementText(elements.yValue, hasY ? formatLinear(data.y) : '—');
  setElementText(elements.thetaValue, hasTheta ? formatAngle(data.theta) : '—');

  setElementText(
    elements.xDiff,
    Number.isFinite(diffX)
      ? `Diff: ${formatLinearDiff(diffX)} (ajustado: ${formatLinear(adjustedX)})`
      : 'Diff: —'
  );
  setElementText(
    elements.yDiff,
    Number.isFinite(diffY)
      ? `Diff: ${formatLinearDiff(diffY)} (ajustado: ${formatLinear(adjustedY)})`
      : 'Diff: —'
  );
  setElementText(
    elements.thetaDiff,
    Number.isFinite(diffTheta)
      ? `Diff: ${formatAngleDiff(diffTheta)} (ajustado: ${formatAngle(adjustedTheta)})`
      : 'Diff: —'
  );

  setElementText(
    elements.xError,
    Number.isFinite(percentX) ? `Error: ${formatPercent(percentX)}` : 'Error: —'
  );
  setElementText(
    elements.yError,
    Number.isFinite(percentY) ? `Error: ${formatPercent(percentY)}` : 'Error: —'
  );
  setElementText(
    elements.thetaError,
    Number.isFinite(percentTheta) ? `Error: ${formatPercent(percentTheta)}` : 'Error: —'
  );

  setElementText(
    elements.distance,
    Number.isFinite(distanceDiff) ? formatLinear(distanceDiff) : '—'
  );
}

function updateImuCard(desired, offsets) {
  const data = calibracionData.imu;
  const elements = calibElements.imu;
  if (!data || !elements) return;

  const hasTheta = Number.isFinite(data.theta);
  const adjustedTheta = hasTheta ? normalizeAngle(data.theta + offsets.theta) : null;
  const diffTheta = Number.isFinite(adjustedTheta) ? normalizeAngle(adjustedTheta - desired.theta) : null;
  const percentTheta = Number.isFinite(adjustedTheta) ? computePercentError(desired.theta, adjustedTheta) : null;

  setElementText(elements.stamp, formatTimestamp(data.stamp));
  setElementText(elements.thetaValue, hasTheta ? formatAngle(data.theta) : '—');
  setElementText(
    elements.thetaDiff,
    Number.isFinite(diffTheta)
      ? `Diff: ${formatAngleDiff(diffTheta)} (ajustado: ${formatAngle(adjustedTheta)})`
      : 'Diff: —'
  );
  setElementText(
    elements.thetaError,
    Number.isFinite(percentTheta) ? `Error: ${formatPercent(percentTheta)}` : 'Error: —'
  );

  setElementText(elements.angX, formatScalar(calibracionData.imu.angVel.x));
  setElementText(elements.angY, formatScalar(calibracionData.imu.angVel.y));
  setElementText(elements.angZ, formatScalar(calibracionData.imu.angVel.z));

  setElementText(elements.linX, formatScalar(calibracionData.imu.linAcc.x));
  setElementText(elements.linY, formatScalar(calibracionData.imu.linAcc.y));
  setElementText(elements.linZ, formatScalar(calibracionData.imu.linAcc.z));
}

function startCalibracionMonitoring() {
  if (!window.ros || !window.ros.isConnected) return;

  if (!calibEncodersMonitor) {
    clearCalibStatusTimer('encoders');
    setCalibStatus('encoders', 'waiting');
    calibEncodersMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: '/diff_drive_controller/odom',
      messageType: 'nav_msgs/Odometry',
      queue_length: 1,
      throttle_rate: 0
    });
    calibEncodersMonitor.subscribe(handleEncodersOdom);
  }

  if (!calibFilteredMonitor) {
    clearCalibStatusTimer('filtered');
    setCalibStatus('filtered', 'waiting');
    calibFilteredMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: '/odometry/filtered',
      messageType: 'nav_msgs/Odometry',
      queue_length: 1,
      throttle_rate: 0
    });
    calibFilteredMonitor.subscribe(handleFilteredOdom);
  }

  if (!calibImuMonitor) {
    clearCalibStatusTimer('imu');
    setCalibStatus('imu', 'waiting');
    calibImuMonitor = new ROSLIB.Topic({
      ros: window.ros,
      name: '/imu/data',
      messageType: 'sensor_msgs/Imu',
      queue_length: 1,
      throttle_rate: 0
    });
    calibImuMonitor.subscribe(handleImuData);
  }
}

function stopCalibracionMonitoring(clearData = false) {
  if (calibEncodersMonitor) {
    calibEncodersMonitor.unsubscribe();
    calibEncodersMonitor = null;
  }
  clearCalibStatusTimer('encoders');
  if (calibFilteredMonitor) {
    calibFilteredMonitor.unsubscribe();
    calibFilteredMonitor = null;
  }
  clearCalibStatusTimer('filtered');
  if (calibImuMonitor) {
    calibImuMonitor.unsubscribe();
    calibImuMonitor = null;
  }
  clearCalibStatusTimer('imu');
  if (clearData) {
    resetCalibracionData();
    updateCalibracionDisplays();
    setCalibStatus('encoders', 'off');
    setCalibStatus('filtered', 'off');
    setCalibStatus('imu', 'off');
  } else {
    const rosConnected = !!(window.ros && window.ros.isConnected);
    setCalibStatus('encoders', rosConnected ? 'paused' : 'off');
    setCalibStatus('filtered', rosConnected ? 'paused' : 'off');
    setCalibStatus('imu', rosConnected ? 'paused' : 'off');
  }
}

function handleEncodersOdom(msg) {
  const position = msg?.pose?.pose?.position;
  const orientation = msg?.pose?.pose?.orientation || {};
  const stamp = rosTimeToDate(msg?.header?.stamp) ?? new Date();

  const x = Number(position?.x);
  const y = Number(position?.y);
  const yaw = quaternionToYaw(orientation);

  calibracionData.encoders.x = Number.isFinite(x) ? x : null;
  calibracionData.encoders.y = Number.isFinite(y) ? y : null;
  calibracionData.encoders.theta = Number.isFinite(yaw) ? yaw : null;
  calibracionData.encoders.stamp = stamp;

  markCalibHeartbeat('encoders');
  updateCalibracionDisplays();
}

function updateCalibracionMonitoring() {
  if (!window.ros || !window.ros.isConnected) {
    stopCalibracionMonitoring(false);
    return;
  }
  if (activeTab === 'calibracion') {
    startCalibracionMonitoring();
  } else {
    stopCalibracionMonitoring(false);
  }
}

function handleFilteredOdom(msg) {
  const position = msg?.pose?.pose?.position;
  const orientation = msg?.pose?.pose?.orientation || {};
  const stamp = rosTimeToDate(msg?.header?.stamp) ?? new Date();

  const x = Number(position?.x);
  const y = Number(position?.y);
  const yaw = quaternionToYaw(orientation);

  calibracionData.filtered.x = Number.isFinite(x) ? x : null;
  calibracionData.filtered.y = Number.isFinite(y) ? y : null;
  calibracionData.filtered.theta = Number.isFinite(yaw) ? yaw : null;
  calibracionData.filtered.stamp = stamp;

  markCalibHeartbeat('filtered');
  updateCalibracionDisplays();
}

function handleImuData(msg) {
  const orientation = msg?.orientation || {};
  const angVel = msg?.angular_velocity || {};
  const linAcc = msg?.linear_acceleration || {};
  const stamp = rosTimeToDate(msg?.header?.stamp) ?? new Date();

  const yaw = quaternionToYaw(orientation);

  calibracionData.imu.theta = Number.isFinite(yaw) ? yaw : null;
  calibracionData.imu.angVel.x = Number.isFinite(Number(angVel?.x)) ? Number(angVel.x) : null;
  calibracionData.imu.angVel.y = Number.isFinite(Number(angVel?.y)) ? Number(angVel.y) : null;
  calibracionData.imu.angVel.z = Number.isFinite(Number(angVel?.z)) ? Number(angVel.z) : null;

  calibracionData.imu.linAcc.x = Number.isFinite(Number(linAcc?.x)) ? Number(linAcc.x) : null;
  calibracionData.imu.linAcc.y = Number.isFinite(Number(linAcc?.y)) ? Number(linAcc.y) : null;
  calibracionData.imu.linAcc.z = Number.isFinite(Number(linAcc?.z)) ? Number(linAcc.z) : null;
  calibracionData.imu.stamp = stamp;

  markCalibHeartbeat('imu');
  updateCalibracionDisplays();
}

function convertAngleInputValue(input, fromMode, toMode) {
  if (!input) return;
  const current = Number.parseFloat(input.value);
  if (!Number.isFinite(current)) return;

  let converted = current;
  if (fromMode === 'rad' && toMode === 'deg') {
    converted = radToDeg(current);
  } else if (fromMode === 'deg' && toMode === 'rad') {
    converted = degToRad(current);
  }

  const decimals = toMode === 'deg' ? 2 : 3;
  input.value = Number.isFinite(converted) ? converted.toFixed(decimals) : '';
}

function updateAngleModeUI() {
  if (calibDesiredThetaLabel) {
    calibDesiredThetaLabel.textContent = calibAngleMode === 'rad' ? 'Yaw (rad)' : 'Yaw (°)';
  }
  if (calibOffsetThetaLabel) {
    calibOffsetThetaLabel.textContent = calibAngleMode === 'rad' ? 'Offset yaw (rad)' : 'Offset yaw (°)';
  }
  if (calibAngleModeHint) {
    calibAngleModeHint.textContent = calibAngleMode === 'rad'
      ? 'Mostrando ángulos en radianes.'
      : 'Mostrando ángulos en grados.';
  }
  if (calibToggleAngleModeBtn) {
    calibToggleAngleModeBtn.textContent = calibAngleMode === 'rad'
      ? 'Ver en grados'
      : 'Ver en radianes';
  }
  if (calibDesiredThetaInput) {
    calibDesiredThetaInput.step = calibAngleMode === 'rad' ? '0.01' : '1';
  }
  if (calibOffsetThetaInput) {
    calibOffsetThetaInput.step = calibAngleMode === 'rad' ? '0.01' : '1';
  }
}

function toggleCalibAngleMode() {
  const newMode = calibAngleMode === 'rad' ? 'deg' : 'rad';
  convertAngleInputValue(calibDesiredThetaInput, calibAngleMode, newMode);
  convertAngleInputValue(calibOffsetThetaInput, calibAngleMode, newMode);
  calibAngleMode = newMode;
  updateAngleModeUI();
  updateCalibracionDisplays();
}

const calibInputs = [
  calibDesiredXInput,
  calibDesiredYInput,
  calibDesiredThetaInput,
  calibOffsetXInput,
  calibOffsetYInput,
  calibOffsetThetaInput
].filter(Boolean);

calibInputs.forEach(input => {
  input.addEventListener('input', () => updateCalibracionDisplays());
});

if (calibToggleAngleModeBtn) {
  calibToggleAngleModeBtn.addEventListener('click', toggleCalibAngleMode);
}

resetCalibracionData();
updateAngleModeUI();
updateCalibracionDisplays();

function showSlamServiceFeedback(text, isError = false) {
  if (!slamServiceFeedback) return;
  slamServiceFeedback.textContent = text;
  slamServiceFeedback.style.color = isError ? '#ffb3b3' : '#a5d6a7';
}

function requestDynamicMap(button) {
  if (!window.ros || !window.ros.isConnected) {
    showSlamServiceFeedback('No hay conexión con ROS. Conéctate antes de solicitar el mapa.', true);
    return;
  }
  if (!button) return;
  button.disabled = true;
  showSlamServiceFeedback('Solicitando snapshot del mapa...', false);
  if (!dynamicMapService) {
    dynamicMapService = new ROSLIB.Service({
      ros: window.ros,
      name: '/slam_toolbox/dynamic_map',
      serviceType: 'nav_msgs/srv/GetMap'
    });
  }
  dynamicMapService.callService(new ROSLIB.ServiceRequest({}), respuesta => {
    button.disabled = false;
    const mapa = respuesta?.map || respuesta;
    if (mapa && mapa.info) {
      updateMapMetadata(mapa);
      markSlamHeartbeat('dynamic_map');
      showSlamServiceFeedback('Snapshot del mapa recibido correctamente.', false);
    } else {
      showSlamServiceFeedback('Respuesta vacía del servicio dynamic_map.', true);
    }
  }, error => {
    button.disabled = false;
    showSlamServiceFeedback(`Error al solicitar dynamic_map: ${error}`, true);
  });
}

function executeSlamServiceButton(button) {
  if (!button || !button.dataset) return;
  if (!window.ros || !window.ros.isConnected) {
    showSlamServiceFeedback('No hay conexión con ROS. No se puede ejecutar el servicio.', true);
    return;
  }

  const serviceName = button.dataset.slamService;
  const serviceType = button.dataset.slamType;
  if (!serviceName || !serviceType) return;

  let requestPayload = {};

  switch (serviceType) {
    case 'slam_toolbox/DeserializePoseGraph': {
      const filename = prompt('Ruta del archivo serializado a cargar (.posegraph/.data):', '');
      if (!filename) {
        showSlamServiceFeedback('Operación cancelada. Se requiere una ruta válida.', true);
        return;
      }
      const matchPrompt = prompt('Modo de arranque (0=UNSET, 1=START_AT_FIRST_NODE, 2=START_AT_GIVEN_POSE, 3=LOCALIZE_AT_POSE):', '1');
      const matchType = Number.parseInt(matchPrompt ?? '1', 10);
      let initialPose = { x: 0, y: 0, theta: 0 };
      if (matchType === 2 || matchType === 3) {
        const posePrompt = prompt('Introduce la pose inicial (x y theta) en metros y radianes, separados por espacio:', '0 0 0');
        if (!posePrompt) {
          showSlamServiceFeedback('Carga cancelada. Falta la pose inicial.', true);
          return;
        }
        const [xStr, yStr, thetaStr] = posePrompt.trim().split(/\s+/);
        initialPose = {
          x: Number.parseFloat(xStr ?? '0') || 0,
          y: Number.parseFloat(yStr ?? '0') || 0,
          theta: Number.parseFloat(thetaStr ?? '0') || 0
        };
      }
      requestPayload = {
        filename,
        match_type: Number.isFinite(matchType) ? matchType : 1,
        initial_pose: initialPose
      };
      break;
    }
    case 'slam_toolbox/SerializePoseGraph': {
      const filename = prompt('Nombre base del archivo de salida (sin extensión):', 'posegraph');
      if (!filename) {
        showSlamServiceFeedback('Serialización cancelada. Se requiere un nombre.', true);
        return;
      }
      requestPayload = { filename };
      break;
    }
    case 'slam_toolbox/SaveMap': {
      const mapName = prompt('Nombre base del mapa a guardar (generará .pgm y .yaml):', 'slam_toolbox_map');
      if (!mapName) {
        showSlamServiceFeedback('Guardado cancelado. Se requiere un nombre.', true);
        return;
      }
      requestPayload = { name: { data: mapName } };
      break;
    }
    case 'slam_toolbox/Reset': {
      const shouldPause = confirm('¿Deseas pausar nuevas mediciones inmediatamente después del reset?');
      requestPayload = { pause_new_measurements: shouldPause };
      break;
    }
    default:
      requestPayload = {};
      break;
  }

  button.disabled = true;
  showSlamServiceFeedback(`Ejecutando ${serviceName}...`, false);

  const servicio = new ROSLIB.Service({
    ros: window.ros,
    name: serviceName,
    serviceType
  });

  servicio.callService(new ROSLIB.ServiceRequest(requestPayload), respuesta => {
    button.disabled = false;
    if (serviceType === 'slam_toolbox/Pause' && respuesta && 'status' in respuesta) {
      showSlamServiceFeedback(`Pausa de nuevas mediciones: ${respuesta.status ? 'activada' : 'desactivada'}.`, false);
    } else if (respuesta && Object.keys(respuesta).length) {
      showSlamServiceFeedback(`Servicio ${serviceName} completado (${JSON.stringify(respuesta)}).`, false);
    } else {
      showSlamServiceFeedback(`Servicio ${serviceName} completado.`, false);
    }
  }, error => {
    button.disabled = false;
    showSlamServiceFeedback(`Error al ejecutar ${serviceName}: ${error}`, true);
  });
}

// EXPONER la misma instancia al resto del dashboard
window.ros = new ROSLIB.Ros({ url: 'ws://192.168.0.162:9090' });
const ros = window.ros;

// Eventos de conexión
ros.on('connection', function () {
  if (rosStatus)   rosStatus.textContent   = '🟢 Conectado';
  if (topicStatus) topicStatus.textContent = '—';
  startSlamMonitoring();
  startLidarMonitoring();
  startImuTempMonitoring();
  updateCalibracionMonitoring();
  updatePlanoMonitoring();
  updateNavegacionMonitoring();
  if (activeTab === 'mapa') {
    ensureRos2dViewer();
  }
});

ros.on('close', function () {
  if (rosStatus)   rosStatus.textContent   = '🔴 Desconectado';
  if (topicStatus) topicStatus.textContent = '—';
  stopSlamMonitoring();
  stopLidarMonitoring();
  stopImuTempMonitoring(true);
  stopCalibracionMonitoring(true);
  stopPlanoMonitoring(true);
  deactivateNavSalidas();
  setSlamIndicator('disconnected', 'SLAMToolbox: Sin conexión');
  setSlamLastUpdate(null);
  destroyRos2dViewer();
  renderMapaPlaceholder('Conéctate a ROS para visualizar el mapa.');
});

ros.on('error', function () {
  if (rosStatus)   rosStatus.textContent   = '⚠️ Error';
  if (topicStatus) topicStatus.textContent = '—';
  stopSlamMonitoring();
  stopLidarMonitoring();
  stopImuTempMonitoring(true);
  stopCalibracionMonitoring(true);
  stopPlanoMonitoring(true);
  deactivateNavSalidas();
  setSlamIndicator('disconnected', 'SLAMToolbox: Error de conexión');
  setSlamLastUpdate(null);
  destroyRos2dViewer();
  renderMapaPlaceholder('Error al conectar con ROS. Reintenta para recuperar el mapa.');
});

// (Opcional) Auto-reconexión simple
setInterval(() => {
  try {
    if (!ros.isConnected) ros.connect('ws://192.168.0.162:9090');
  } catch (_) {}
}, 3000);


  // ========================
  // 2. Inicialización de la gráfica Chart.js
  // ========================
  const ctx = document.getElementById('miGrafica');
  let chart = null;

  if (ctx) {
    chart = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [{
          label: 'ROS Data',
          data: []
        }]
      },
      options: {
        scales: {
          x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Eje X' } },
          y: { title: { display: true, text: 'Eje Y' } }
        }
      }
    });
  }

  // ========================
  // 3. Suscripción al tópico de ROS y actualización de datos
  // ========================
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/datos_grafica',
    messageType: 'std_msgs/Float32MultiArray'
  });

  const MAX_PUNTOS = 100;
  const RESET_TIMEOUT_MS = 3000;
  let resetTimeoutId = null;

  listener.subscribe(function(message) {
    if (topicStatus) topicStatus.textContent = '🟢 Recibiendo datos (' + new Date().toLocaleTimeString() + ')';
    if (datosRos) datosRos.textContent = JSON.stringify(message.data, null, 2);

    if (resetTimeoutId) clearTimeout(resetTimeoutId);
    resetTimeoutId = setTimeout(() => {
      chart.data.datasets[0].data = [];
      chart.update();
      if (topicStatus) topicStatus.textContent = '⏹️ Sin datos recientes (historial borrado)';
    }, RESET_TIMEOUT_MS);

    if (chart && message.data && message.data.length > 1) {
      for (let i = 0; i < message.data.length; i += 2) {
        if (message.data[i + 1] !== undefined) {
          chart.data.datasets[0].data.push({ x: message.data[i], y: message.data[i + 1] });
        }
      }
      if (chart.data.datasets[0].data.length > MAX_PUNTOS) {
        chart.data.datasets[0].data = chart.data.datasets[0].data.slice(-MAX_PUNTOS);
      }
      chart.update();
    }
  });

// ========================
// 4. Sistema de pestañas en el dashboard
// ========================
const sections = document.querySelectorAll('main > section');
const navLinks = document.querySelectorAll('nav a[data-tab]');
let activeTab = null;

function showTab(tabId) {
  sections.forEach(section => {
    section.style.display = (section.id === tabId) ? 'block' : 'none';
  });
  navLinks.forEach(link => {
    link.classList.toggle('active', link.dataset.tab === tabId);
  });
  activeTab = tabId;

  // --- Refresca dropdown de monitoreo si es la pestaña 'monitoreo' ---
  if (tabId === 'monitoreo') {
    // Espera a que el DOM haga visible la sección antes de poblar el dropdown
    setTimeout(actualizarMonitorDropdownAuto, 30);
  }

  if (tabId === 'mapa') {
    ensureRos2dViewer();
  }

  if (tabId === 'plano_xy') {
    xyvizInit();
  }

  updateCalibracionMonitoring();
  updatePlanoMonitoring();
  updateNavegacionMonitoring();
}

function getTabFromHash() {
  const hash = window.location.hash.replace('#', '');
  const tabIds = Array.from(sections).map(sec => sec.id);
  return tabIds.includes(hash) ? hash : tabIds[0];
}

function handleHashChange() {
  showTab(getTabFromHash());
}

// Permite navegación por menú (clic en el nav)
navLinks.forEach(link => {
  link.addEventListener('click', e => {
    // Previene scroll automático del navegador
    e.preventDefault();
    const tabId = link.getAttribute('href').replace('#', '');
    window.location.hash = tabId;
    // La función handleHashChange se dispara automáticamente por el hashchange
  });
});

// Carga la pestaña correcta al cargar la página y al cambiar el hash
window.addEventListener('hashchange', handleHashChange);
window.addEventListener('DOMContentLoaded', handleHashChange);

// Tabs internos de la pestaña Menú
const menuNavLinks = document.querySelectorAll('.menu-nav-link');
const menuPanels = document.querySelectorAll('.menu-panel');

function showMenuPanel(target) {
  if (!menuPanels.length || !menuNavLinks.length) return;
  const targetId = `menu-${target}`;
  menuPanels.forEach(panel => panel.classList.toggle('active', panel.id === targetId));
  menuNavLinks.forEach(link => link.classList.toggle('active', link.dataset.menuTarget === target));
}

menuNavLinks.forEach(link => {
  link.addEventListener('click', () => showMenuPanel(link.dataset.menuTarget));
});

showMenuPanel('comida');

const menuMesaInputs = document.querySelectorAll('.menu-mesa-input');

function updateMesaLabel(mesaId) {
  const xInput = document.querySelector(`.menu-mesa-input[data-mesa="${mesaId}"][data-axis="x"]`);
  const yInput = document.querySelector(`.menu-mesa-input[data-mesa="${mesaId}"][data-axis="y"]`);
  const label = document.querySelector(`.menu-mesa-current[data-mesa="${mesaId}"]`);
  if (!label || !xInput || !yInput) return;

  const xVal = Number.parseFloat(xInput.value);
  const yVal = Number.parseFloat(yInput.value);
  const format = value => (Number.isFinite(value) ? value.toFixed(2) : '—');
  label.textContent = `Actual: X=${format(xVal)}, Y=${format(yVal)}`;
}

menuMesaInputs.forEach(input => {
  input.addEventListener('input', () => updateMesaLabel(input.dataset.mesa));
  updateMesaLabel(input.dataset.mesa);
});

if (dynamicMapButton) {
  dynamicMapButton.addEventListener('click', () => requestDynamicMap(dynamicMapButton));
}

const slamServiceButtons = document.querySelectorAll('.slam-action-btn[data-slam-service]');
slamServiceButtons.forEach(btn => {
  btn.addEventListener('click', () => executeSlamServiceButton(btn));
});

if (xyvizAxisToggle) {
  xyvizAxisToggle.addEventListener('click', xyvizToggleAxisMode);
}

if (xyvizResetViewBtn) {
  xyvizResetViewBtn.addEventListener('click', xyvizResetView);
}

if (xyvizZoomInBtn) {
  xyvizZoomInBtn.addEventListener('click', () => xyvizApplyZoom(1.15, xyvizState.width / 2, xyvizState.height / 2));
}

if (xyvizZoomOutBtn) {
  xyvizZoomOutBtn.addEventListener('click', () => xyvizApplyZoom(0.85, xyvizState.width / 2, xyvizState.height / 2));
}

if (xyvizLandmarkForm) {
  xyvizLandmarkForm.addEventListener('submit', xyvizHandleLandmarkSubmit);
}

if (xyvizUseCurrentBtn) {
  xyvizUseCurrentBtn.addEventListener('click', () => xyvizHandleUseCurrent());
}

if (xyPoiForm) {
  xyPoiForm.addEventListener('submit', event => {
    event.preventDefault();
    const rawName = xyNameInput?.value ?? '';
    const rawX = xyXInput ? Number.parseFloat(xyXInput.value) : 0;
    const rawY = xyYInput ? Number.parseFloat(xyYInput.value) : 0;
    const rawYaw = xyYawInput ? Number.parseFloat(xyYawInput.value) : NaN;

    const xValue = Number.isFinite(rawX) ? rawX : 0;
    const yValue = Number.isFinite(rawY) ? rawY : 0;
    const yawRad = Number.isFinite(rawYaw) ? degToRad(rawYaw) : undefined;

    addPoi({
      name: rawName,
      x: xValue,
      y: yValue,
      yaw: yawRad
    });

    if (xyNameInput) xyNameInput.value = '';
    if (xyXInput) xyXInput.value = '0';
    if (xyYInput) xyYInput.value = '0';
    if (xyYawInput) xyYawInput.value = '0';
  });
}

if (xyUseCurrentBtn) {
  xyUseCurrentBtn.addEventListener('click', () => {
    if (!xyPose.hasPose) return;
    const timestamp = new Date().toLocaleTimeString('es-ES', {
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
    addPoi({
      name: `Pose ${timestamp}`,
      x: xyPose.x,
      y: xyPose.y,
      yaw: xyPose.yaw
    });
  });
  xyUseCurrentBtn.disabled = true;
}

refreshPoiList();

// ========================
// 5. Visualización de Nodos y Tópicos ROS
// ========================
const contenedorNodos = document.getElementById('contenedor-nodos');
const contenedorTopicos = document.getElementById('contenedor-topicos');
const btnActualizar = document.getElementById('btn-actualizar-nodos-topicos');
const dropdownSeleccion = document.getElementById('dropdown-seleccion');
const dropdownComando = document.getElementById('dropdown-comando');
const resultadoComando = document.getElementById('resultado-comando');
const btnConsultar = document.getElementById('btn-consultar-comando');

function actualizarListaNodos() {
  if (!ros.isConnected) {
    contenedorNodos.innerHTML = '<div style="color:#888;">No conectado a ROS</div>';
    return;
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/nodes',
    serviceType: 'rosapi/GetNodes'
  });
  service.callService(new ROSLIB.ServiceRequest({}), function(result) {
    if (result.nodes && result.nodes.length > 0) {
      contenedorNodos.innerHTML =
        `<ul style="font-family:monospace; font-size:1.08em; padding-left:1em; margin:0;">
          ${result.nodes.map(nodo => `<li>${nodo}</li>`).join('')}
        </ul>`;
    } else {
      contenedorNodos.innerHTML = '<div style="color:#888;">No hay nodos activos.</div>';
    }
  }, function(error) {
    contenedorNodos.innerHTML = '<div style="color:#c00;">Error al consultar nodos.</div>';
  });
}

function actualizarListaTopicos() {
  if (!ros.isConnected) {
    contenedorTopicos.innerHTML = '<div style="color:#888;">No conectado a ROS</div>';
    return;
  }
  const service = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/topics',
    serviceType: 'rosapi/GetTopics'
  });
  service.callService(new ROSLIB.ServiceRequest({}), function(result) {
    if (result.topics && result.topics.length > 0) {
      contenedorTopicos.innerHTML =
        `<ul style="font-family:monospace; font-size:1.08em; padding-left:1em; margin:0;">
          ${result.topics.map(topico => `<li>${topico}</li>`).join('')}
        </ul>`;
    } else {
      contenedorTopicos.innerHTML = '<div style="color:#888;">No hay tópicos activos.</div>';
    }
  }, function(error) {
    contenedorTopicos.innerHTML = '<div style="color:#c00;">Error al consultar tópicos.</div>';
  });
}

// --- INTEGRADO: Dropdown avanzado (nodos/tópicos y comandos) ---
function limpiarDropdownSeleccion() {
  if (!dropdownSeleccion) return;
  dropdownSeleccion.innerHTML = '<option value="" disabled selected>Selecciona nodo o tópico</option>';
}

function poblarDropdownSeleccion(nodos, topicos) {
  if (!dropdownSeleccion) return;
  limpiarDropdownSeleccion();
  if (topicos && topicos.length) {
    const group = document.createElement('optgroup');
    group.label = "Tópicos";
    topicos.forEach(t => {
      const opt = document.createElement('option');
      opt.value = "topic:" + t;
      opt.textContent = t;
      group.appendChild(opt);
    });
    dropdownSeleccion.appendChild(group);
  }
  if (nodos && nodos.length) {
    const group = document.createElement('optgroup');
    group.label = "Nodos";
    nodos.forEach(n => {
      const opt = document.createElement('option');
      opt.value = "node:" + n;
      opt.textContent = n;
      group.appendChild(opt);
    });
    dropdownSeleccion.appendChild(group);
  }
}

function actualizarNodosYTopicosYDropdown() {
  actualizarListaNodos();
  actualizarListaTopicos();

  if (!ros || !ros.isConnected) {
    limpiarDropdownSeleccion();
    if (resultadoComando) resultadoComando.textContent = "No conectado a ROS.";
    return;
  }
  let nodos = [], topicos = [];
  let nodosReady = false, topicosReady = false;
  const checkReady = () => {
    if (nodosReady && topicosReady) {
      poblarDropdownSeleccion(nodos, topicos);
    }
  };
  const srvNodos = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/nodes',
    serviceType: 'rosapi/GetNodes'
  });
  srvNodos.callService(new ROSLIB.ServiceRequest({}), function(res) {
    nodos = res.nodes || [];
    nodosReady = true;
    checkReady();
  }, function() {
    nodos = [];
    nodosReady = true;
    checkReady();
  });
  const srvTopicos = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/topics',
    serviceType: 'rosapi/GetTopics'
  });
  srvTopicos.callService(new ROSLIB.ServiceRequest({}), function(res) {
    topicos = res.topics || [];
    topicosReady = true;
    checkReady();
  }, function() {
    topicos = [];
    topicosReady = true;
    checkReady();
  });
}

// Actualizar al presionar el botón y al abrir la pestaña
if (btnActualizar) {
  btnActualizar.addEventListener('click', actualizarNodosYTopicosYDropdown);
}
window.addEventListener('hashchange', function() {
  if (window.location.hash.replace('#', '') === 'nodos_topicos') {
    actualizarNodosYTopicosYDropdown();
  }
});
if (window.location.hash.replace('#', '') === 'nodos_topicos') {
  actualizarNodosYTopicosYDropdown();
}

// === Lógica de ejecución de comandos con el BOTÓN Consultar ===
if (btnConsultar) {
  btnConsultar.addEventListener('click', function() {
    if (!dropdownSeleccion || !resultadoComando || !dropdownComando) return;
    resultadoComando.textContent = '';
    const valor = dropdownSeleccion.value;
    if (!valor) {
      resultadoComando.textContent = 'Primero selecciona un nodo o tópico.';
      return;
    }
    const [tipo, nombre] = valor.split(':');
    const comando = dropdownComando.value;

    if (!comando) {
      resultadoComando.textContent = 'Selecciona un comando.';
      return;
    }

    if (comando === 'info') {
      if (tipo === 'topic') {
        const srvTipo = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/topic_type',
          serviceType: 'rosapi/TopicType'
        });
        srvTipo.callService(new ROSLIB.ServiceRequest({ topic: nombre }), function(res) {
          resultadoComando.textContent = `Tipo de mensaje: ${res.type}`;
        }, function() {
          resultadoComando.textContent = "Error obteniendo tipo de mensaje.";
        });
      } else if (tipo === 'node') {
        const srv = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/node_topics',
          serviceType: 'rosapi/NodeTopics'
        });
        srv.callService(new ROSLIB.ServiceRequest({ node: nombre }), function(res) {
          resultadoComando.textContent =
            `Publica:\n${(res.publishing || []).join('\n')}\n\nSuscribe:\n${(res.subscribing || []).join('\n')}`;
        }, function() {
          resultadoComando.textContent = "Error obteniendo información del nodo.";
        });
      }
    }
    else if (comando === 'publishers' && tipo === 'topic') {
      const srv = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/publishers',
        serviceType: 'rosapi/Publishers'
      });
      srv.callService(new ROSLIB.ServiceRequest({ topic: nombre }), function(res) {
        resultadoComando.textContent = "Publishers:\n" + (res.publishers || []).join('\n');
      }, function() {
        resultadoComando.textContent = "Error obteniendo publishers.";
      });
    }
    else if (comando === 'subscribers' && tipo === 'topic') {
      const srv = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/subscribers',
        serviceType: 'rosapi/Subscribers'
      });
      srv.callService(new ROSLIB.ServiceRequest({ topic: nombre }), function(res) {
        resultadoComando.textContent = "Subscribers:\n" + (res.subscribers || []).join('\n');
      }, function() {
        resultadoComando.textContent = "Error obteniendo subscribers.";
      });
    }
    else if (comando === 'node_topics' && tipo === 'node') {
      const srv = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/node_topics',
        serviceType: 'rosapi/NodeTopics'
      });
      srv.callService(new ROSLIB.ServiceRequest({ node: nombre }), function(res) {
        resultadoComando.textContent =
          `Publica:\n${(res.publishing || []).join('\n')}\n\nSuscribe:\n${(res.subscribing || []).join('\n')}`;
      }, function() {
        resultadoComando.textContent = "Error obteniendo información del nodo.";
      });
    }
    else if (comando === 'topic_type' && tipo === 'topic') {
      const srv = new ROSLIB.Service({
        ros: ros,
        name: '/rosapi/topic_type',
        serviceType: 'rosapi/TopicType'
      });
      srv.callService(new ROSLIB.ServiceRequest({ topic: nombre }), function(res) {
        resultadoComando.textContent = `Tipo de mensaje: ${res.type}`;
      }, function() {
        resultadoComando.textContent = "Error obteniendo tipo de mensaje.";
      });
    }
    else {
      resultadoComando.textContent = 'Este comando no está implementado para el elemento seleccionado.';
    }
  });
}

// Limpia selección anterior al cambiar el dropdown
if (dropdownSeleccion) {
  dropdownSeleccion.addEventListener('change', function() {
    if (resultadoComando) resultadoComando.textContent = '';
    if (dropdownComando) dropdownComando.selectedIndex = 0;
  });
}

  // ==================== Monitor Dinámico de Tópicos ====================
const monitorDropdownTopico = document.getElementById('monitor-dropdown-topico');
const monitorAddBtn = document.getElementById('monitor-add-btn');
const monitorListaTopicos = document.getElementById('monitor-lista-topicos');

let monitorSubs = {}; // { 'topic_name': {sub: ROSLIB.Topic, lastValue: ...} }

function actualizarDropdownMonitorTopicos(topicos) {
  if (!monitorDropdownTopico) return;
  monitorDropdownTopico.innerHTML = '<option value="" disabled selected>Selecciona tópico</option>';
  (topicos || []).forEach(t => {
    const opt = document.createElement('option');
    opt.value = t;
    opt.textContent = t;
    monitorDropdownTopico.appendChild(opt);
  });
}

// Llama esto en tu función donde poblas dropdowns, ejemplo después de poblarDropdownSeleccion()
function actualizarMonitorDropdownAuto() {
  // Consulta los tópicos activos y actualiza el dropdown
  const srvTopicos = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/topics',
    serviceType: 'rosapi/GetTopics'
  });
  srvTopicos.callService(new ROSLIB.ServiceRequest({}), function(res) {
    actualizarDropdownMonitorTopicos(res.topics || []);
  }, function() {
    actualizarDropdownMonitorTopicos([]);
  });
}

// Botón para agregar monitoreo
if (monitorAddBtn && monitorDropdownTopico) {
  monitorAddBtn.addEventListener('click', function () {
    const topico = monitorDropdownTopico.value;
    if (!topico || monitorSubs[topico]) return;
    // Consulta tipo de mensaje antes de subscribirse
    const srvTipo = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/topic_type',
      serviceType: 'rosapi/TopicType'
    });
    srvTipo.callService(new ROSLIB.ServiceRequest({ topic: topico }), function (res) {
      agregarMonitorTopico(topico, res.type || '');
    });
  });
}



// Función para crear la suscripción y mostrar datos
function agregarMonitorTopico(topico, tipoMsg) {
  if (!ros.isConnected) return;
  const sub = new ROSLIB.Topic({
    ros: ros,
    name: topico,
    messageType: tipoMsg || 'std_msgs/String', // por si acaso
    throttle_rate: 250
  });
  monitorSubs[topico] = { sub, lastValue: null, tipoMsg };
  // Crear fila de monitoreo
  const divFila = document.createElement('div');
  divFila.className = "monitor-fila";
  divFila.style.cssText = "padding:0.6em 1em; margin-bottom:0.4em; border:1px solid #eee; border-radius:8px; background:#f8faff; font-family:monospace;";
  divFila.innerHTML = `
    <b style="color:#2987d6;">${topico}</b>
    <span style="color:#888; margin-left:1em;">(${tipoMsg})</span>
    <button class="monitor-remove-btn" style="margin-left:1em;">✖</button>
    <pre style="margin:0.5em 0 0 0; font-size:1.05em; height:100px; overflow:auto; background:#eef6ff; border-radius:7px;">
      <span class="monitor-valor"></span>
    </pre>
  `;
  
  // Botón de remover
  divFila.querySelector('.monitor-remove-btn').onclick = function () {
    sub.unsubscribe();
    divFila.remove();
    delete monitorSubs[topico];
  };
  monitorListaTopicos.appendChild(divFila);

  // Subscribirse y actualizar valor
  sub.subscribe(function(msg) {
    monitorSubs[topico].lastValue = msg;
    const span = divFila.querySelector('.monitor-valor');
    if (span) {
      span.textContent = JSON.stringify(msg, null, 2);
    }
  });
}

// --- Cuando cambien los tópicos activos, actualiza el dropdown ---
window.addEventListener('hashchange', function () {
  if (window.location.hash.replace('#', '') === 'nodos_topicos') {
    actualizarMonitorDropdownAuto();
  }
});
// También lo puedes llamar al cargar la pestaña:
if (window.location.hash.replace('#', '') === 'nodos_topicos') {
  actualizarMonitorDropdownAuto();
}

  // ========================
// 6. Control Manual del Robot (TwistStamped, tópico y frecuencia configurables)
// ========================
const btnFwd = document.getElementById('btn-forward');
const btnBack = document.getElementById('btn-back');
const btnLeft = document.getElementById('btn-left');
const btnRight = document.getElementById('btn-right');
const btnStop = document.getElementById('btn-stop');

const sliderLinear = document.getElementById('slider-linear');
const sliderAngular = document.getElementById('slider-angular');
const inputLinear = document.getElementById('input-linear');
const inputAngular = document.getElementById('input-angular');
const controlStatus = document.getElementById('control-status');
const controlEnableToggle = document.getElementById('control-enable');

const topicSelect = document.getElementById('topic-select');
const publishRate = document.getElementById('publish-rate');
const currentTopic = document.getElementById('current-topic');
const pubStatus = document.getElementById('pub-status');

let cmdVelTopic = null;
let pubInterval = null;
let lastCmd = { linear: 0, angular: 0 };
let currentHz = Number(publishRate.value) || 10;

function manualControlActivo() {
  if (!ros.isConnected) return false;
  if (controlEnableToggle) {
    return controlEnableToggle.checked;
  }
  return true;
}

// --- Sincronizar slider y input (bidireccional) ---
function syncSliderAndInput(slider, input, min, max) {
  slider.addEventListener('input', () => {
    input.value = slider.value;
  });
  input.addEventListener('input', () => {
    let val = Number(input.value);
    if (isNaN(val)) val = 0;
    if (val < min) val = min;
    if (val > max) val = max;
    input.value = val.toFixed(2);
    slider.value = input.value;
  });
}
syncSliderAndInput(sliderLinear, inputLinear, -0.11, 0.11);
syncSliderAndInput(sliderAngular, inputAngular, -0.45, 0.45);

// --- Inicializa el publisher y su frecuencia ---
function crearCmdVelTopic() {
  if (cmdVelTopic) cmdVelTopic.unadvertise();
  cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: topicSelect.value,
    messageType: 'geometry_msgs/TwistStamped' // <- ¡Este es el tipo correcto!
  });
  if (currentTopic) currentTopic.textContent = `Publicando en: ${topicSelect.value}`;
}
crearCmdVelTopic();

// Cambiar tópico dinámicamente
topicSelect.addEventListener('change', () => {
  crearCmdVelTopic();
});

// Cambiar frecuencia de publicación
publishRate.addEventListener('change', () => {
  let hz = Number(publishRate.value);
  if (isNaN(hz) || hz < 1) hz = 1;
  if (hz > 30) hz = 30;
  publishRate.value = hz;
  currentHz = hz;
  reiniciarIntervalo();
});

// Función para publicar a la frecuencia indicada (Hz)
function reiniciarIntervalo() {
  if (pubInterval) clearInterval(pubInterval);
  pubInterval = setInterval(() => {
    if (!cmdVelTopic) return;
    if (!manualControlActivo()) {
      if (pubStatus) {
        pubStatus.textContent = ros.isConnected
          ? 'Control manual deshabilitado (interruptor apagado).'
          : 'Sin conexión.';
      }
      return;
    }
    publishTwistStamped(lastCmd.linear, lastCmd.angular);
    if (pubStatus) {
      pubStatus.textContent =
        `Enviando a "${topicSelect.value}" a ${currentHz} Hz. Lineal: ${lastCmd.linear}, Angular: ${lastCmd.angular}`;
    }
  }, 1000 / currentHz);
}
reiniciarIntervalo();

// Función para armar y publicar TwistStamped
function publishTwistStamped(linear, angular) {
  // header.stamp formato tipo ROS2 (segs, nanosegs)
  const now = Date.now();
  const msg = {
    header: {
      stamp: {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1e6
      },
      frame_id: "" // o "base_link" si tu controlador lo requiere
    },
    twist: {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    }
  };
  cmdVelTopic.publish(msg);
}

// Actualiza el estado de los controles
function actualizarEstadoControl() {
  const conectado = ros.isConnected;
  const manualArmado = controlEnableToggle ? controlEnableToggle.checked : true;
  const habilitado = conectado && manualArmado;

  [btnFwd, btnBack, btnLeft, btnRight, btnStop,
   sliderLinear, sliderAngular, inputLinear, inputAngular].forEach(elem => {
    if (elem) elem.disabled = !habilitado;
  });

  [topicSelect, publishRate].forEach(elem => {
    if (elem) elem.disabled = !conectado;
  });

  if (controlStatus) {
    if (!conectado) {
      controlStatus.textContent = 'Control deshabilitado. ROS no conectado.';
      controlStatus.style.color = '#c00';
    } else if (!manualArmado) {
      controlStatus.textContent = 'Control deshabilitado. Activa el interruptor para enviar comandos.';
      controlStatus.style.color = '#c60';
    } else {
      controlStatus.textContent = 'Control habilitado. ROS conectado.';
      controlStatus.style.color = '#080';
    }
  }
  if (pubStatus) {
    if (!conectado) {
      pubStatus.textContent = 'Sin conexión.';
    } else if (!manualArmado) {
      pubStatus.textContent = 'Control manual deshabilitado (interruptor apagado).';
    } else {
      pubStatus.textContent = `Enviando a "${topicSelect.value}" a ${currentHz} Hz.`;
    }
  }
  refreshLidarControls();
}

// Botones de movimiento: actualizan el último comando y lo envían una vez
function setTwist(linear, angular) {
  if (!manualControlActivo() || !cmdVelTopic) return;
  lastCmd = { linear, angular };
  publishTwistStamped(linear, angular); // Publica una vez de inmediato
  // Se seguirá publicando en el intervalo configurado
}

if (btnFwd) btnFwd.onclick = () => setTwist(Number(inputLinear.value), 0);
if (btnBack) btnBack.onclick = () => setTwist(-Number(inputLinear.value), 0);
if (btnLeft) btnLeft.onclick = () => setTwist(0, Number(inputAngular.value));
if (btnRight) btnRight.onclick = () => setTwist(0, -Number(inputAngular.value));
if (btnStop) btnStop.onclick = () => setTwist(0, 0);

if (controlEnableToggle) {
  controlEnableToggle.addEventListener('change', () => {
    if (!controlEnableToggle.checked && ros.isConnected && cmdVelTopic) {
      lastCmd = { linear: 0, angular: 0 };
      publishTwistStamped(0, 0);
    }
    actualizarEstadoControl();
  });
}

// Estado inicial
actualizarEstadoControl();

// Al conectar/desconectar ROS, actualiza controles
ros.on('connection', actualizarEstadoControl);
ros.on('close', actualizarEstadoControl);
ros.on('error', actualizarEstadoControl);

// Atajos de teclado: WASD y espacio
window.addEventListener('keydown', (e) => {
  if (!manualControlActivo() || !cmdVelTopic) return;
  if (e.repeat) return;
  if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA")) return;
  switch (e.key.toLowerCase()) {
    case 'w': setTwist(Number(inputLinear.value), 0); break;
    case 's': setTwist(-Number(inputLinear.value), 0); break;
    case 'a': setTwist(0, Number(inputAngular.value)); break;
    case 'd': setTwist(0, -Number(inputAngular.value)); break;
    case ' ': setTwist(0, 0); break;
  }
});
//Odometria Graficas con Historial

// ======================= ODOMETRÍA (solo activa cuando se muestra) =======================

// Parámetros y configuración
const RADIO_RUEDA = 0.029;
const VEL_MAX_MS = 0.11;
const VEL_MIN_MS = -0.11;
const nombresRuedas = [
  'joint_wheel_1',
  'joint_wheel_2',
  'joint_wheel_3',
  'joint_wheel_4'
];
navxState.acceleration.wheelNames = nombresRuedas.slice();
navxState.acceleration.wheelState = nombresRuedas.map(() => ({
  lastLinearVel: null,
  lastPos: null
}));
const chartsRuedas = [];
let tiempoInicioOdo = null;
let jointStatesListenerOdo = null;

// ======= NUEVO: Variables para historial de estadísticas =======
let historialGlobal = {
  valores: [], // Todos los valores registrados
  maxAbsoluto: 0,
  minAbsoluto: 0,
  promedioAcumulado: 0,
  totalMuestras: 0,
  tiempoInicio: null,
  ultimaActualizacion: null
};

let historialRuedas = [];
for (let i = 0; i < 4; i++) {
  historialRuedas.push({
    valores: [],
    maxAbsoluto: 0,
    minAbsoluto: 0,
    promedioAcumulado: 0,
    totalMuestras: 0,
    velocidadActual: 0
  });
}

// Elementos del DOM
const divStats = document.getElementById('odometria-stats');
if (divStats) divStats.innerHTML = "<b>Esperando datos...</b>";
const inputMaxVel = document.getElementById('input-max-vel');
const inputMinVel = document.getElementById('input-min-vel');
const toggleMaxLine = document.getElementById('toggle-maxline');
const toggleMinLine = document.getElementById('toggle-minline');
const toggleFixedScaleRuedas = document.getElementById('toggle-fixed-scale-ruedas');

// Paleta de colores para los valores
function colorValor(val) {
  if (Math.abs(val) > Math.abs(VEL_MAX_MS)) return '#d22';
  if (Math.abs(val) > 0.8 * Math.abs(VEL_MAX_MS)) return '#fa0';
  if (Math.abs(val) > 0.5 * Math.abs(VEL_MAX_MS)) return '#3a7';
  return '#08d';
}

// ======= NUEVO: Función para resetear historial =======
function resetearHistorial() {
  historialGlobal = {
    valores: [],
    maxAbsoluto: 0,
    minAbsoluto: 0,
    promedioAcumulado: 0,
    totalMuestras: 0,
    tiempoInicio: Date.now(),
    ultimaActualizacion: null
  };
  
  for (let i = 0; i < 4; i++) {
    historialRuedas[i] = {
      valores: [],
      maxAbsoluto: 0,
      minAbsoluto: 0,
      promedioAcumulado: 0,
      totalMuestras: 0,
      velocidadActual: 0
    };
  }
  
  if (divStats) {
    divStats.innerHTML = "<b>Historial reseteado. Esperando nuevos datos...</b>";
  }
}

// ======= NUEVO: Función para actualizar historial =======
function actualizarHistorial(valoresRuedas) {
  const ahora = Date.now();
  
  if (!historialGlobal.tiempoInicio) {
    historialGlobal.tiempoInicio = ahora;
  }
  historialGlobal.ultimaActualizacion = ahora;
  
  // Actualizar historial por rueda
  valoresRuedas.forEach((valor, idx) => {
    if (valor !== null && valor !== undefined) {
      const rueda = historialRuedas[idx];
      rueda.velocidadActual = valor;
      rueda.valores.push(valor);
      rueda.totalMuestras++;
      
      // Actualizar máximo y mínimo absolutos
      if (rueda.totalMuestras === 1) {
        rueda.maxAbsoluto = valor;
        rueda.minAbsoluto = valor;
        rueda.promedioAcumulado = valor;
      } else {
        rueda.maxAbsoluto = Math.max(rueda.maxAbsoluto, valor);
        rueda.minAbsoluto = Math.min(rueda.minAbsoluto, valor);
        rueda.promedioAcumulado = ((rueda.promedioAcumulado * (rueda.totalMuestras - 1)) + valor) / rueda.totalMuestras;
      }
      
      // Mantener solo los últimos 1000 valores para evitar uso excesivo de memoria
      if (rueda.valores.length > 1000) {
        rueda.valores.shift();
      }
    }
  });
  
  // Actualizar historial global
  const valoresValidos = valoresRuedas.filter(v => v !== null && v !== undefined);
  if (valoresValidos.length > 0) {
    historialGlobal.valores.push(...valoresValidos);
    historialGlobal.totalMuestras += valoresValidos.length;
    
    const maxActual = Math.max(...valoresValidos);
    const minActual = Math.min(...valoresValidos);
    const promedioActual = valoresValidos.reduce((a, b) => a + b, 0) / valoresValidos.length;
    
    if (historialGlobal.totalMuestras === valoresValidos.length) {
      historialGlobal.maxAbsoluto = maxActual;
      historialGlobal.minAbsoluto = minActual;
      historialGlobal.promedioAcumulado = promedioActual;
    } else {
      historialGlobal.maxAbsoluto = Math.max(historialGlobal.maxAbsoluto, maxActual);
      historialGlobal.minAbsoluto = Math.min(historialGlobal.minAbsoluto, minActual);
      const muestrasAnteriores = historialGlobal.totalMuestras - valoresValidos.length;
      historialGlobal.promedioAcumulado = ((historialGlobal.promedioAcumulado * muestrasAnteriores) + (promedioActual * valoresValidos.length)) / historialGlobal.totalMuestras;
    }
    
    // Mantener solo los últimos 4000 valores globales
    if (historialGlobal.valores.length > 4000) {
      historialGlobal.valores = historialGlobal.valores.slice(-4000);
    }
  }
}

// ======= NUEVO: Función para formatear tiempo transcurrido =======
function formatearTiempoTranscurrido(inicio, fin) {
  if (!inicio || !fin) return "N/A";
  
  const diff = Math.floor((fin - inicio) / 1000); // segundos
  
  if (diff < 60) return `${diff}s`;
  if (diff < 3600) return `${Math.floor(diff / 60)}m ${diff % 60}s`;
  
  const horas = Math.floor(diff / 3600);
  const minutos = Math.floor((diff % 3600) / 60);
  const segundos = diff % 60;
  
  return `${horas}h ${minutos}m ${segundos}s`;
}

// Inicializar gráficas (Chart.js)
for (let i = 1; i <= 4; i++) {
  const canvas = document.getElementById(`grafica-rueda-${i}`);
  if (canvas) {
    chartsRuedas.push(new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: `Rueda ${i} (m/s)`,
          data: [],
          borderWidth: 2,
          borderColor: '#2987d6',
          pointRadius: 1,
        }]
      },
      options: {
        responsive: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Tiempo (s)' } },
          y: {
            title: { display: true, text: 'Velocidad (m/s)' },
            min: VEL_MIN_MS * 1.2,
            max: VEL_MAX_MS * 1.2
          }
        },
        plugins: {
          legend: { display: false },
          annotation: {
            annotations: {
              maxVel: {
                type: 'line',
                yMin: VEL_MAX_MS,
                yMax: VEL_MAX_MS,
                borderColor: 'red',
                borderWidth: 2,
                borderDash: [4, 4],
                label: {
                  display: true, content: 'Máx', color: 'red', position: 'end',
                  font: { size: 11 }
                }
              },
              minVel: {
                type: 'line',
                yMin: VEL_MIN_MS,
                yMax: VEL_MIN_MS,
                borderColor: 'red',
                borderWidth: 2,
                borderDash: [4, 4],
                label: {
                  display: true, content: 'Mín', color: 'red', position: 'end',
                  font: { size: 11 }
                }
              }
            }
          }
        }
      }
    }));
  }
}

// Suscripción dinámica (MODIFICADA)
function activarGraficasOdo() {
  if (jointStatesListenerOdo) return;
  tiempoInicioOdo = null;
  
  // Inicializar historial si es la primera vez
  if (!historialGlobal.tiempoInicio) {
    historialGlobal.tiempoInicio = Date.now();
  }
  
  jointStatesListenerOdo = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
  });
  
  jointStatesListenerOdo.subscribe(function(msg) {
    if (!tiempoInicioOdo) tiempoInicioOdo = Date.now();
    const tiempo = ((Date.now() - tiempoInicioOdo) / 1000).toFixed(1);
    let allVals = [];
    let valoresRuedas = [null, null, null, null];
    let detalles = [];

    nombresRuedas.forEach((nombreRueda, idx) => {
      const index = msg.name.indexOf(nombreRueda);
      if (index >= 0 && msg.velocity && msg.velocity[index] !== undefined && chartsRuedas[idx]) {
        const valMS = msg.velocity[index] * RADIO_RUEDA;
        allVals.push(valMS);
        valoresRuedas[idx] = valMS;

        const chart = chartsRuedas[idx];
        chart.data.labels.push(tiempo);
        chart.data.datasets[0].data.push(valMS);
        if (chart.data.labels.length > 50) {
          chart.data.labels.shift();
          chart.data.datasets[0].data.shift();
        }
        chart.update();

        // ======= MODIFICADO: Estadísticas con historial =======
        const ruedaHistorial = historialRuedas[idx];
        const last = chart.data.datasets[0].data.slice(-20);
        const vmax = Math.max(...last).toFixed(3);
        const vmin = Math.min(...last).toFixed(3);
        const vavg = (last.reduce((a, b) => a + b, 0) / last.length).toFixed(3);

        detalles.push(
          `<div style="margin-bottom:0.8em; padding:0.3em; border:1px solid #ddd; border-radius:3px;">
            <b style="color:#2987d6;">Rueda ${idx+1}</b><br>
            <small><b>Actual (últimos 20):</b></small><br>
            <span style="color:${colorValor(vmax)}">Max:</span> ${vmax} | 
            <span style="color:${colorValor(vmin)}">Min:</span> ${vmin} | 
            <span style="color:${colorValor(vavg)}">Prom:</span> ${vavg}<br>
            <small><b>Historial total:</b></small><br>
            <span style="color:${colorValor(ruedaHistorial.maxAbsoluto)}">Máx:</span> ${ruedaHistorial.maxAbsoluto.toFixed(3)} | 
            <span style="color:${colorValor(ruedaHistorial.minAbsoluto)}">Mín:</span> ${ruedaHistorial.minAbsoluto.toFixed(3)} | 
            <span style="color:${colorValor(ruedaHistorial.promedioAcumulado)}">Prom:</span> ${ruedaHistorial.promedioAcumulado.toFixed(3)}<br>
            <small>Muestras: ${ruedaHistorial.totalMuestras}</small>
          </div>`
        );
      }
    });

    // ======= NUEVO: Actualizar historial =======
    if (allVals.length > 0) {
      actualizarHistorial(valoresRuedas);
    }

    // ======= MODIFICADO: Estadística global con historial =======
    if (divStats && allVals.length) {
      const max = Math.max(...allVals).toFixed(3);
      const min = Math.min(...allVals).toFixed(3);
      const avg = (allVals.reduce((a, b) => a + b, 0) / allVals.length).toFixed(3);
      
      const tiempoTotal = formatearTiempoTranscurrido(historialGlobal.tiempoInicio, historialGlobal.ultimaActualizacion);
      
      divStats.innerHTML = 
        `<div style="margin-bottom:1em; padding:0.5em; background:#f5f5f5; border-radius:5px;">
          <b>📊 Resumen Actual</b><br>
          <span style="color:${colorValor(max)};">Máx: ${max}</span> | 
          <span style="color:${colorValor(min)};">Mín: ${min}</span> | 
          <span style="color:${colorValor(avg)};">Promedio: ${avg}</span> m/s
        </div>
        
        <div style="margin-bottom:1em; padding:0.5em; background:#e8f4fd; border-radius:5px;">
          <b>📈 Historial Completo</b><br>
          <span style="color:${colorValor(historialGlobal.maxAbsoluto)};">Máx absoluto: ${historialGlobal.maxAbsoluto.toFixed(3)}</span><br>
          <span style="color:${colorValor(historialGlobal.minAbsoluto)};">Mín absoluto: ${historialGlobal.minAbsoluto.toFixed(3)}</span><br>
          <span style="color:${colorValor(historialGlobal.promedioAcumulado)};">Promedio total: ${historialGlobal.promedioAcumulado.toFixed(3)}</span><br>
          <small>Tiempo total: ${tiempoTotal} | Muestras: ${historialGlobal.totalMuestras}</small><br>
          <button id="btn-reset-historial" style="margin-top:0.3em; padding:0.2em 0.5em; background:#ff6b6b; color:white; border:none; border-radius:3px; cursor:pointer; font-size:11px;">
            🗑️ Reset Historial
          </button>
        </div>
        
        <hr style="margin:0.5em 0;">
        <b>📋 Detalle por Rueda</b><br>
        ${detalles.join('')}`;
    }
  });
}

function desactivarGraficasOdo() {
  if (jointStatesListenerOdo) {
    jointStatesListenerOdo.unsubscribe();
    jointStatesListenerOdo = null;
  }
  // NOTA: No reseteamos el historial al desactivar, se mantiene
}

// Límites de odometría (sin cambios)
function updateOdometryLimits() {
  const maxVel = parseFloat(inputMaxVel.value);
  const minVel = parseFloat(inputMinVel.value);
  const showMax = toggleMaxLine.checked;
  const showMin = toggleMinLine.checked;
  const fixedScale = toggleFixedScaleRuedas ? toggleFixedScaleRuedas.checked : false;
  for (const chart of chartsRuedas) {
    if (showMax) {
      chart.options.plugins.annotation.annotations.maxVel = {
        type: 'line', yMin: maxVel, yMax: maxVel, borderColor: 'red',
        borderWidth: 2, borderDash: [4, 4],
        label: { display: true, content: 'Máx', color: 'red', position: 'end', font: { size: 11 } }
      };
    } else {
      delete chart.options.plugins.annotation.annotations.maxVel;
    }
    if (showMin) {
      chart.options.plugins.annotation.annotations.minVel = {
        type: 'line', yMin: minVel, yMax: minVel, borderColor: 'red',
        borderWidth: 2, borderDash: [4, 4],
        label: { display: true, content: 'Mín', color: 'red', position: 'end', font: { size: 11 } }
      };
    } else {
      delete chart.options.plugins.annotation.annotations.minVel;
    }
    if (fixedScale) {
      chart.options.scales.y.max = maxVel * 1.2;
      chart.options.scales.y.min = minVel * 1.2;
    } else {
      chart.options.scales.y.max = showMax ? maxVel * 1.2 : undefined;
      chart.options.scales.y.min = showMin ? minVel * 1.2 : undefined;
    }
    chart.update();
  }
}

if (inputMaxVel) inputMaxVel.addEventListener('input', updateOdometryLimits);
if (inputMinVel) inputMinVel.addEventListener('input', updateOdometryLimits);
if (toggleMaxLine) toggleMaxLine.addEventListener('change', updateOdometryLimits);
if (toggleMinLine) toggleMinLine.addEventListener('change', updateOdometryLimits);
if (toggleFixedScaleRuedas) toggleFixedScaleRuedas.addEventListener('change', updateOdometryLimits);
updateOdometryLimits();

// ======= NUEVO: Event listener delegado para el botón de reset =======
if (divStats) {
  divStats.addEventListener('click', function(event) {
    if (event.target && event.target.id === 'btn-reset-historial') {
      resetearHistorial();
    }
  });
}

// ======= NUEVO: Hacer resetearHistorial disponible globalmente =======
window.resetearHistorial = resetearHistorial;
// --- ¡No agregues más lógica de botones aquí! ---
// El manejo de mostrar/ocultar ya está en tu bloque global de botones.
// Solo llama activarGraficasOdo() y desactivarGraficasOdo() desde allí.
// Ejemplo, tu bloque de botones debería verse así:

/*
botonesGraficas.forEach(boton => {
  boton.addEventListener('click', function () {
    // ...
    if (tipo === "odometro") {
      divGraficasOdometro.style.display = "flex";
      updateOdometryLimits();
      activarGraficasOdo();
    } else {
      desactivarGraficasOdo();
    }
    // ...
  });
});
*/


// ======================= ENCODERS (Modo incremental/modular) =======================

const nombresEncoders = [
  'joint_wheel_1',
  'joint_wheel_2',
  'joint_wheel_3',
  'joint_wheel_4'
];
const chartsEncoders = [];
let tiempoInicioEnc = null;

const divEncStats = document.getElementById('encoders-stats');
const spanEncStatHistN = document.getElementById('enc-stat-hist-n');
const spanEncStatMax = document.getElementById('enc-stat-max');
const spanEncStatMin = document.getElementById('enc-stat-min');
const spanEncStatAvg = document.getElementById('enc-stat-avg');
const divEncodersDetalles = document.getElementById('encoders-detalles');

const ENCODER_HISTORY_SIZE = 50;
const STATS_HISTORY_SIZE = 20;
const COLOR_GRAFICA_ENCODER = '#2db04c';
const COLOR_LINEA_LIMITE = '#d22';

const inputMaxEnc = document.getElementById('input-max-enc');
const inputMinEnc = document.getElementById('input-min-enc');
const toggleMaxLineEnc = document.getElementById('toggle-maxline-enc');
const toggleMinLineEnc = document.getElementById('toggle-minline-enc');
const VUELTA_ENCODER = 2 * Math.PI;
const TICKS_POR_VUELTA = 4096; // ajusta si la resolución del encoder cambia
let modoModular = false;
let encodersEnTicks = false;
const btnToggleModular = document.getElementById('toggle-modular-enc');
const btnResetEncTime = document.getElementById('reset-encoders-time');
const btnToggleUnidadEnc = document.getElementById('toggle-unidad-enc');
let encodersResetPendiente = false;

// Nueva: Guarda la instancia de la suscripción para poder activarla/desactivarla
let jointStatesListenerEnc = null;

if (spanEncStatHistN) spanEncStatHistN.textContent = STATS_HISTORY_SIZE;
if (divEncStats) divEncStats.innerHTML = "<b>Esperando datos...</b>";

// Inicializar gráficas (con Chart.js) para cada encoder
for (let i = 1; i <= 4; i++) {
  const canvas = document.getElementById(`grafica-encoder-${i}`);
  if (canvas) {
    chartsEncoders.push(new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: `Encoder ${i} (rad)`,
          data: [],
          borderWidth: 2,
          borderColor: COLOR_GRAFICA_ENCODER,
          pointRadius: 1,
        }]
      },
      options: {
        responsive: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Tiempo (s)' } },
          y: { title: { display: true, text: 'Posición (rad)' } }
        },
        plugins: {
          legend: { display: false },
          annotation: { annotations: {} }
        }
      }
    }));
  }
}

// Función para actualizar los límites de posición y las líneas en las gráficas
function updateEncoderLimits() {
  const maxPos = parseFloat(inputMaxEnc ? inputMaxEnc.value : '10');
  const minPos = parseFloat(inputMinEnc ? inputMinEnc.value : '-10');
  const showMax = toggleMaxLineEnc ? toggleMaxLineEnc.checked : true;
  const showMin = toggleMinLineEnc ? toggleMinLineEnc.checked : true;

  for (const chart of chartsEncoders) {
    chart.options.plugins.annotation.annotations = {};

    if (showMax) {
      chart.options.plugins.annotation.annotations.maxPos = {
        type: 'line',
        yMin: maxPos,
        yMax: maxPos,
        borderColor: COLOR_LINEA_LIMITE,
        borderWidth: 2,
        borderDash: [4, 4],
        label: {
          display: true,
          content: 'Máx',
          color: COLOR_LINEA_LIMITE,
          position: 'end',
          font: { size: 11 }
        }
      };
    }
    if (showMin) {
      chart.options.plugins.annotation.annotations.minPos = {
        type: 'line',
        yMin: minPos,
        yMax: minPos,
        borderColor: COLOR_LINEA_LIMITE,
        borderWidth: 2,
        borderDash: [4, 4],
        label: {
          display: true,
          content: 'Mín',
          color: COLOR_LINEA_LIMITE,
          position: 'end',
          font: { size: 11 }
        }
      };
    }

    chart.options.scales.y.max = showMax ? maxPos * 1.1 : undefined;
    chart.options.scales.y.min = showMin ? minPos * 1.1 : undefined;
    if (!showMax && !showMin) {
      chart.options.scales.y.max = undefined;
      chart.options.scales.y.min = undefined;
    }
    chart.update();
  }
}

// Listeners para los controles de límites
if (inputMaxEnc) inputMaxEnc.addEventListener('input', updateEncoderLimits);
if (inputMinEnc) inputMinEnc.addEventListener('input', updateEncoderLimits);
if (toggleMaxLineEnc) toggleMaxLineEnc.addEventListener('change', updateEncoderLimits);
if (toggleMinLineEnc) toggleMinLineEnc.addEventListener('change', updateEncoderLimits);
updateEncoderLimits();

// Alternancia modo modular/incremental
if (btnToggleModular) {
  btnToggleModular.addEventListener('click', function () {
    modoModular = !modoModular;
    btnToggleModular.textContent = modoModular ? 'Ver modo incremental' : 'Ver modo modular';
    resetearDatosEncodersParaUnidad();
  });
}

if (btnToggleUnidadEnc) {
  btnToggleUnidadEnc.addEventListener('click', function () {
    encodersEnTicks = !encodersEnTicks;
    btnToggleUnidadEnc.textContent = encodersEnTicks ? 'Ver en rad' : 'Ver en ticks';
    resetearDatosEncodersParaUnidad();
  });
}

function etiquetaUnidadEncoder() {
  return encodersEnTicks ? 'ticks' : 'rad';
}

function convertirEncoderUnidad(valorRad) {
  return encodersEnTicks ? valorRad * (TICKS_POR_VUELTA / VUELTA_ENCODER) : valorRad;
}

function resetearDatosEncodersParaUnidad() {
  const unidad = etiquetaUnidadEncoder();
  chartsEncoders.forEach((chart, idx) => {
    chart.data.labels = [];
    chart.data.datasets[0].data = [];
    chart.data.datasets[0].label = `Encoder ${idx + 1} (${unidad})`;
    if (chart.options?.scales?.y?.title) {
      chart.options.scales.y.title.text = encodersEnTicks ? 'Posición (ticks)' : 'Posición (rad)';
    }
    chart.update();
  });
  if (divEncStats) divEncStats.innerHTML = "<b>Esperando datos...</b>";
  if (divEncodersDetalles) divEncodersDetalles.innerHTML = '';
  if (spanEncStatMax) spanEncStatMax.textContent = '';
  if (spanEncStatMin) spanEncStatMin.textContent = '';
  if (spanEncStatAvg) spanEncStatAvg.textContent = '';
}

function resetearTiempoEncoders() {
  // Reinicia la base de tiempo y limpia datos/estadísticas mostradas.
  tiempoInicioEnc = null;
  encodersResetPendiente = true;
  resetearDatosEncodersParaUnidad();
}

if (btnResetEncTime) {
  btnResetEncTime.addEventListener('click', resetearTiempoEncoders);
}

// --------- SUSCRIPCIÓN Y DESUSCRIPCIÓN DINÁMICA ---------
function activarGraficasEncoders() {
  if (jointStatesListenerEnc) return; // Ya está activa
  tiempoInicioEnc = null; // Reset tiempo base al mostrar
  jointStatesListenerEnc = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
  });
  jointStatesListenerEnc.subscribe(function (msg) {
    const unidad = etiquetaUnidadEncoder();
    if (!tiempoInicioEnc) tiempoInicioEnc = Date.now();
    const tiempo = ((Date.now() - tiempoInicioEnc) / 1000).toFixed(1);

    if (encodersResetPendiente) {
      chartsEncoders.forEach(chart => {
        chart.data.labels.push(0);
        chart.data.datasets[0].data.push(0);
        if (chart.data.labels.length > ENCODER_HISTORY_SIZE) {
          chart.data.labels.shift();
          chart.data.datasets[0].data.shift();
        }
        chart.update();
      });
      encodersResetPendiente = false;
    }

    let allPositions = [];
    let detallesHTML = [];

    nombresEncoders.forEach((nombreEncoder, idx) => {
      const index = msg.name.indexOf(nombreEncoder);
      if (index >= 0 && msg.position && msg.position[index] !== undefined && chartsEncoders[idx]) {
        let valPos = msg.position[index];
        if (modoModular) {
          valPos = ((valPos % VUELTA_ENCODER) + VUELTA_ENCODER) % VUELTA_ENCODER;
        }
        const valorMostrar = convertirEncoderUnidad(valPos);
        allPositions.push(valorMostrar);

        const chart = chartsEncoders[idx];
        chart.data.labels.push(tiempo);
        chart.data.datasets[0].data.push(valorMostrar);

        if (chart.data.labels.length > ENCODER_HISTORY_SIZE) {
          chart.data.labels.shift();
          chart.data.datasets[0].data.shift();
        }
        chart.update();

        const last = chart.data.datasets[0].data.slice(-STATS_HISTORY_SIZE);
        const pmax = Math.max(...last).toFixed(3);
        const pmin = Math.min(...last).toFixed(3);
        const pavg = (last.reduce((a, b) => a + b, 0) / last.length).toFixed(3);

        detallesHTML.push(
          `<div style="margin-bottom:0.5em;">
            <b style="color:${COLOR_GRAFICA_ENCODER};">Encoder ${idx + 1}</b><br>
            <span>Máx:</span> ${pmax} ${unidad}<br>
            <span>Mín:</span> ${pmin} ${unidad}<br>
            <span>Prom:</span> ${pavg} ${unidad}
          </div>`
        );
      }
    });

    // Estadísticas globales
    if (divEncStats && allPositions.length) {
      const maxGlobal = Math.max(...allPositions).toFixed(3);
      const minGlobal = Math.min(...allPositions).toFixed(3);
      const avgGlobal = (allPositions.reduce((a, b) => a + b, 0) / allPositions.length).toFixed(3);

      if (spanEncStatMax) spanEncStatMax.textContent = maxGlobal;
      if (spanEncStatMin) spanEncStatMin.textContent = minGlobal;
      if (spanEncStatAvg) spanEncStatAvg.textContent = avgGlobal;
      if (divEncodersDetalles) divEncodersDetalles.innerHTML = detallesHTML.join('');
    }
  });
}

function desactivarGraficasEncoders() {
  if (jointStatesListenerEnc) {
    jointStatesListenerEnc.unsubscribe();
    jointStatesListenerEnc = null;
  }
}



// ============ PESTAÑA MONITOREO INDEPENDIENTE Y AISLADA =============

// 1. Variables DOM únicas para Monitoreo
const monitDropdownTopico = document.getElementById('monit-dropdown-topico');
const monitAddBtn = document.getElementById('monit-add-btn');
const monitListaTopicos = document.getElementById('monit-lista-topicos');
const monitFrecuenciaInput = document.getElementById('monit-frecuencia');

// 2. Diccionario de suscripciones activas exclusivo de Monitoreo
let monitSubs = {}; // { 'topic_name': {sub: ROSLIB.Topic, lastValue: ..., lastUpdate: timestamp} }

// 3. Poblar el dropdown con los tópicos activos de ROS SOLO en la pestaña monitoreo
function actualizarMonitDropdownAuto() {
  if (!monitDropdownTopico) return;
  monitDropdownTopico.innerHTML = '<option value="" disabled selected>Cargando tópicos...</option>';

  if (typeof ROSLIB === 'undefined' || !window.ros || !ros.isConnected) {
    monitDropdownTopico.innerHTML = '<option value="" disabled selected>No conectado a ROS</option>';
    return;
  }

  const srvTopicos = new ROSLIB.Service({
    ros: ros,
    name: '/rosapi/topics',
    serviceType: 'rosapi/GetTopics'
  });

  srvTopicos.callService(new ROSLIB.ServiceRequest({}), function(res) {
    const topics = res.topics || [];
    monitDropdownTopico.innerHTML = '<option value="" disabled selected>Selecciona tópico</option>';
    topics.forEach(t => {
      const opt = document.createElement('option');
      opt.value = t;
      opt.textContent = t;
      monitDropdownTopico.appendChild(opt);
    });
    if (!topics.length) {
      monitDropdownTopico.innerHTML = '<option value="" disabled selected>No hay tópicos activos</option>';
    }
  }, function() {
    monitDropdownTopico.innerHTML = '<option value="" disabled selected>Error consultando tópicos</option>';
  });
}

// 4. Agregar monitoreo en vivo a un tópico (frecuencia controlada)
function agregarMonitTopico(topico, tipoMsg) {
  if (!ros.isConnected) return;
  const sub = new ROSLIB.Topic({
    ros: ros,
    name: topico,
    messageType: tipoMsg || 'std_msgs/String',
    throttle_rate: 0 // lo controlamos manualmente
  });
  monitSubs[topico] = { sub, lastValue: null, tipoMsg, lastUpdate: 0 };

  const divFila = document.createElement('div');
  divFila.className = "monit-fila";
  divFila.style.cssText = "padding:0.6em 1em; margin-bottom:0.4em; border:1px solid #eee; border-radius:8px; background:#f8faff; font-family:monospace;";
  divFila.innerHTML = `
  <b style="color:#2987d6;">${topico}</b>
  <span style="color:#888; margin-left:1em;">(${tipoMsg})</span>
  <button class="monit-remove-btn" style="margin-left:1em;">✖</button>
  <div class="monit-terminal">
    <pre><span class="monit-valor"></span></pre>
  </div>
`;


  divFila.querySelector('.monit-remove-btn').onclick = function () {
    sub.unsubscribe();
    divFila.remove();
    delete monitSubs[topico];
  };
  monitListaTopicos.appendChild(divFila);

  sub.subscribe(function(msg) {
    // Controla frecuencia de actualización visual
    const freq = parseFloat(monitFrecuenciaInput?.value) || 3;
    const minInterval = 1000 / freq;
    const now = Date.now();
    if (!monitSubs[topico].lastUpdate || (now - monitSubs[topico].lastUpdate) >= minInterval) {
      monitSubs[topico].lastUpdate = now;
      monitSubs[topico].lastValue = msg;
      const span = divFila.querySelector('.monit-valor');
      if (span) {
        span.textContent = JSON.stringify(msg, null, 2);
      }
    }
    // Si no, simplemente no actualiza la interfaz (no hay flickering)
  });
}

// 5. Lógica del botón "Monitorear" solo para Monitoreo
if (monitAddBtn && monitDropdownTopico) {
  monitAddBtn.addEventListener('click', function () {
    const topico = monitDropdownTopico.value;
    if (!topico || monitSubs[topico]) return;
    const srvTipo = new ROSLIB.Service({
      ros: ros,
      name: '/rosapi/topic_type',
      serviceType: 'rosapi/TopicType'
    });
    srvTipo.callService(new ROSLIB.ServiceRequest({ topic: topico }), function (res) {
      agregarMonitTopico(topico, res.type || '');
    });
  });
}

// 6. Refresca tópicos solo cuando la pestaña #monitoreo está activa
window.addEventListener('hashchange', function () {
  if (window.location.hash.replace('#', '') === 'monitoreo') {
    setTimeout(actualizarMonitDropdownAuto, 30);
  }
});
if (window.location.hash.replace('#', '') === 'monitoreo') {
  setTimeout(actualizarMonitDropdownAuto, 30);
}

// --- Parámetros de Corriente (Motores) ---
const nombresMotores = ['joint_wheel_1', 'joint_wheel_2', 'joint_wheel_3', 'joint_wheel_4'];
const chartsCorrientes = [];
let offsetsCorriente = [0, 0, 0, 0]; // Offset ajustable por usuario

let tiempoInicioCorr = null;
let jointStatesListenerCorr = null;

let historialCorrGlobal = {
  valores: [],
  maxAbsoluto: 0,
  minAbsoluto: 0,
  promedioAcumulado: 0,
  totalMuestras: 0,
  tiempoInicio: null,
  ultimaActualizacion: null
};
let historialCorrMot = [];
for (let i = 0; i < 4; i++) {
  historialCorrMot.push({
    valores: [],
    maxAbsoluto: 0,
    minAbsoluto: 0,
    promedioAcumulado: 0,
    totalMuestras: 0,
    corrienteActual: 0
  });
}

// === Paleta de color para corriente ===
function colorCorr(val, max) {
  if (val > max) return '#d22';
  if (val > 0.8 * max) return '#fa0';
  if (val > 0.5 * max) return '#3a7';
  return '#08d';
}

// === Inicializar gráficas de corriente ===
for (let i = 1; i <= 4; i++) {
  const canvas = document.getElementById(`grafica-corriente-${i}`);
  if (canvas) {
    chartsCorrientes.push(new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: `Motor ${i} (A)`,
          data: [],
          borderWidth: 2,
          borderColor: '#f28c20',
          pointRadius: 1,
        }]
      },
      options: {
        responsive: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Tiempo (s)' } },
          y: {
            title: { display: true, text: 'Corriente (A)' },
            min: 0,
            max: 4.5
          }
        },
        plugins: {
          legend: { display: false },
          annotation: { annotations: {} }
        }
      }
    }));
  }
}

// === Actualización de límites ===
const inputMaxCorr = document.getElementById('input-max-corr');
const inputMinCorr = document.getElementById('input-min-corr');
const toggleMaxLineCorr = document.getElementById('toggle-maxline-corr');
const toggleMinLineCorr = document.getElementById('toggle-minline-corr');

function updateCorrienteLimits() {
  const maxCorr = parseFloat(inputMaxCorr ? inputMaxCorr.value : '4.5');
  const minCorr = parseFloat(inputMinCorr ? inputMinCorr.value : '0');
  const showMax = toggleMaxLineCorr ? toggleMaxLineCorr.checked : true;
  const showMin = toggleMinLineCorr ? toggleMinLineCorr.checked : true;
  for (const chart of chartsCorrientes) {
    chart.options.plugins.annotation.annotations = {};
    if (showMax) {
      chart.options.plugins.annotation.annotations.maxCorr = {
        type: 'line',
        yMin: maxCorr,
        yMax: maxCorr,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: {
          display: true,
          content: 'Máx',
          color: 'red',
          position: 'end',
          font: { size: 11 }
        }
      };
    }
    if (showMin) {
      chart.options.plugins.annotation.annotations.minCorr = {
        type: 'line',
        yMin: minCorr,
        yMax: minCorr,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: {
          display: true,
          content: 'Mín',
          color: 'red',
          position: 'end',
          font: { size: 11 }
        }
      };
    }
    chart.options.scales.y.max = showMax ? maxCorr * 1.2 : undefined;
    chart.options.scales.y.min = showMin ? minCorr * 0.8 : undefined;
    if (!showMax && !showMin) {
      chart.options.scales.y.max = undefined;
      chart.options.scales.y.min = undefined;
    }
    chart.update();
  }
}
if (inputMaxCorr) inputMaxCorr.addEventListener('input', updateCorrienteLimits);
if (inputMinCorr) inputMinCorr.addEventListener('input', updateCorrienteLimits);
if (toggleMaxLineCorr) toggleMaxLineCorr.addEventListener('change', updateCorrienteLimits);
if (toggleMinLineCorr) toggleMinLineCorr.addEventListener('change', updateCorrienteLimits);

// === Offset handlers ===
function actualizarOffsetCorriente(idx) {
  const inp = document.getElementById(`offset-corriente-${idx+1}`);
  if (inp) {
    offsetsCorriente[idx] = parseFloat(inp.value) || 0;
  }
}
window.actualizarOffsetCorriente = actualizarOffsetCorriente;

// === Subscripción dinámica de corriente ===
function activarGraficasCorriente() {
  if (jointStatesListenerCorr) return;
  tiempoInicioCorr = null;
  jointStatesListenerCorr = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
  });

  jointStatesListenerCorr.subscribe(function(msg) {
    if (!tiempoInicioCorr) tiempoInicioCorr = Date.now();
    const maxCorr = parseFloat(inputMaxCorr ? inputMaxCorr.value : '4.5');
    const tiempo = ((Date.now() - tiempoInicioCorr) / 1000).toFixed(1);
    let allVals = [];
    let valoresCorr = [null, null, null, null];
    let detalles = [];

    nombresMotores.forEach((nombre, idx) => {
      const i = msg.name.indexOf(nombre);
      if (i >= 0 && msg.effort && msg.effort[i] !== undefined && chartsCorrientes[idx]) {
        const valA = msg.effort[i] + (offsetsCorriente[idx] || 0);
        allVals.push(valA);
        valoresCorr[idx] = valA;

        const chart = chartsCorrientes[idx];
        chart.data.labels.push(tiempo);
        chart.data.datasets[0].data.push(valA);
        if (chart.data.labels.length > 50) {
          chart.data.labels.shift();
          chart.data.datasets[0].data.shift();
        }
        chart.update();

        // Estadísticas
        const motHistorial = historialCorrMot[idx];
        const last = chart.data.datasets[0].data.slice(-20);
        const vmax = Math.max(...last).toFixed(3);
        const vmin = Math.min(...last).toFixed(3);
        const vavg = (last.reduce((a, b) => a + b, 0) / last.length).toFixed(3);

        detalles.push(
          `<div style="margin-bottom:0.8em; padding:0.3em; border:1px solid #eee; border-radius:3px;">
            <b style="color:#f28c20;">Motor ${idx+1}</b><br>
            <small><b>Actual (últimos 20):</b></small><br>
            <span style="color:${colorCorr(vmax, maxCorr)}">Max:</span> ${vmax} | 
            <span style="color:${colorCorr(vmin, maxCorr)}">Min:</span> ${vmin} | 
            <span style="color:${colorCorr(vavg, maxCorr)}">Prom:</span> ${vavg}<br>
            <small><b>Historial total:</b></small><br>
            <span style="color:${colorCorr(motHistorial.maxAbsoluto, maxCorr)}">Máx:</span> ${motHistorial.maxAbsoluto.toFixed(3)} | 
            <span style="color:${colorCorr(motHistorial.minAbsoluto, maxCorr)}">Mín:</span> ${motHistorial.minAbsoluto.toFixed(3)} | 
            <span style="color:${colorCorr(motHistorial.promedioAcumulado, maxCorr)}">Prom:</span> ${motHistorial.promedioAcumulado.toFixed(3)}<br>
            <small>Muestras: ${motHistorial.totalMuestras}</small><br>
            <div>
              <label style="font-size:0.96em;">Offset: 
                <input type="number" id="offset-corriente-${idx+1}" value="${offsetsCorriente[idx] || 0}" step="0.01" style="width:55px;" onchange="actualizarOffsetCorriente(${idx})">
              </label>
            </div>
          </div>`
        );
      }
    });

    // Historial y estadísticas (igual que antes, omitido por espacio pero igual a tu patrón)
    // ...
    // Aquí puedes agregar tu lógica para el historial y para actualizar #corriente-stats como ya la tienes.

    // Estadísticas en HTML
    const divStatsCorr = document.getElementById('corriente-stats');
    if (divStatsCorr && allVals.length) {
      const max = Math.max(...allVals).toFixed(3);
      const min = Math.min(...allVals).toFixed(3);
      const avg = (allVals.reduce((a, b) => a + b, 0) / allVals.length).toFixed(3);

      divStatsCorr.innerHTML =
        `<div style="margin-bottom:1em; padding:0.5em; background:#fff5e0; border-radius:5px;">
          <b>📊 Resumen Actual</b><br>
          <span style="color:${colorCorr(max, maxCorr)};">Máx: ${max}</span> | 
          <span style="color:${colorCorr(min, maxCorr)};">Mín: ${min}</span> | 
          <span style="color:${colorCorr(avg, maxCorr)};">Promedio: ${avg}</span> A
        </div>
        <hr style="margin:0.5em 0;">
        <b>📋 Detalle por Motor</b><br>
        ${detalles.join('')}`;
    }
  });
}

function desactivarGraficasCorriente() {
  if (jointStatesListenerCorr) {
    jointStatesListenerCorr.unsubscribe();
    jointStatesListenerCorr = null;
  }
}

// --- Parámetros y gráficas de Batería ---
const bateriaConfigs = [
  { key: '5v', canvasId: 'grafica-bateria-5v', topic: '/voltaje/v5', label: '5V (Arduino / ESP32)' },
  { key: '14v', canvasId: 'grafica-bateria-14v', topic: '/voltaje/v14', label: '14V (Batería)' }
];
const bateriaCharts = {};
const bateriaListeners = {};
const bateriaHistoryLimit = 120;
let bateriaT0 = null;
const bateria5vValor = document.getElementById('bateria-5v-valor');
const bateria14vValor = document.getElementById('bateria-14v-valor');

function initChartsBateria() {
  bateriaConfigs.forEach(cfg => {
    if (bateriaCharts[cfg.key]) return;
    const canvas = document.getElementById(cfg.canvasId);
    if (!canvas) return;
    bateriaCharts[cfg.key] = new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: cfg.label,
          data: [],
          borderWidth: 2,
          borderColor: cfg.key === '5v' ? '#16a34a' : '#dc2626',
          pointRadius: 1
        }]
      },
      options: {
        responsive: false,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Tiempo (s)' } },
          y: {
            title: { display: true, text: 'Voltaje (V)' }
          }
        },
        plugins: { legend: { display: true } }
      }
    });
  });
}

function updateBateriaLabel(cfgKey, valor) {
  const target = cfgKey === '5v' ? bateria5vValor : cfgKey === '14v' ? bateria14vValor : null;
  if (!target) return;
  target.textContent = (cfgKey === '5v' ? '5V:' : '14V:') + ` ${Number.isFinite(valor) ? valor.toFixed(2) : '—'}`;
}

function activarGraficasBateria() {
  if (!ros) return;
  initChartsBateria();
  if (!bateriaT0) bateriaT0 = Date.now();

  bateriaConfigs.forEach(cfg => {
    if (bateriaListeners[cfg.key]) return;
    const chart = bateriaCharts[cfg.key];
    if (!chart) return;
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: cfg.topic,
      messageType: 'std_msgs/Float32'
    });
    bateriaListeners[cfg.key] = topic;
    topic.subscribe(message => {
      const valor = Number(message?.data);
      const t = ((Date.now() - bateriaT0) / 1000).toFixed(1);
      chart.data.labels.push(t);
      chart.data.datasets[0].data.push(valor);
      if (chart.data.labels.length > bateriaHistoryLimit) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
      }
      chart.update();
      updateBateriaLabel(cfg.key, valor);
    });
  });
}

function desactivarGraficasBateria() {
  bateriaConfigs.forEach(cfg => {
    if (bateriaListeners[cfg.key]) {
      bateriaListeners[cfg.key].unsubscribe();
      delete bateriaListeners[cfg.key];
    }
  });
}

// --- Charts y parámetros de MPU ---
let chartAcel, chartGyro, chartOrient;
let mpuListener = null;
let mpuT0 = null;

// === Inicializar gráficas de MPU (idéntico a otros paneles) ===
function initChartsMPU() {
  if (chartAcel && chartGyro && chartOrient) return;
  // Gráfica aceleración
  chartAcel = new Chart(document.getElementById('grafica-aceleracion').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'X', data: [], borderColor: '#08d', borderWidth: 2, pointRadius: 1 },
        { label: 'Y', data: [], borderColor: '#fa0', borderWidth: 2, pointRadius: 1 },
        { label: 'Z', data: [], borderColor: '#d22', borderWidth: 2, pointRadius: 1 }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tiempo (s)' } },
        y: { title: { display: true, text: 'm/s²' }, min: -20, max: 20 }
      },
      plugins: {
        legend: { display: true },
        annotation: { annotations: {} }
      }
    }
  });
  // Gráfica giroscopio
  chartGyro = new Chart(document.getElementById('grafica-giroscopio').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'X', data: [], borderColor: '#08d', borderWidth: 2, pointRadius: 1 },
        { label: 'Y', data: [], borderColor: '#fa0', borderWidth: 2, pointRadius: 1 },
        { label: 'Z', data: [], borderColor: '#d22', borderWidth: 2, pointRadius: 1 }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tiempo (s)' } },
        y: { title: { display: true, text: 'rad/s' }, min: -10, max: 10 }
      },
      plugins: {
        legend: { display: true },
        annotation: { annotations: {} }
      }
    }
  });
  // Gráfica orientación (roll, pitch, yaw)
  chartOrient = new Chart(document.getElementById('grafica-orientacion').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Roll', data: [], borderColor: '#08d', borderWidth: 2, pointRadius: 1 },
        { label: 'Pitch', data: [], borderColor: '#fa0', borderWidth: 2, pointRadius: 1 },
        { label: 'Yaw', data: [], borderColor: '#d22', borderWidth: 2, pointRadius: 1 }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tiempo (s)' } },
        y: { title: { display: true, text: 'Grados (°)' }, min: -180, max: 180 }
      },
      plugins: {
        legend: { display: true }
      }
    }
  });
}

// === Actualización de límites ===
const inputMaxAcel = document.getElementById('input-max-acel');
const inputMinAcel = document.getElementById('input-min-acel');
const toggleMaxLineAcel = document.getElementById('toggle-maxline-acel');
const toggleMinLineAcel = document.getElementById('toggle-minline-acel');

const inputMaxGyro = document.getElementById('input-max-gyro');
const inputMinGyro = document.getElementById('input-min-gyro');
const toggleMaxLineGyro = document.getElementById('toggle-maxline-gyro');
const toggleMinLineGyro = document.getElementById('toggle-minline-gyro');

function updateLimitsMPU() {
  // Aceleración
  const maxA = parseFloat(inputMaxAcel?.value ?? "20");
  const minA = parseFloat(inputMinAcel?.value ?? "-20");
  const showMaxA = toggleMaxLineAcel?.checked ?? true;
  const showMinA = toggleMinLineAcel?.checked ?? true;
  if (chartAcel) {
    chartAcel.options.plugins.annotation.annotations = {};
    if (showMaxA) {
      chartAcel.options.plugins.annotation.annotations.maxAcel = {
        type: 'line',
        yMin: maxA,
        yMax: maxA,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: { display: true, content: 'Máx', color: 'red', position: 'end', font: { size: 11 } }
      };
    }
    if (showMinA) {
      chartAcel.options.plugins.annotation.annotations.minAcel = {
        type: 'line',
        yMin: minA,
        yMax: minA,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: { display: true, content: 'Mín', color: 'red', position: 'end', font: { size: 11 } }
      };
    }
    chartAcel.options.scales.y.max = showMaxA ? maxA * 1.1 : undefined;
    chartAcel.options.scales.y.min = showMinA ? minA * 1.1 : undefined;
    if (!showMaxA && !showMinA) {
      chartAcel.options.scales.y.max = undefined;
      chartAcel.options.scales.y.min = undefined;
    }
    chartAcel.update();
  }
  // Giroscopio
  const maxG = parseFloat(inputMaxGyro?.value ?? "10");
  const minG = parseFloat(inputMinGyro?.value ?? "-10");
  const showMaxG = toggleMaxLineGyro?.checked ?? true;
  const showMinG = toggleMinLineGyro?.checked ?? true;
  if (chartGyro) {
    chartGyro.options.plugins.annotation.annotations = {};
    if (showMaxG) {
      chartGyro.options.plugins.annotation.annotations.maxGyro = {
        type: 'line',
        yMin: maxG,
        yMax: maxG,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: { display: true, content: 'Máx', color: 'red', position: 'end', font: { size: 11 } }
      };
    }
    if (showMinG) {
      chartGyro.options.plugins.annotation.annotations.minGyro = {
        type: 'line',
        yMin: minG,
        yMax: minG,
        borderColor: 'red',
        borderWidth: 2,
        borderDash: [4, 4],
        label: { display: true, content: 'Mín', color: 'red', position: 'end', font: { size: 11 } }
      };
    }
    chartGyro.options.scales.y.max = showMaxG ? maxG * 1.1 : undefined;
    chartGyro.options.scales.y.min = showMinG ? minG * 1.1 : undefined;
    if (!showMaxG && !showMinG) {
      chartGyro.options.scales.y.max = undefined;
      chartGyro.options.scales.y.min = undefined;
    }
    chartGyro.update();
  }
}

// === Listeners (igual que el patrón de corriente) ===
if (inputMaxAcel) inputMaxAcel.addEventListener('input', updateLimitsMPU);
if (inputMinAcel) inputMinAcel.addEventListener('input', updateLimitsMPU);
if (toggleMaxLineAcel) toggleMaxLineAcel.addEventListener('change', updateLimitsMPU);
if (toggleMinLineAcel) toggleMinLineAcel.addEventListener('change', updateLimitsMPU);

if (inputMaxGyro) inputMaxGyro.addEventListener('input', updateLimitsMPU);
if (inputMinGyro) inputMinGyro.addEventListener('input', updateLimitsMPU);
if (toggleMaxLineGyro) toggleMaxLineGyro.addEventListener('change', updateLimitsMPU);
if (toggleMinLineGyro) toggleMinLineGyro.addEventListener('change', updateLimitsMPU);

// === Subscripción dinámica de la IMU ===
function activarGraficasMPU() {
  if (mpuListener) return;
  initChartsMPU();
  updateLimitsMPU();
  mpuT0 = null;
  mpuListener = new ROSLIB.Topic({
    ros: ros,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu'
  });
  mpuListener.subscribe(function(msg) {
    if (!mpuT0) mpuT0 = Date.now();
    const t = ((Date.now() - mpuT0) / 1000).toFixed(1);

    // Aceleración
    chartAcel.data.labels.push(t);
    chartAcel.data.datasets[0].data.push(msg.linear_acceleration.x);
    chartAcel.data.datasets[1].data.push(msg.linear_acceleration.y);
    chartAcel.data.datasets[2].data.push(msg.linear_acceleration.z);
    if (chartAcel.data.labels.length > 50) {
      chartAcel.data.labels.shift();
      chartAcel.data.datasets.forEach(ds=>ds.data.shift());
    }
    chartAcel.update();

    // Giroscopio
    chartGyro.data.labels.push(t);
    chartGyro.data.datasets[0].data.push(msg.angular_velocity.x);
    chartGyro.data.datasets[1].data.push(msg.angular_velocity.y);
    chartGyro.data.datasets[2].data.push(msg.angular_velocity.z);
    if (chartGyro.data.labels.length > 50) {
      chartGyro.data.labels.shift();
      chartGyro.data.datasets.forEach(ds=>ds.data.shift());
    }
    chartGyro.update();

    // Orientación (quaternion → euler)
    const q = msg.orientation;
    const euler = quaternionToEuler(q.x, q.y, q.z, q.w);
    chartOrient.data.labels.push(t);
    chartOrient.data.datasets[0].data.push(euler[0]);
    chartOrient.data.datasets[1].data.push(euler[1]);
    chartOrient.data.datasets[2].data.push(euler[2]);
    if (chartOrient.data.labels.length > 50) {
      chartOrient.data.labels.shift();
      chartOrient.data.datasets.forEach(ds=>ds.data.shift());
    }
    chartOrient.update();

    // Estado actual
    const divStats = document.getElementById('mpu-stats');
    if (divStats) {
      divStats.innerHTML = `
        <div style="margin-top:1em; padding:0.7em; background:#f8faff; border-radius:7px;">
          <b>Estado Actual IMU</b><br>
          <span style="color:#08d;">Aceleración:</span>
          x=${msg.linear_acceleration.x.toFixed(2)} y=${msg.linear_acceleration.y.toFixed(2)} z=${msg.linear_acceleration.z.toFixed(2)}<br>
          <span style="color:#fa0;">Vel. Angular:</span>
          x=${msg.angular_velocity.x.toFixed(3)} y=${msg.angular_velocity.y.toFixed(3)} z=${msg.angular_velocity.z.toFixed(3)}<br>
          <span style="color:#d22;">Orientación:</span>
          Roll=${euler[0].toFixed(1)}° Pitch=${euler[1].toFixed(1)}° Yaw=${euler[2].toFixed(1)}°
        </div>
      `;
    }
  });
}

function desactivarGraficasMPU() {
  if (mpuListener) {
    mpuListener.unsubscribe();
    mpuListener = null;
  }
}

// --- Quaternion a Euler ---
function quaternionToEuler(x, y, z, w) {
  let ysqr = y * y;
  let t0 = +2.0 * (w * x + y * z);
  let t1 = +1.0 - 2.0 * (x * x + ysqr);
  let roll = Math.atan2(t0, t1);
  let t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  let pitch = Math.asin(t2);
  let t3 = +2.0 * (w * z + x * y);
  let t4 = +1.0 - 2.0 * (ysqr + z * z);
  let yaw = Math.atan2(t3, t4);
  return [roll * 180/Math.PI, pitch * 180/Math.PI, yaw * 180/Math.PI];
}



// --- MANEJO DE BOTONES DE GRÁFICAS, VERSIÓN FINAL ES GLOBAL SE USA PARA TODOS LOS CONJUNTOS DE GRÁFICAS.---
const botonesGraficas = document.querySelectorAll('.boton-grafica');
const divGraficasOdometro = document.getElementById('graficas-odometro');
const divGraficasEncoders = document.getElementById('graficas-encoders');
const divGraficasMotores = document.getElementById('graficas-motores');
const divGraficasBateria = document.getElementById('graficas-bateria');
const divGraficasUnicas = document.getElementById('graficas-unicas');
const divGraficasMPU = document.getElementById('graficas-mpu');  // << NUEVO

function updateNavegacionMonitoring() {
  if (!navxAsideButtons || navxAsideButtons.length === 0) {
    deactivateNavSalidas();
    return;
  }
  const salidasSelected = Array.from(navxAsideButtons).some(btn =>
    btn.dataset.target === 'salidas' && btn.classList.contains('active')
  );
  if (activeTab === 'navegacion' && salidasSelected) {
    activateNavSalidas();
  } else {
    deactivateNavSalidas();
  }
  updateNavPanelSubscriptions();
}

function activateNavSalidas() {
  if (navxState.salidasActive) return;
  navxState.salidasActive = true;
  updateNavPanelSubscriptions();
}

function deactivateNavSalidas() {
  if (!navxState.salidasActive) return;
  navxState.salidasActive = false;
  updateNavPanelSubscriptions();
}

function updateNavPanelSubscriptions() {
  if (!navxState.salidasActive) {
    stopNavVelocityMonitoring();
    stopNavFrequencyMonitoring();
    stopNavAccelerationMonitoring();
    return;
  }

  const panel = navxState.activePanel;

  if (panel === 'velocidades') {
    startNavVelocityMonitoring();
    stopNavFrequencyMonitoring();
    stopNavAccelerationMonitoring();
  } else if (panel === 'frecuencia') {
    stopNavVelocityMonitoring();
    startNavFrequencyMonitoring();
    stopNavAccelerationMonitoring();
  } else if (panel === 'aceleraciones') {
    stopNavVelocityMonitoring();
    stopNavFrequencyMonitoring();
    startNavAccelerationMonitoring();
    navxApplyAccelerationVisibility();
  } else {
    stopNavVelocityMonitoring();
    stopNavFrequencyMonitoring();
    stopNavAccelerationMonitoring();
  }
}

function startNavVelocityMonitoring() {
  if (!ros || !ros.isConnected || navxState.activePanel !== 'velocidades') return;
  NAVX_VEL_TOPIC_CONFIGS.forEach(config => {
    const chart = ensureNavVelocityChart(config);
    if (!chart) return;
    resolveNavVelocityType(config, resolvedType => {
      if (!navxState.salidasActive || navxState.activePanel !== 'velocidades') return;
      const normalized = navxNormalizeTwistType(resolvedType);
      if (navxState.velocity.topics[config.key] && navxState.velocity.topicTypes[config.key] === normalized) {
        return;
      }
      subscribeNavVelocityTopic(config, normalized);
    });
  });
}

function stopNavVelocityMonitoring() {
  Object.values(navxState.velocity.topics).forEach(topic => {
    if (topic) topic.unsubscribe();
  });
  navxState.velocity.topics = {};
  navxState.velocity.startTimes = {};
  Object.values(navxState.velocity.charts).forEach(chart => navxClearChart(chart));
}

function subscribeNavVelocityTopic(config, rawType) {
  if (!ros || !navxState.salidasActive) return;
  if (navxState.velocity.topics[config.key]) {
    navxState.velocity.topics[config.key].unsubscribe();
  }
  const resolvedType = navxNormalizeTwistType(rawType);
  const topic = new ROSLIB.Topic({
    ros,
    name: config.topic,
    messageType: resolvedType
  });
  topic.subscribe(msg => handleNavVelocityMessage(config, msg));
  navxState.velocity.topics[config.key] = topic;
  navxState.velocity.topicTypes[config.key] = resolvedType;
  navxState.velocity.startTimes[config.key] = null;
}

function navxNormalizeTwistType(type) {
  if (typeof type !== 'string') {
    return 'geometry_msgs/msg/Twist';
  }
  const lower = type.trim().toLowerCase();
  if (lower.includes('twiststamped')) {
    return 'geometry_msgs/msg/TwistStamped';
  }
  if (lower.includes('twist')) {
    return 'geometry_msgs/msg/Twist';
  }
  return 'geometry_msgs/msg/Twist';
}

function navxSetVelocityMode(mode) {
  const normalizedMode = mode === 'angular' ? 'angular' : 'linear';
  navxState.velocity.mode = normalizedMode;
  navxUpdateVelocityModeButtons();
  navxApplyVelocityModeToCharts();
}

function navxUpdateVelocityModeButtons() {
  if (navxVelModeLinearBtn) {
    navxVelModeLinearBtn.classList.toggle('active', navxState.velocity.mode === 'linear');
  }
  if (navxVelModeAngularBtn) {
    navxVelModeAngularBtn.classList.toggle('active', navxState.velocity.mode === 'angular');
  }
}

function navxApplyVelocityModeToCharts() {
  Object.values(navxState.velocity.charts).forEach(chart => {
    navxApplyVelocityModeToChart(chart, true);
    chart.update();
  });
}

function navxApplyVelocityModeToChart(chart, suppressUpdate = false) {
  if (!chart) return;
  const isLinearMode = navxState.velocity.mode === 'linear';
  chart.data.datasets.forEach((dataset, idx) => {
    const isLinearDataset = idx === 0;
    dataset.hidden = isLinearMode ? !isLinearDataset : isLinearDataset;
  });
  if (!suppressUpdate) {
    chart.update();
  }
}

function navxApplyFrequencyMode() {
  if (navxFreqToggleBtn) {
    navxFreqToggleBtn.textContent = navxState.frequency.mode === 'delta'
      ? 'Mostrar en Hz'
      : 'Mostrar en segundos';
  }
  const mode = navxState.frequency.mode;
  const chart = navxState.frequency.chart;
  if (!chart) return;
  chart.data.datasets[0].label = mode === 'delta' ? 'Δt (s)' : 'Frecuencia (Hz)';
  chart.data.datasets[1].label = mode === 'delta' ? 'Discontinuidad' : 'Caída de Hz';
  chart.options.scales.y.title.text = mode === 'delta' ? 'Δt (s)' : 'Frecuencia (Hz)';
  navxRenderFrequencyChart();
}

function navxConvertFrequencyValue(mode, dt) {
  if (!Number.isFinite(dt) || dt <= 0) return null;
  return mode === 'hz' ? 1 / dt : dt;
}

function navxRenderFrequencyChart() {
  const chart = navxState.frequency.chart;
  if (!chart) return;
  const mode = navxState.frequency.mode;
  const history = navxState.frequency.history;
  chart.data.labels = history.map(entry => entry.x);
  chart.data.datasets[0].data = history.map(entry => ({
    x: entry.x,
    y: navxConvertFrequencyValue(mode, entry.dt)
  }));
  chart.data.datasets[1].data = history.map(entry => ({
    x: entry.x,
    y: entry.jump ? navxConvertFrequencyValue(mode, entry.dt) : null
  }));
  chart.update();
}

function navxApplyAccelerationVisibility() {
  const enabled = navxState.acceleration.highWheelsEnabled;
  if (navxAccelToggleHighBtn) {
    navxAccelToggleHighBtn.textContent = enabled ? 'Ocultar ruedas 3 y 4' : 'Mostrar ruedas 3 y 4';
  }
  const card3 = document.getElementById('nav-accel-card-3');
  const card4 = document.getElementById('nav-accel-card-4');
  if (card3) card3.style.display = enabled ? '' : 'none';
  if (card4) card4.style.display = enabled ? '' : 'none';
  if (!enabled) {
    [2, 3].forEach(idx => {
      const chart = navxState.acceleration.charts[idx];
      if (chart) navxClearChart(chart);
    });
  } else if (navxState.salidasActive && navxState.activePanel === 'aceleraciones') {
    navxState.acceleration.wheelNames.forEach((_, idx) => {
      if (idx >= 2) ensureNavAccelerationChart(idx);
    });
  }
}

function resolveNavVelocityType(config, callback) {
  const hintType = navxNormalizeTwistType(config.typeHint);
  if (!ros) {
    callback(hintType);
    return;
  }

  const cached = navxState.velocity.topicTypes[config.key];
  if (cached) {
    callback(cached);
    return;
  }

  if (typeof ros.getTopics === 'function') {
    ros.getTopics(result => {
      const topics = Array.isArray(result?.topics) ? result.topics : [];
      const types = Array.isArray(result?.types) ? result.types : [];
      const idx = topics.indexOf(config.topic);
      if (idx >= 0 && types[idx]) {
        callback(navxNormalizeTwistType(types[idx]));
      } else {
        resolveNavVelocityTypeByLookup(config, hintType, callback);
      }
    }, () => resolveNavVelocityTypeByLookup(config, hintType, callback));
    return;
  }

  resolveNavVelocityTypeByLookup(config, hintType, callback);
}

function resolveNavVelocityTypeByLookup(config, fallbackType, callback) {
  if (!ros || typeof ros.getTopicsForType !== 'function') {
    callback(fallbackType);
    return;
  }

  const preferred = navxNormalizeTwistType(fallbackType);
  const alternatives = preferred === 'geometry_msgs/msg/TwistStamped'
    ? ['geometry_msgs/msg/TwistStamped', 'geometry_msgs/msg/Twist']
    : ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'];

  const uniqueTypes = Array.from(new Set(alternatives));

  const tryResolve = index => {
    if (index >= uniqueTypes.length) {
      callback(fallbackType);
      return;
    }
    const type = uniqueTypes[index];
    ros.getTopicsForType(type, topics => {
      const list = Array.isArray(topics?.topics) ? topics.topics : Array.isArray(topics) ? topics : [];
      if (list.includes(config.topic)) {
        callback(type);
      } else {
        tryResolve(index + 1);
      }
    }, () => tryResolve(index + 1));
  };

  tryResolve(0);
}

function ensureNavVelocityChart(config) {
  if (navxState.velocity.charts[config.key]) {
    return navxState.velocity.charts[config.key];
  }
  const canvas = document.getElementById(config.canvasId);
  if (!canvas || !window.Chart) return null;
  const chart = new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Lineal X (m/s)',
          data: [],
          borderColor: config.colorLinear,
          borderWidth: 2,
          pointRadius: 1,
          tension: 0.1,
          yAxisID: 'linear'
        },
        {
          label: 'Angular Z (rad/s)',
          data: [],
          borderColor: config.colorAngular,
          borderWidth: 2,
          borderDash: [6, 2],
          pointRadius: 1,
          tension: 0.1,
          yAxisID: 'angular'
        }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      maintainAspectRatio: false,
      scales: {
        x: {
          title: { display: true, text: 'Tiempo (s)' }
        },
        linear: {
          position: 'left',
          title: { display: true, text: 'Lineal (m/s)' }
        },
        angular: {
          position: 'right',
          title: { display: true, text: 'Angular (rad/s)' },
          grid: { drawOnChartArea: false }
        }
      },
      plugins: {
        legend: { position: 'bottom' }
      }
    }
  });
  navxState.velocity.charts[config.key] = chart;
  navxApplyVelocityModeToChart(chart, true);
  chart.update();
  return chart;
}

function handleNavVelocityMessage(config, msg) {
  const chart = ensureNavVelocityChart(config);
  if (!chart) return;
  const now = Date.now();
  if (!navxState.velocity.startTimes[config.key]) {
    navxState.velocity.startTimes[config.key] = now;
  }
  const elapsed = (now - navxState.velocity.startTimes[config.key]) / 1000;
  const twist = navxExtractTwistFromMessage(navxState.velocity.topicTypes[config.key], msg);
  const linearX = twist && twist.linear && Number.isFinite(twist.linear.x) ? twist.linear.x : null;
  const angularZ = twist && twist.angular && Number.isFinite(twist.angular.z) ? twist.angular.z : null;
  navxAppendData(chart, Number(elapsed.toFixed(2)), [linearX, angularZ], NAVX_MAX_POINTS);
}

function startNavFrequencyMonitoring() {
  if (!ros || !ros.isConnected || !navxState.salidasActive || navxState.activePanel !== 'frecuencia') return;
  const chart = ensureNavFrequencyChart();
  if (!chart || navxState.frequency.topic) return;
  navxState.frequency.topic = new ROSLIB.Topic({
    ros,
    name: '/odometry/filtered',
    messageType: 'nav_msgs/Odometry'
  });
  navxState.frequency.topic.subscribe(handleNavFrequencyMessage);
}

function stopNavFrequencyMonitoring() {
  if (navxState.frequency.topic) {
    navxState.frequency.topic.unsubscribe();
  }
  navxState.frequency.topic = null;
  navxState.frequency.startStamp = null;
  navxState.frequency.lastStamp = null;
  navxState.frequency.history = [];
  navxClearChart(navxState.frequency.chart);
}

function ensureNavFrequencyChart() {
  if (navxState.frequency.chart) return navxState.frequency.chart;
  const canvas = document.getElementById('nav-chart-odom-timestamp');
  if (!canvas || !window.Chart) return null;
  navxState.frequency.chart = new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Δt (s)',
          data: [],
          borderColor: '#0ea5e9',
          borderWidth: 2,
          pointRadius: 1.5,
          tension: 0.1,
          fill: false
        },
        {
          label: 'Discontinuidad',
          data: [],
          borderColor: '#ef4444',
          backgroundColor: '#ef4444',
          pointRadius: 4,
          showLine: false
        }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tiempo (s)' } },
        y: {
          title: { display: true, text: 'Δt (s)' },
          beginAtZero: true
        }
      },
      plugins: {
        legend: { position: 'bottom' }
      }
    }
  });
  navxApplyFrequencyMode();
  return navxState.frequency.chart;
}

function handleNavFrequencyMessage(msg) {
  const chart = ensureNavFrequencyChart();
  if (!chart) return;
  const stampSec = navxStampToSeconds(msg && msg.header ? msg.header.stamp : null);
  if (!Number.isFinite(stampSec)) return;
  if (navxState.frequency.startStamp === null) {
    navxState.frequency.startStamp = stampSec;
  }
  const dtRaw = navxState.frequency.lastStamp !== null ? stampSec - navxState.frequency.lastStamp : 0;
  const dt = Number.isFinite(dtRaw) ? dtRaw : 0;
  navxState.frequency.lastStamp = stampSec;
  const elapsed = stampSec - (navxState.frequency.startStamp ?? stampSec);
  const isJump = dt > NAVX_ODOM_JUMP_THRESHOLD;
  const point = {
    x: Number(elapsed.toFixed(2)),
    dt,
    jump: isJump
  };
  navxState.frequency.history.push(point);
  if (navxState.frequency.history.length > NAVX_ODOM_MAX_POINTS) {
    navxState.frequency.history = navxState.frequency.history.slice(-NAVX_ODOM_MAX_POINTS);
  }
  navxRenderFrequencyChart();
}

function startNavAccelerationMonitoring() {
  if (!ros || !ros.isConnected || !navxState.salidasActive || navxState.activePanel !== 'aceleraciones') return;
  if (!navxState.acceleration.wheelNames.length) return;
  navxState.acceleration.wheelNames.forEach((_, idx) => {
    if (!navxState.acceleration.highWheelsEnabled && idx >= 2) return;
    ensureNavAccelerationChart(idx);
  });
  if (navxState.acceleration.topic) return;
  navxState.acceleration.topic = new ROSLIB.Topic({
    ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
  });
  navxState.acceleration.topic.subscribe(handleNavAccelerationMessage);
}

function stopNavAccelerationMonitoring() {
  if (navxState.acceleration.topic) {
    navxState.acceleration.topic.unsubscribe();
  }
  navxState.acceleration.topic = null;
  navxState.acceleration.startStamp = null;
  navxState.acceleration.lastStamp = null;
  navxState.acceleration.wheelState = navxState.acceleration.wheelNames.map(() => ({
    lastLinearVel: null,
    lastPos: null
  }));
  Object.values(navxState.acceleration.charts).forEach(chart => navxClearChart(chart));
}

function ensureNavAccelerationChart(idx) {
  if (navxState.acceleration.charts[idx]) return navxState.acceleration.charts[idx];
  const canvas = document.getElementById(`nav-chart-wheel-accel-${idx + 1}`);
  if (!canvas || !window.Chart) return null;
  const palette = ['#2563eb', '#22c55e', '#f97316', '#a855f7', '#6366f1', '#10b981'];
  const wheelName = navxState.acceleration.wheelNames[idx] || `Rueda ${idx + 1}`;
  const chart = new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: wheelName,
          data: [],
          borderColor: palette[idx % palette.length],
          borderWidth: 2,
          pointRadius: 1,
          tension: 0.12,
          spanGaps: true
        }
      ]
    },
    options: {
      responsive: false,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tiempo (s)' } },
        y: { title: { display: true, text: 'Aceleración (m/s²)' } }
      },
      plugins: {
        legend: { display: false }
      }
    }
  });
  navxState.acceleration.charts[idx] = chart;
  return chart;
}

function handleNavAccelerationMessage(msg) {
  if (!navxState.acceleration.wheelNames.length) return;
  const stampSec = navxStampToSeconds(msg && msg.header ? msg.header.stamp : null) ?? (Date.now() / 1000);
  if (!Number.isFinite(stampSec)) return;
  if (navxState.acceleration.startStamp === null) {
    navxState.acceleration.startStamp = stampSec;
  }

  if (navxState.acceleration.lastStamp === null) {
    navxState.acceleration.lastStamp = stampSec;
    primeNavAccelerationState(msg);
    return;
  }

  const dt = stampSec - navxState.acceleration.lastStamp;
  navxState.acceleration.lastStamp = stampSec;

  if (!Number.isFinite(dt) || dt <= 0) {
    primeNavAccelerationState(msg);
    return;
  }

  const elapsed = Number((stampSec - navxState.acceleration.startStamp).toFixed(2));
  const accValues = navxState.acceleration.wheelNames.map((wheel, idx) => {
    const wheelIdx = Array.isArray(msg.name) ? msg.name.indexOf(wheel) : -1;
    if (wheelIdx < 0) return null;

    const state = navxState.acceleration.wheelState[idx];
    if (!state) return null;

    let currentVelLinear = null;
    if (Array.isArray(msg.velocity) && Number.isFinite(msg.velocity[wheelIdx])) {
      currentVelLinear = msg.velocity[wheelIdx] * RADIO_RUEDA;
    }

    const hasPosition = Array.isArray(msg.position) && Number.isFinite(msg.position[wheelIdx]);
    const currentPos = hasPosition ? msg.position[wheelIdx] : null;

    if (currentVelLinear === null && hasPosition) {
      if (state.lastPos !== null) {
        const velFromPos = (currentPos - state.lastPos) / dt;
        currentVelLinear = velFromPos * RADIO_RUEDA;
      }
    }

    if (hasPosition) {
      state.lastPos = currentPos;
    }

    if (currentVelLinear === null) {
      return null;
    }

    let accel = null;
    if (state.lastLinearVel !== null) {
      accel = (currentVelLinear - state.lastLinearVel) / dt;
    }
    state.lastLinearVel = currentVelLinear;

    return Number.isFinite(accel) ? accel : null;
  });

  if (!accValues.some(val => val !== null && Number.isFinite(val))) {
    return;
  }

  navxState.acceleration.wheelNames.forEach((wheel, idx) => {
    const chart = ensureNavAccelerationChart(idx);
    if (!chart) return;
    const value = accValues[idx];
    if (!navxState.acceleration.highWheelsEnabled && idx >= 2) {
      return;
    }
    navxAppendData(chart, elapsed, [Number.isFinite(value) ? value : null], NAVX_ACCEL_MAX_POINTS);
  });
}

function primeNavAccelerationState(msg) {
  const names = Array.isArray(msg.name) ? msg.name : [];
  navxState.acceleration.wheelNames.forEach((wheel, idx) => {
    const wheelIdx = names.indexOf(wheel);
    if (wheelIdx < 0) return;
    const state = navxState.acceleration.wheelState[idx];
    if (!state) return;
    if (Array.isArray(msg.velocity) && Number.isFinite(msg.velocity[wheelIdx])) {
      state.lastLinearVel = msg.velocity[wheelIdx] * RADIO_RUEDA;
    }
    if (Array.isArray(msg.position) && Number.isFinite(msg.position[wheelIdx])) {
      state.lastPos = msg.position[wheelIdx];
    }
  });
}

function navxAppendData(chart, label, values, limit) {
  if (!chart) return;
  chart.data.labels.push(label);
  chart.data.datasets.forEach((dataset, idx) => {
    dataset.data.push(values[idx] ?? null);
  });
  if (chart.data.labels.length > limit) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(dataset => dataset.data.shift());
  }
  chart.update();
}

function navxClearChart(chart) {
  if (!chart) return;
  chart.data.labels.length = 0;
  chart.data.datasets.forEach(dataset => {
    dataset.data.length = 0;
  });
  chart.update();
}

function navxStampToSeconds(stamp) {
  if (!stamp) return null;
  const sec = Number(stamp.sec ?? stamp.secs ?? 0);
  const nanosec = Number(stamp.nanosec ?? stamp.nsecs ?? 0);
  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) return null;
  return sec + nanosec / 1e9;
}

function navxExtractTwistFromMessage(topicType, message) {
  if (!message) return null;
  const type = typeof topicType === 'string' ? topicType.toLowerCase() : '';
  if (type.includes('twiststamped')) {
    return message.twist ?? null;
  }
  if (!message.linear && message.twist) {
    return message.twist;
  }
  return message;
}

if (botonesGraficas.length > 0) {
  botonesGraficas.forEach(boton => {
    boton.addEventListener('click', function () {
      botonesGraficas.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      const tipo = this.dataset.tipo;

      // Oculta todos los paneles
      if (divGraficasOdometro) divGraficasOdometro.style.display = "none";
      if (divGraficasEncoders) divGraficasEncoders.style.display = "none";
      if (divGraficasMotores) divGraficasMotores.style.display = "none";
      if (divGraficasBateria) divGraficasBateria.style.display = "none";
      if (divGraficasUnicas) divGraficasUnicas.style.display = "none";
      if (divGraficasMPU) divGraficasMPU.style.display = "none";      // << NUEVO

      // Siempre desactiva todos antes de activar uno
      if (typeof desactivarGraficasOdo === "function") desactivarGraficasOdo();
      if (typeof desactivarGraficasEncoders === "function") desactivarGraficasEncoders();
      if (typeof desactivarGraficasCorriente === "function") desactivarGraficasCorriente();
      if (typeof desactivarGraficasBateria === "function") desactivarGraficasBateria();
      if (typeof desactivarGraficasMPU === "function") desactivarGraficasMPU(); // << NUEVO

      // Solo activa el que corresponde
      if (tipo === "odometro" && divGraficasOdometro) {
        divGraficasOdometro.style.display = "flex";
        if (typeof updateOdometryLimits === "function") updateOdometryLimits();
        if (typeof activarGraficasOdo === "function") activarGraficasOdo();
      }
      if (tipo === "encoders" && divGraficasEncoders) {
        divGraficasEncoders.style.display = "flex";
        if (typeof updateEncoderLimits === "function") updateEncoderLimits();
        if (typeof activarGraficasEncoders === "function") activarGraficasEncoders();
      }
      if (tipo === "motores" && divGraficasMotores) {
        divGraficasMotores.style.display = "flex";
        if (typeof updateCorrienteLimits === "function") updateCorrienteLimits();
        if (typeof activarGraficasCorriente === "function") activarGraficasCorriente();
      }
      if (tipo === "bateria" && divGraficasBateria) {
        divGraficasBateria.style.display = "flex";
        if (typeof activarGraficasBateria === "function") activarGraficasBateria();
      }
      if (tipo === "unicas" && divGraficasUnicas) {
        divGraficasUnicas.style.display = "block";
      }
      if (tipo === "mpu" && divGraficasMPU) {  // << NUEVO
        divGraficasMPU.style.display = "flex";
        if (typeof activarGraficasMPU === "function") activarGraficasMPU();
      }
    });
  });

  // Por defecto, muestra odometría (puedes poner encoders o motores si prefieres)
  const botonOdometroInicial = document.querySelector('[data-tipo="odometro"]');
  if (botonOdometroInicial) botonOdometroInicial.click();
}

});
