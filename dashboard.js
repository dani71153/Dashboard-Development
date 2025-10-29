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
      name: '/pose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
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
window.ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
const ros = window.ros;

// Eventos de conexión
ros.on('connection', function () {
  if (rosStatus)   rosStatus.textContent   = '🟢 Conectado';
  if (topicStatus) topicStatus.textContent = '—';
  startSlamMonitoring();
  if (activeTab === 'mapa') {
    ensureRos2dViewer();
  }
});

ros.on('close', function () {
  if (rosStatus)   rosStatus.textContent   = '🔴 Desconectado';
  if (topicStatus) topicStatus.textContent = '—';
  stopSlamMonitoring();
  setSlamIndicator('disconnected', 'SLAMToolbox: Sin conexión');
  setSlamLastUpdate(null);
  destroyRos2dViewer();
  renderMapaPlaceholder('Conéctate a ROS para visualizar el mapa.');
});

ros.on('error', function () {
  if (rosStatus)   rosStatus.textContent   = '⚠️ Error';
  if (topicStatus) topicStatus.textContent = '—';
  stopSlamMonitoring();
  setSlamIndicator('disconnected', 'SLAMToolbox: Error de conexión');
  setSlamLastUpdate(null);
  destroyRos2dViewer();
  renderMapaPlaceholder('Error al conectar con ROS. Reintenta para recuperar el mapa.');
});

// (Opcional) Auto-reconexión simple
setInterval(() => {
  try {
    if (!ros.isConnected) ros.connect('ws://localhost:9090');
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

if (dynamicMapButton) {
  dynamicMapButton.addEventListener('click', () => requestDynamicMap(dynamicMapButton));
}

const slamServiceButtons = document.querySelectorAll('.slam-action-btn[data-slam-service]');
slamServiceButtons.forEach(btn => {
  btn.addEventListener('click', () => executeSlamServiceButton(btn));
});

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
    chart.options.scales.y.max = showMax ? maxVel * 1.2 : undefined;
    chart.options.scales.y.min = showMin ? minVel * 1.2 : undefined;
    chart.update();
  }
}

if (inputMaxVel) inputMaxVel.addEventListener('input', updateOdometryLimits);
if (inputMinVel) inputMinVel.addEventListener('input', updateOdometryLimits);
if (toggleMaxLine) toggleMaxLine.addEventListener('change', updateOdometryLimits);
if (toggleMinLine) toggleMinLine.addEventListener('change', updateOdometryLimits);
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
let modoModular = false;
const btnToggleModular = document.getElementById('toggle-modular-enc');

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
    chartsEncoders.forEach(chart => {
      chart.data.labels = [];
      chart.data.datasets[0].data = [];
      chart.update();
    });
  });
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
    if (!tiempoInicioEnc) tiempoInicioEnc = Date.now();
    const tiempo = ((Date.now() - tiempoInicioEnc) / 1000).toFixed(1);

    let allPositions = [];
    let detallesHTML = [];

    nombresEncoders.forEach((nombreEncoder, idx) => {
      const index = msg.name.indexOf(nombreEncoder);
      if (index >= 0 && msg.position && msg.position[index] !== undefined && chartsEncoders[idx]) {
        let valPos = msg.position[index];
        if (modoModular) {
          valPos = ((valPos % VUELTA_ENCODER) + VUELTA_ENCODER) % VUELTA_ENCODER;
        }
        allPositions.push(valPos);

        const chart = chartsEncoders[idx];
        chart.data.labels.push(tiempo);
        chart.data.datasets[0].data.push(valPos);

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
            <span>Máx:</span> ${pmax} rad<br>
            <span>Mín:</span> ${pmin} rad<br>
            <span>Prom:</span> ${pavg} rad
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
const divGraficasUnicas = document.getElementById('graficas-unicas');
const divGraficasMPU = document.getElementById('graficas-mpu');  // << NUEVO

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
      if (divGraficasUnicas) divGraficasUnicas.style.display = "none";
      if (divGraficasMPU) divGraficasMPU.style.display = "none";      // << NUEVO

      // Siempre desactiva todos antes de activar uno
      if (typeof desactivarGraficasOdo === "function") desactivarGraficasOdo();
      if (typeof desactivarGraficasEncoders === "function") desactivarGraficasEncoders();
      if (typeof desactivarGraficasCorriente === "function") desactivarGraficasCorriente();
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
