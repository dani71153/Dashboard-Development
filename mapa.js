// ============================================================================
// mapa.js — Visualizador de Mapa 2D para ROS2 con integración perezosa
// ============================================================================
// Este módulo maneja la visualización del mapa 2D usando ROS2DJS de forma
// eficiente, creando el viewer solo cuando se necesita y limpiando recursos
// cuando no está en uso.
// ============================================================================

(function () {
  'use strict';

  // ========== VARIABLES PRIVADAS ==========
  let viewer = null;
  let gridClient = null;
  let resizeObserver = null;
  let container = null;
  let isActive = false;

  // Pose y trayectoria del robot
  let robotArrow = null;
  let pathShape = null;
  let poseSubscriptions = [];
  let pathSubscriptions = [];
  let landmarkSubscriptions = [];
  let landmarkShapes = new Map();

  // Configuración de tópicos
  const CONFIG = {
    MAP_TOPIC: '/map',
    POSE_TOPICS: [
      { name: '/slam_toolbox/pose', types: ['geometry_msgs/PoseStamped', 'geometry_msgs/PoseWithCovarianceStamped'] },
      { name: '/robot_pose', types: ['geometry_msgs/PoseStamped'] },
      { name: '/amcl_pose', types: ['geometry_msgs/PoseWithCovarianceStamped'] }
    ],
    PATH_TOPICS: [
      { name: '/slam_toolbox/trajectory', type: 'nav_msgs/Path' },
      { name: '/path', type: 'nav_msgs/Path' },
      { name: '/plan', type: 'nav_msgs/Path' }
    ],
    LANDMARK_TOPICS: [
      '/slam_toolbox/landmarks',
      '/slam_toolbox/landmark_markers',
      '/slam_toolbox/graph_visualization'
    ],
    // Estilos visuales
    STYLES: {
      ROBOT_SIZE: 0.55,
      ROBOT_STROKE: 0.05,
      PATH_STROKE: 0.02,
      LANDMARK_MIN_SIZE: 0.05,
      LANDMARK_DEFAULT_SIZE: 0.12
    }
  };

  // ========== FUNCIONES AUXILIARES ==========

  /**
   * Convierte un quaternion a ángulo yaw (rotación en Z)
   */
  function quaternionToYaw(q) {
    const x = q.x || 0;
    const y = q.y || 0;
    const z = q.z || 0;
    const w = q.w === undefined ? 1 : q.w;
    const siny = 2.0 * (w * z + x * y);
    const cosy = 1.0 - 2.0 * (y * y + z * z);
    return Math.atan2(siny, cosy);
  }

  /**
   * Extrae la pose de diferentes tipos de mensajes ROS
   */
  function extraerPose(msg) {
    if (!msg) return null;
    // PoseWithCovarianceStamped
    if (msg.pose && msg.pose.pose) return msg.pose.pose;
    // PoseStamped
    if (msg.pose) return msg.pose;
    // Pose directa
    if (msg.position && msg.orientation) return msg;
    return null;
  }

  /**
   * Convierte color de marker ROS a formato CreateJS
   */
  function markerColorToCss(color) {
    if (typeof createjs === 'undefined') {
      return 'rgba(255, 202, 40, 0.9)';
    }
    if (!color) {
      return createjs.Graphics.getRGB(255, 202, 40, 0.9);
    }
    const r = Math.round(Math.min(Math.max(color.r || 0, 0), 1) * 255);
    const g = Math.round(Math.min(Math.max(color.g || 0, 0), 1) * 255);
    const b = Math.round(Math.min(Math.max(color.b || 0, 0), 1) * 255);
    const a = (typeof color.a === 'number') ? color.a : 0.9;
    return createjs.Graphics.getRGB(r, g, b, a);
  }

  /**
   * Verifica si las dependencias necesarias están disponibles
   */
  function verificarDependencias() {
    if (typeof ROS2D === 'undefined') {
      console.error('❌ ROS2D no está disponible. Verifica que ros2d.min.js esté cargado.');
      return false;
    }
    if (typeof createjs === 'undefined') {
      console.error('❌ CreateJS no está disponible. Verifica que easeljs.min.js esté cargado.');
      return false;
    }
    if (!window.ros) {
      console.error('❌ ROS no está inicializado. Verifica que roslib.min.js esté cargado y ros conectado.');
      return false;
    }
    return true;
  }

  // ========== GESTIÓN DEL VIEWER ==========

  /**
   * Crea el viewer de ROS2D y configura el cliente del mapa
   */
  function crearViewer(containerId) {
    if (!verificarDependencias()) return false;

    container = document.getElementById(containerId);
    if (!container) {
      console.error(`❌ No se encontró el contenedor con ID: ${containerId}`);
      return false;
    }

    // Calcular dimensiones del contenedor
    const width = container.clientWidth || container.offsetWidth || 960;
    const height = container.clientHeight || Math.max(480, Math.round(width * 0.6));

    // Asegurar altura mínima visible
    if (!container.style.height || container.style.height === 'auto') {
      container.style.height = '60vh';
    }

    console.log(`📐 Creando viewer: ${width}x${height}px`);

    try {
      // Crear el viewer principal
      viewer = new ROS2D.Viewer({
        divID: containerId,
        width: width,
        height: height,
        antialias: true
      });

      // Configurar cliente del mapa
      gridClient = new ROS2D.OccupancyGridClient({
        ros: window.ros,
        rootObject: viewer.scene,
        topic: CONFIG.MAP_TOPIC,
        continuous: true
      });

      // Manejar cambios en el mapa
      gridClient.on('change', function () {
        if (!viewer || !gridClient || !gridClient.currentGrid) return;

        const grid = gridClient.currentGrid;
        const pose = grid.pose || { position: { x: 0, y: 0 } };

        // Calcular dimensiones reales del mapa
        const rawWidth = grid.width || 0;
        const rawHeight = grid.height || 0;
        const scaleX = Math.abs(grid.scaleX || 1);
        const scaleY = Math.abs(grid.scaleY || scaleX);

        const widthMeters = rawWidth > 200 ? rawWidth * scaleX : rawWidth * (scaleX || 1);
        const heightMeters = rawHeight > 200 ? rawHeight * scaleY : rawHeight * (scaleY || 1);

        if (widthMeters && heightMeters) {
          viewer.scaleToDimensions(widthMeters, heightMeters);
        }

        viewer.shift(
          pose.position ? pose.position.x || 0 : 0,
          pose.position ? pose.position.y || 0 : 0
        );

        // Reorganizar capas (mapa, trayectoria, landmarks, robot)
        reorganizarCapas();
      });

      // Crear elementos visuales
      crearElementosVisuales();

      // Configurar auto-resize
      configurarAutoResize();

      console.log('✅ Viewer del mapa creado exitosamente');
      return true;

    } catch (error) {
      console.error('❌ Error al crear viewer:', error);
      destruir();
      return false;
    }
  }

  /**
   * Crea los elementos visuales del mapa (trayectoria, robot, etc.)
   */
  function crearElementosVisuales() {
    if (!viewer) return;

    // Crear shape para la trayectoria
    pathShape = new ROS2D.PathShape({
      ros: window.ros,
      strokeSize: CONFIG.STYLES.PATH_STROKE,
      strokeColor: createjs.Graphics.getRGB(30, 144, 255, 0.9)
    });
    viewer.scene.addChild(pathShape);

    // Crear flecha para el robot
    robotArrow = new ROS2D.NavigationArrow({
      size: CONFIG.STYLES.ROBOT_SIZE,
      strokeSize: CONFIG.STYLES.ROBOT_STROKE,
      fillColor: createjs.Graphics.getRGB(255, 99, 71, 0.95),
      pulse: false
    });
    robotArrow.visible = false;
    viewer.scene.addChild(robotArrow);

    console.log('✅ Elementos visuales creados');
  }

  /**
   * Reorganiza las capas para mantener el orden correcto de renderizado
   */
  function reorganizarCapas() {
    if (!viewer || !viewer.scene) return;

    // Orden: mapa -> trayectoria -> landmarks -> robot
    if (pathShape) {
      viewer.scene.removeChild(pathShape);
      viewer.scene.addChild(pathShape);
    }

    landmarkShapes.forEach(shape => {
      viewer.scene.removeChild(shape);
      viewer.scene.addChild(shape);
    });

    if (robotArrow) {
      viewer.scene.removeChild(robotArrow);
      viewer.scene.addChild(robotArrow);
    }

    if (viewer.scene && typeof viewer.scene.update === 'function') {
      viewer.scene.update();
    }
  }

  /**
   * Configura el observador de cambios de tamaño del contenedor
   */
  function configurarAutoResize() {
    if (!container || !viewer) return;

    resizeObserver = new ResizeObserver(() => {
      const newWidth = container.clientWidth;
      const newHeight = container.clientHeight;

      if (newWidth && newHeight && viewer && typeof viewer.resize === 'function') {
        viewer.resize(newWidth, newHeight);
        console.log(`📐 Viewer redimensionado: ${newWidth}x${newHeight}px`);
      }
    });

    resizeObserver.observe(container);
  }

  // ========== SUSCRIPCIONES A TÓPICOS ==========

  /**
   * Activa todas las suscripciones a tópicos del mapa
   */
  function activarSuscripciones() {
    if (!window.ros || !window.ros.isConnected) {
      console.warn('⚠️ ROS no está conectado. No se pueden activar suscripciones.');
      return;
    }

    // Suscribirse a tópicos de pose del robot
    CONFIG.POSE_TOPICS.forEach(cfg => {
      suscribirConDeteccion(cfg.name, cfg.types, poseSubscriptions, actualizarPoseRobot);
    });

    // Suscribirse a tópicos de trayectorias
    CONFIG.PATH_TOPICS.forEach(cfg => {
      const topic = new ROSLIB.Topic({
        ros: window.ros,
        name: cfg.name,
        messageType: cfg.type
      });
      topic.subscribe(msg => actualizarTrayectoria(msg));
      pathSubscriptions.push(topic);
    });

    // Suscribirse a tópicos de landmarks
    CONFIG.LANDMARK_TOPICS.forEach(name => {
      const topic = new ROSLIB.Topic({
        ros: window.ros,
        name: name,
        messageType: 'visualization_msgs/MarkerArray'
      });
      topic.subscribe(msg => actualizarLandmarks(msg));
      landmarkSubscriptions.push(topic);
    });

    console.log('✅ Suscripciones al mapa activadas');
  }

  /**
   * Suscribe a un tópico detectando automáticamente su tipo
   */
  function suscribirConDeteccion(nombre, candidatos, registro, handler) {
    if (!window.ros || !window.ros.isConnected) return;

    const tipos = Array.isArray(candidatos) && candidatos.length ? candidatos : [candidatos].filter(Boolean);
    if (!tipos.length) return;

    const servicio = new ROSLIB.Service({
      ros: window.ros,
      name: '/rosapi/topic_type',
      serviceType: 'rosapi/TopicType'
    });

    servicio.callService(new ROSLIB.ServiceRequest({ topic: nombre }), res => {
      if (!window.ros || !window.ros.isConnected) return;
      const tipo = res.type || tipos[0];
      if (tipo) {
        const sub = crearSuscripcion(nombre, tipo, handler);
        if (sub) registro.push(sub);
      }
    }, () => {
      if (!window.ros || !window.ros.isConnected) return;
      if (tipos[0]) {
        const sub = crearSuscripcion(nombre, tipos[0], handler);
        if (sub) registro.push(sub);
      }
    });
  }

  /**
   * Crea una suscripción a un tópico
   */
  function crearSuscripcion(nombre, tipo, handler) {
    if (!window.ros || !window.ros.isConnected) return null;

    const sub = new ROSLIB.Topic({
      ros: window.ros,
      name: nombre,
      messageType: tipo
    });
    sub.subscribe(handler);
    return sub;
  }

  /**
   * Desactiva todas las suscripciones
   */
  function desactivarSuscripciones() {
    poseSubscriptions.forEach(sub => sub.unsubscribe());
    pathSubscriptions.forEach(sub => sub.unsubscribe());
    landmarkSubscriptions.forEach(sub => sub.unsubscribe());

    poseSubscriptions = [];
    pathSubscriptions = [];
    landmarkSubscriptions = [];

    limpiarLandmarks();

    if (pathShape && typeof pathShape.setPath === 'function') {
      try {
        pathShape.setPath({ poses: [] });
      } catch (err) {
        console.warn('⚠️ No se pudo limpiar la trayectoria:', err);
      }
    }

    console.log('✅ Suscripciones desactivadas');
  }

  // ========== ACTUALIZACIÓN DE VISUALIZACIONES ==========

  /**
   * Actualiza la posición y orientación del robot
   */
  function actualizarPoseRobot(msg) {
    if (!robotArrow) return;

    const pose = extraerPose(msg);
    if (!pose || !pose.position) return;

    robotArrow.x = pose.position.x;
    robotArrow.y = -pose.position.y;

    const yaw = quaternionToYaw(pose.orientation || {});
    robotArrow.rotation = -yaw * 180 / Math.PI;

    robotArrow.visible = true;
  }

  /**
   * Actualiza la trayectoria del robot
   */
  function actualizarTrayectoria(msg) {
    if (!pathShape || !msg || !Array.isArray(msg.poses)) return;

    if (typeof pathShape.setPath === 'function') {
      pathShape.setPath(msg);
    }
  }

  /**
   * Actualiza los landmarks del SLAM
   */
  function actualizarLandmarks(msg) {
    if (!viewer || !msg || !Array.isArray(msg.markers) || msg.markers.length === 0) return;

    for (const marker of msg.markers) {
      // DELETEALL
      if (marker.action === 3) {
        limpiarLandmarks();
        continue;
      }

      const key = `${marker.ns || ''}:${marker.id}`;

      // DELETE
      if (marker.action === 2) {
        const shape = landmarkShapes.get(key);
        if (shape && viewer && viewer.scene) {
          viewer.scene.removeChild(shape);
        }
        landmarkShapes.delete(key);
        continue;
      }

      // ADD o MODIFY
      const pose = extraerPose(marker);
      if (!pose || !pose.position) continue;

      let shape = landmarkShapes.get(key);
      if (!shape) {
        shape = new createjs.Shape();
        viewer.scene.addChild(shape);
        landmarkShapes.set(key, shape);
      }

      const color = markerColorToCss(marker.color);
      const radius = marker.scale && marker.scale.x
        ? Math.max(marker.scale.x * 0.5, CONFIG.STYLES.LANDMARK_MIN_SIZE)
        : CONFIG.STYLES.LANDMARK_DEFAULT_SIZE;

      shape.graphics.clear();
      shape.graphics.beginFill(color).drawCircle(0, 0, radius);
      shape.x = pose.position.x;
      shape.y = -pose.position.y;
      shape.alpha = (marker.color && typeof marker.color.a === 'number') ? marker.color.a : 0.9;
    }
  }

  /**
   * Limpia todos los landmarks del mapa
   */
  function limpiarLandmarks() {
    if (!landmarkShapes) return;

    landmarkShapes.forEach(shape => {
      if (viewer && viewer.scene && shape) {
        viewer.scene.removeChild(shape);
      }
    });

    landmarkShapes.clear();
  }

  /**
   * Oculta el robot del mapa
   */
  function ocultarRobot() {
    if (robotArrow) {
      robotArrow.visible = false;
    }
  }

  // ========== DESTRUCCIÓN Y LIMPIEZA ==========

  /**
   * Destruye completamente el viewer y libera recursos
   */
  function destruir() {
    console.log('🧹 Limpiando recursos del mapa...');

    // Desactivar suscripciones
    desactivarSuscripciones();

    // Limpiar cliente del mapa
    if (gridClient) {
      if (gridClient.rootObject) {
        gridClient.rootObject.removeAllChildren();
      }
      if (typeof gridClient.unsubscribe === 'function') {
        gridClient.unsubscribe();
      }
      gridClient = null;
    }

    // Limpiar scene del viewer
    if (viewer && viewer.scene) {
      viewer.scene.removeAllChildren();
    }

    // Desconectar resize observer
    if (resizeObserver && container) {
      resizeObserver.unobserve(container);
      resizeObserver.disconnect();
    }

    // Limpiar referencias
    viewer = null;
    robotArrow = null;
    pathShape = null;
    resizeObserver = null;
    container = null;
    isActive = false;

    console.log('✅ Recursos del mapa liberados');
  }

  // ========== API PÚBLICA ==========

  window.MapaROS2D = {
    /**
     * Activa el visualizador del mapa
     * @param {string} containerId - ID del contenedor HTML
     * @returns {boolean} - true si se activó correctamente
     */
    activar: function (containerId = 'mapa-canvas') {
      if (isActive) {
        console.log('ℹ️ El mapa ya está activo');
        return true;
      }

      if (!window.ros || !window.ros.isConnected) {
        console.warn('⚠️ ROS no está conectado. Esperando conexión...');
        return false;
      }

      console.log('🗺️ Activando visualizador del mapa...');

      if (crearViewer(containerId)) {
        activarSuscripciones();
        isActive = true;
        return true;
      }

      return false;
    },

    /**
     * Desactiva el visualizador del mapa y libera recursos
     */
    desactivar: function () {
      if (!isActive) {
        console.log('ℹ️ El mapa ya está desactivado');
        return;
      }

      console.log('🗺️ Desactivando visualizador del mapa...');
      destruir();
    },

    /**
     * Verifica si el mapa está activo
     * @returns {boolean}
     */
    estaActivo: function () {
      return isActive;
    },

    /**
     * Oculta temporalmente el robot sin desactivar el mapa
     */
    ocultarRobot: function () {
      ocultarRobot();
    },

    /**
     * Muestra el robot en el mapa
     */
    mostrarRobot: function () {
      if (robotArrow) {
        robotArrow.visible = true;
      }
    },

    /**
     * Obtiene información del estado actual del mapa
     * @returns {object}
     */
    obtenerEstado: function () {
      return {
        activo: isActive,
        visorCreado: viewer !== null,
        robotVisible: robotArrow ? robotArrow.visible : false,
        numLandmarks: landmarkShapes.size,
        suscripcionesPose: poseSubscriptions.length,
        suscripcionesTrayectoria: pathSubscriptions.length,
        suscripcionesLandmarks: landmarkSubscriptions.length
      };
    }
  };

  console.log('✅ Módulo MapaROS2D cargado');

  // ========== INTEGRACIÓN CON EVENTOS DEL DASHBOARD ==========

  /**
   * Auto-activar cuando ROS se conecta (si la pestaña de mapa está visible)
   */
  if (window.ros) {
    window.ros.on('connection', function() {
      const hash = window.location.hash.replace('#', '');
      if (hash === 'mapa' && !isActive) {
        setTimeout(() => {
          window.MapaROS2D.activar('mapa-canvas');
        }, 100);
      }
    });

    window.ros.on('close', function() {
      if (isActive) {
        ocultarRobot();
        console.log('⚠️ Conexión ROS perdida - Robot ocultado del mapa');
      }
    });
  }

})();