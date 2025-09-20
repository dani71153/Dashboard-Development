document.addEventListener('DOMContentLoaded', () => {

  // ========================
  // 1. Estado e integración con ROS (roslibjs)
  // ========================
  const rosStatus = document.getElementById('ros-status');
  const topicStatus = document.getElementById('topic-status');
  const datosRos = document.getElementById('datos-ros');


  if (window.Chart && window['chartjsPluginAnnotation']) {
    Chart.register(window['chartjsPluginAnnotation']);
  }
  
  
  if (rosStatus) rosStatus.textContent = '🔴 Desconectado';
  if (topicStatus) topicStatus.textContent = '—';

  const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    if (rosStatus) rosStatus.textContent = '🟢 Conectado';
    if (topicStatus) topicStatus.textContent = '—';
  });
  ros.on('close', function() {
    if (rosStatus) rosStatus.textContent = '🔴 Desconectado';
    if (topicStatus) topicStatus.textContent = '—';
  });
  ros.on('error', function() {
    if (rosStatus) rosStatus.textContent = '⚠️ Error';
    if (topicStatus) topicStatus.textContent = '—';
  });

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

  function showTab(tabId) {
    sections.forEach(section => {
      section.style.display = (section.id === tabId) ? 'block' : 'none';
    });
    navLinks.forEach(link => {
      link.classList.toggle('active', link.dataset.tab === tabId);
    });
  }

  function getTabFromHash() {
    const hash = window.location.hash.replace('#', '');
    const tabIds = Array.from(sections).map(sec => sec.id);
    return tabIds.includes(hash) ? hash : tabIds[0];
  }

  function handleHashChange() {
    showTab(getTabFromHash());
  }

  window.addEventListener('hashchange', handleHashChange);
  handleHashChange();

  // ========================
  // 5. Visualización de Nodos y Tópicos ROS
  // ========================
  const contenedorNodos = document.getElementById('contenedor-nodos');
  const contenedorTopicos = document.getElementById('contenedor-topicos');
  const btnActualizar = document.getElementById('btn-actualizar-nodos-topicos');

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
  const dropdownSeleccion = document.getElementById('dropdown-seleccion');
  const dropdownComando = document.getElementById('dropdown-comando');
  const resultadoComando = document.getElementById('resultado-comando');

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

  // === Lógica de ejecución de comandos del dropdown ===
  if (dropdownComando) {
    dropdownComando.addEventListener('change', function() {
      if (!dropdownSeleccion || !resultadoComando) return;
      resultadoComando.textContent = '';
      const valor = dropdownSeleccion.value;
      if (!valor) {
        resultadoComando.textContent = 'Primero selecciona un nodo o tópico.';
        return;
      }
      const [tipo, nombre] = valor.split(':');
      const comando = dropdownComando.value;

      if (comando === 'info') {
        if (tipo === 'topic') {
          // Tipo de mensaje
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

  if (dropdownSeleccion) {
    dropdownSeleccion.addEventListener('change', function() {
      if (resultadoComando) resultadoComando.textContent = '';
      if (dropdownComando) dropdownComando.selectedIndex = 0;
    });
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

const topicSelect = document.getElementById('topic-select');
const publishRate = document.getElementById('publish-rate');
const currentTopic = document.getElementById('current-topic');
const pubStatus = document.getElementById('pub-status');

let cmdVelTopic = null;
let pubInterval = null;
let lastCmd = { linear: 0, angular: 0 };
let currentHz = Number(publishRate.value) || 10;

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
    if (!ros.isConnected || !cmdVelTopic) return;
    publishTwistStamped(lastCmd.linear, lastCmd.angular);
    if (pubStatus) pubStatus.textContent =
      `Enviando a "${topicSelect.value}" a ${currentHz} Hz. Lineal: ${lastCmd.linear}, Angular: ${lastCmd.angular}`;
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
  [btnFwd, btnBack, btnLeft, btnRight, btnStop,
   sliderLinear, sliderAngular, inputLinear, inputAngular,
   topicSelect, publishRate].forEach(elem => {
    if (elem) elem.disabled = !conectado;
  });
  if (controlStatus) {
    controlStatus.textContent = conectado
      ? 'Control habilitado. ROS conectado.'
      : 'Control deshabilitado. ROS no conectado.';
    controlStatus.style.color = conectado ? "#080" : "#c00";
  }
  if (pubStatus) {
    pubStatus.textContent = conectado
      ? `Enviando a "${topicSelect.value}" a ${currentHz} Hz.`
      : 'Sin conexión.';
  }
}

// Botones de movimiento: actualizan el último comando y lo envían una vez
function setTwist(linear, angular) {
  if (!ros.isConnected || !cmdVelTopic) return;
  lastCmd = { linear, angular };
  publishTwistStamped(linear, angular); // Publica una vez de inmediato
  // Se seguirá publicando en el intervalo configurado
}

if (btnFwd) btnFwd.onclick = () => setTwist(Number(inputLinear.value), 0);
if (btnBack) btnBack.onclick = () => setTwist(-Number(inputLinear.value), 0);
if (btnLeft) btnLeft.onclick = () => setTwist(0, Number(inputAngular.value));
if (btnRight) btnRight.onclick = () => setTwist(0, -Number(inputAngular.value));
if (btnStop) btnStop.onclick = () => setTwist(0, 0);

// Estado inicial
actualizarEstadoControl();

// Al conectar/desconectar ROS, actualiza controles
ros.on('connection', actualizarEstadoControl);
ros.on('close', actualizarEstadoControl);
ros.on('error', actualizarEstadoControl);

// Atajos de teclado: WASD y espacio
window.addEventListener('keydown', (e) => {
  if (!ros.isConnected || !cmdVelTopic) return;
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

// ======================= Parámetros =======================
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
let tiempoInicio = null;

const divStats = document.getElementById('odometria-stats');
if (divStats) divStats.innerHTML = "<b>Esperando datos...</b>";

// Paleta de colores para los valores
function colorValor(val) {
  if (Math.abs(val) > Math.abs(VEL_MAX_MS)) return '#d22'; // rojo fuerte fuera de rango
  if (Math.abs(val) > 0.8 * Math.abs(VEL_MAX_MS)) return '#fa0'; // naranja advertencia
  if (Math.abs(val) > 0.5 * Math.abs(VEL_MAX_MS)) return '#3a7'; // verde medio
  return '#08d'; // azul (seguro)
}

// Inicializar gráficas (con Chart.js)
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

// Listener para /joint_states
const jointStatesListener = new ROSLIB.Topic({
  ros: ros,
  name: '/joint_states',
  messageType: 'sensor_msgs/JointState'
});

jointStatesListener.subscribe(function(msg) {
  if (!tiempoInicio) tiempoInicio = Date.now();
  const tiempo = ((Date.now() - tiempoInicio) / 1000).toFixed(1);
  let allVals = [];
  let detalles = [];

  nombresRuedas.forEach((nombreRueda, idx) => {
    const index = msg.name.indexOf(nombreRueda);
    if (index >= 0 && msg.velocity && msg.velocity[index] !== undefined && chartsRuedas[idx]) {
      // Convertir a m/s
      const valMS = msg.velocity[index] * RADIO_RUEDA;
      allVals.push(valMS);

      const chart = chartsRuedas[idx];
      chart.data.labels.push(tiempo);
      chart.data.datasets[0].data.push(valMS);
      if (chart.data.labels.length > 50) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
      }
      chart.update();

      // Estadísticas por rueda (últimos 20)
      const last = chart.data.datasets[0].data.slice(-20);
      const vmax = Math.max(...last).toFixed(3);
      const vmin = Math.min(...last).toFixed(3);
      const vavg = (last.reduce((a, b) => a + b, 0) / last.length).toFixed(3);

      detalles.push(
        `<div style="margin-bottom:0.5em;">
          <b style="color:#2987d6;">Rueda ${idx+1}</b><br>
          <span style="color:${colorValor(vmax)}">Max:</span> ${vmax}<br>
          <span style="color:${colorValor(vmin)}">Min:</span> ${vmin}<br>
          <span style="color:${colorValor(vavg)}">Prom:</span> ${vavg}
        </div>`
      );
    }
  });

  // Estadística global
  if (divStats && allVals.length) {
    const max = Math.max(...allVals).toFixed(3);
    const min = Math.min(...allVals).toFixed(3);
    const avg = (allVals.reduce((a, b) => a + b, 0) / allVals.length).toFixed(3);
    divStats.innerHTML =
      `<b>Resumen global</b><br>
      <span style="color:${colorValor(max)};">Máx: ${max}</span> m/s<br>
      <span style="color:${colorValor(min)};">Mín: ${min}</span> m/s<br>
      <span style="color:${colorValor(avg)};">Promedio: ${avg}</span> m/s
      <hr style="margin:0.5em 0;">
      ${detalles.join('')}`;
  }
});

// Mostrar/ocultar gráficas
const botonesGraficas = document.querySelectorAll('.boton-grafica');
const divGraficasOdometro = document.getElementById('graficas-odometro');
const divGraficasUnicas = document.getElementById('graficas-unicas');
if (divGraficasOdometro && divGraficasUnicas && botonesGraficas.length > 0) {
  botonesGraficas.forEach(boton => {
    boton.addEventListener('click', function() {
      botonesGraficas.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      const tipo = this.dataset.tipo;
      if (tipo === "odometro") {
        divGraficasOdometro.style.display = "flex";
        divGraficasUnicas.style.display = "none";
      } else {
        divGraficasOdometro.style.display = "none";
        divGraficasUnicas.style.display = "block";
      }
    });
  });
}

// Límites de odometría: inputs y switches
const inputMaxVel = document.getElementById('input-max-vel');
const inputMinVel = document.getElementById('input-min-vel');
const toggleMaxLine = document.getElementById('toggle-maxline');
const toggleMinLine = document.getElementById('toggle-minline');
function updateOdometryLimits() {
  const maxVel = parseFloat(inputMaxVel.value);
  const minVel = parseFloat(inputMinVel.value);
  const showMax = toggleMaxLine.checked;
  const showMin = toggleMinLine.checked;
  for (const chart of chartsRuedas) {
    // Línea máxima
    if (showMax) {
      chart.options.plugins.annotation.annotations.maxVel = {
        type: 'line', yMin: maxVel, yMax: maxVel, borderColor: 'red',
        borderWidth: 2, borderDash: [4, 4],
        label: { display: true, content: 'Máx', color: 'red', position: 'end', font: { size: 11 } }
      };
    } else {
      delete chart.options.plugins.annotation.annotations.maxVel;
    }
    // Línea mínima
    if (showMin) {
      chart.options.plugins.annotation.annotations.minVel = {
        type: 'line', yMin: minVel, yMax: minVel, borderColor: 'red',
        borderWidth: 2, borderDash: [4, 4],
        label: { display: true, content: 'Mín', color: 'red', position: 'end', font: { size: 11 } }
      };
    } else {
      delete chart.options.plugins.annotation.annotations.minVel;
    }
    // Ajuste de eje Y (opcional)
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

//Odometria final




});
