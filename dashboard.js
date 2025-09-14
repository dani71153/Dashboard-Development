document.addEventListener('DOMContentLoaded', () => {

  // ========================
  // 1. Estado e integración con ROS (roslibjs)
  // ========================
  const rosStatus = document.getElementById('ros-status');
  const topicStatus = document.getElementById('topic-status');
  const datosRos = document.getElementById('datos-ros'); // Opcional, si quieres mostrar datos en texto

  // Estado inicial
  if (rosStatus) rosStatus.textContent = '🔴 Desconectado';
  if (topicStatus) topicStatus.textContent = '—';

  // Conectar a rosbridge
  const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  // Eventos de conexión/desconexión/error
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
          x: {
            type: 'linear',
            position: 'bottom',
            title: { display: true, text: 'Eje X' }
          },
          y: {
            title: { display: true, text: 'Eje Y' }
          }
        }
      }
    });
  }

  // ========================
  // 3. Suscripción al tópico de ROS y actualización de datos
  // ========================
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/datos_grafica', // Cambia por tu tópico real
    messageType: 'std_msgs/Float32MultiArray'
  });

  listener.subscribe(function(message) {
    if (topicStatus) topicStatus.textContent = '🟢 Recibiendo datos (' + new Date().toLocaleTimeString() + ')';

    // Mostrar los datos en consola
    console.log('Datos recibidos:', message.data);

    // Opcional: Mostrar los datos en texto (HTML)
    if (datosRos) datosRos.textContent = JSON.stringify(message.data, null, 2);

    // Actualizar la gráfica si hay al menos dos elementos (pares x,y)
    if (chart && message.data && message.data.length > 1) {
      // Convierte [x1, y1, x2, y2, ...] en [{x, y}, ...]
      const puntos = [];
      for (let i = 0; i < message.data.length; i += 2) {
        if (message.data[i + 1] !== undefined) { // Evita error si el arreglo tiene longitud impar
          puntos.push({ x: message.data[i], y: message.data[i + 1] });
        }
      }
      chart.data.datasets[0].data = puntos;
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
  handleHashChange(); // Mostrar la pestaña correcta al cargar

});
