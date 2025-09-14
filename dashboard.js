document.addEventListener('DOMContentLoaded', () => {
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
  handleHashChange(); // Mostrar la correcta al cargar


  //Grafica 1
   // Cuando la página cargue, crea la gráfica si existe el canvas
  const ctx = document.getElementById('miGrafica');
  if (ctx) {
    new Chart(ctx, {
      type: 'bar',
      data: {
        labels: ['Enero', 'Febrero', 'Marzo', 'Abril'],
        datasets: [{
          label: 'Ventas',
          data: [10, 20, 15, 25],
        }]
      }
    });
  }
});
