import { ref, computed } from 'vue';

const routes = {
  '/': 'Home', // Using string names for components, will map to actual components later
  '/about': 'About',
};

const currentPath = ref(window.location.hash);

window.addEventListener('hashchange', () => {
  currentPath.value = window.location.hash;
});

const currentView = computed(() => {
  return routes[currentPath.value.slice(1) || '/'] || 'NotFound';
});

export default {
  currentView,
  routes,
