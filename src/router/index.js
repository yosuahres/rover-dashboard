import { createRouter, createWebHistory } from 'vue-router';
import MainView from '../views/MainView.vue';
import CameraView from '../views/CamView.vue';
import DebugView from '../views/DebugView.vue';
import ScienceView from '../views/ScienceView.vue';
import ConnectionView from '../views/ConnectionView.vue';

const routes = [
  {
    path: '/',
    redirect: '/connect',
  },
  {
    path: '/connect',
    name: 'Connect',
    component: ConnectionView,
  },
  {
    path: '/data',
    name: 'Data',
    component: MainView,
  },
  {
    path: '/camera',
    name: 'Camera',
    component: CameraView,
  },
  {
    path: '/debug',
    name: 'Debug',
    component: DebugView,
  },
  {
    path: '/science',
    name: 'Science',
    component: ScienceView,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
