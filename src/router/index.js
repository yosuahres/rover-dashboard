import { createRouter, createWebHistory } from 'vue-router';
import MainView from '../views/MainView.vue';
import DriveView from '../views/CamView.vue';
import PIDDebugView from '../views/TerminalView.vue';
import ScienceView from '../views/ScienceView.vue';
import MiscView from '../views/DebugView.vue';

const routes = [
  {
    path: '/',
    redirect: '/main',
  },
  {
    path: '/main',
    name: 'Main',
    component: MainView,
  },
  {
    path: '/drive',
    name: 'Drive',
    component: DriveView,
  },
  {
    path: '/pid_debug',
    name: 'PID_Debug',
    component: PIDDebugView,
  },
  {
    path: '/science',
    name: 'Science',
    component: ScienceView,
  },
  {
    path: '/misc',
    name: 'Misc',
    component: MiscView,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
