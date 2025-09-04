import { createApp } from "vue";
import { createPinia } from 'pinia';
import "./style.css";
import App from "./App.vue";
import router from "./router";
import { createROS } from './composables/useRos';

const app = createApp(App);
const pinia = createPinia();

// Initialize ROS context 
const rosState = createROS();
app.provide(rosState.ROS_KEY, rosState);

app.use(pinia);
app.use(router);
app.mount("#app");
