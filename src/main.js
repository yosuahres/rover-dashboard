import { createApp } from "vue";
import { createPinia } from 'pinia';
import "./style.css";
import App from "./App.vue";
import router from "./router";
import { createROS } from './composables/useRos';

const app = createApp(App);
const pinia = createPinia();

app.use(pinia);
app.use(router);
createROS(); // Initialize ROS context
app.mount("#app");
