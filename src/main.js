import { createApp } from "vue";
import "./style.css";
import App from "./App.vue";
import router from "./router";
import { createROS } from './composables/useRos';

const app = createApp(App);
app.use(router);
createROS(); // Initialize ROS context
app.mount("#app");
