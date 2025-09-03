import { ref, readonly, provide, inject } from 'vue';
import ROSLIB from 'roslib';

const ROS_KEY = Symbol('ros');

export function createROS() {
  const ros = ref(null);
  const loading = ref(false);
  const status = ref(null); // 'Connected' | 'Disconnected' | null
  const message = ref(null);
  const server = ref('');

  function initializeROS(ip, port) {
    const url = `ws://${ip}:${port}`;
    loading.value = true;
    server.value = ip;

    const rosConnection = new ROSLIB.Ros({ url });

    ros.value = rosConnection;

    rosConnection.on('connection', () => {
      message.value = `Connected to ROS master: ${url}`;
      loading.value = false;
      status.value = 'Connected';
    });

    rosConnection.on('error', () => {
      message.value = `Error connecting to ROS: ${url}`;
      loading.value = false;
      status.value = 'Disconnected';
    });

    rosConnection.on('close', () => {
      console.log('ROS connection closed');
      message.value = `Closed connection to ROS master: ${url}`;
      loading.value = false;
      status.value = 'Disconnected';
    });
  }

  const rosState = {
    ros: readonly(ros),
    loading: readonly(loading),
    server: readonly(server),
    status: readonly(status),
    message: readonly(message),
    initializeROS,
  };

  provide(ROS_KEY, rosState);

  return rosState;
}

export function useROS() {
  const rosState = inject(ROS_KEY);
  if (!rosState) {
    throw new Error('useROS must be used within a createROS setup.');
  }
  return rosState;
}
