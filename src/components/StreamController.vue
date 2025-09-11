<template>
  <div class="relative">
    <button @click="closeController" class="absolute top-0 right-0 p-1 rounded-full hover:bg-gray-200 focus:outline-none focus:ring-2 focus:ring-indigo-500">
      <svg xmlns="http://www.w3.org/2000/svg" class="h-5 w-5 text-gray-600" viewBox="0 0 20 20" fill="currentColor">
        <path fill-rule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clip-rule="evenodd" />
      </svg>
    </button>
    <h3 class="text-lg font-semibold text-gray-800 mb-4">Control for {{ selectedCardId }}</h3>

    <div class="mb-4">
      <label for="fps-slider" class="block text-sm font-medium text-gray-700 mb-1">FPS:</label>
      <input
        type="range"
        id="fps-slider"
        min="1"
        max="30"
        v-model="fps"
        class="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-700"
      />
      <span class="text-sm text-gray-600">{{ fps }} FPS</span>
    </div>

    <div class="mb-4">
      <label for="brightness-slider" class="block text-sm font-medium text-gray-700 mb-1">Brightness:</label>
      <input
        type="range"
        id="brightness-slider"
        min="0"
        max="100"
        v-model="brightness"
        class="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-700"
      />
      <span class="text-sm text-gray-600">{{ brightness }}%</span>
    </div>

    <button
      @click="applySettings"
      class="w-full bg-indigo-600 text-white py-2 px-4 rounded-md hover:bg-indigo-700 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-2"
    >
      Apply Settings
    </button>
  </div>
</template>

<script setup>
import { ref, watch, defineProps, defineEmits } from 'vue';
import { useROS } from '../composables/useRos.js';

const props = defineProps({
  selectedCardId: {
    type: String,
    required: true,
  },
});

const emit = defineEmits(['close-controller']);

const { ros, isConnected } = useROS();

const fps = ref(15);
const brightness = ref(50);

// Watch for changes in selectedCardId to potentially load settings or reset
watch(() => props.selectedCardId, (newId) => {
  console.log(`Controller selected for: ${newId}`);
  // In a real application, you would fetch current settings for newId here
  // For now, we'll just reset to default or some arbitrary values
  fps.value = 15;
  brightness.value = 50;
});

const applySettings = () => {
  if (!isConnected.value) {
    console.error('Not connected to ROS. Cannot apply settings.');
    return;
  }

  console.log(`Applying settings for ${props.selectedCardId}: FPS=${fps.value}, Brightness=${brightness.value}`);

  // !TOBECHANGE: Implement actual ROS communication here
  // This is a placeholder for how you might publish to a topic or call a service
  // Example: Publishing to a topic (replace with actual topic and message type)
  // const cameraSettingsPublisher = new ROSLIB.Topic({
  //   ros: ros.value,
  //   name: `/camera/${props.selectedCardId}/settings`, // Example topic
  //   messageType: 'std_msgs/String', // Example message type
  // });
  // const message = new ROSLIB.Message({
  //   data: JSON.stringify({ fps: fps.value, brightness: brightness.value }),
  // });
  // cameraSettingsPublisher.publish(message);

  // Example: Calling a ROS service (replace with actual service name and type)
  // const setCameraSettingsClient = new ROSLIB.Service({
  //   ros: ros.value,
  //   name: `/camera/${props.selectedCardId}/set_settings`, // Example service
  //   serviceType: 'your_package/SetCameraSettings', // Example service type
  // });
  // const request = new ROSLIB.ServiceRequest({
  //   fps: fps.value,
  //   brightness: brightness.value,
  // });
  // setCameraSettingsClient.callService(request, (result) => {
  //   console.log('Service response: ' + result.success);
  // });

  alert(`Settings applied for ${props.selectedCardId}: FPS=${fps.value}, Brightness=${brightness.value}`);
};

const closeController = () => {
  emit('close-controller');
};
</script>
