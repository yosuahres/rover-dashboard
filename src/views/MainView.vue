<template>
  <div class="p-4 h-full">
    <div class="mb-4 p-4 border rounded-lg shadow-md bg-white">
      <h2 class="text-xl text-black font-semibold mb-2">ROS Status</h2>
      <p class="text-sm" :class="{ 'text-green-600': status === 'Connected', 'text-red-600': status === 'Disconnected', 'text-gray-600': status === null }">
        Status: {{ status || 'Not Connected' }} - {{ message }}
      </p>
    </div>

    <div class="flex flex-col md:flex-row gap-2 mt-2">
      <div class="md:w-1/4 p-4 border rounded-lg shadow-md bg-white">
        <h2 class="text-xl text-black font-semibold mb-4">ROS Nodes ({{ nodes.length }})</h2>
        <ul class="list-disc pl-5 text-gray-700 max-h-96 overflow-y-auto">
          <li v-for="node in nodes" :key="node" class="mb-1">{{ node }}</li>
        </ul>
      </div>

      <div class="md:w-3/4 p-4 border rounded-lg shadow-md bg-white">
        <h2 class="text-xl text-black font-semibold mb-4">ROS Topics ({{ topics.size }})</h2>
        <div class="overflow-x-auto">
          <table class="min-w-full divide-y divide-gray-200">
            <thead class="bg-gray-800">
              <tr>
                <th scope="col" class="px-6 py-3 text-left text-xs font-medium text-white uppercase tracking-wider">Topic</th>
                <th scope="col" class="px-6 py-3 text-left text-xs font-medium text-white uppercase tracking-wider">Type</th>
                <th scope="col" class="px-6 py-3 text-left text-xs font-medium text-white uppercase tracking-wider">Message</th>
              </tr>
            </thead>
            <tbody class="bg-white divide-y divide-gray-200">
              <tr v-for="[topicName, topicType] in topics" :key="topicName">
                <td class="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">{{ topicName }}</td>
                <td class="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{{ topicType }}</td>
                <td class="px-6 py-4 text-sm text-gray-500">
                  <div v-if="messages.has(topicName)" class="max-h-20 overflow-y-auto bg-gray-50 p-2 rounded-md">
                    <pre class="text-xs">{{ JSON.stringify(messages.get(topicName), null, 2) }}</pre>
                  </div>
                  <div v-else>No message yet</div>
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import { useROS } from '../composables/useRos.js';

const rosIp = ref('localhost'); // Default ROS IP
const rosPort = ref(9090); // Default ROS Port

const {
  ros,
  loading,
  status,
  message,
  topics,
  nodes,
  messages,
  initializeROS,
  updateTopics,
  updateNodes,
  subscribeToTopic,
  unsubscribeFromTopic,
} = useROS();

let updateInterval = null;

onMounted(() => {
  initializeROS(rosIp.value, rosPort.value);

  updateInterval = setInterval(() => {
    if (status.value === 'Connected') {
      updateTopics();
      updateNodes();
    }
  }, 1000); // Update every 1 second
});

onUnmounted(() => {
  if (updateInterval) {
    clearInterval(updateInterval);
  }
  // Unsubscribe from all topics when component is unmounted
  topics.value.forEach((type, name) => unsubscribeFromTopic(name));
});

// Watch for changes in topics and subscribe to new ones
watch(topics, (newTopics, oldTopics) => {
  newTopics.forEach((type, name) => {
    if (!oldTopics.has(name)) {
      subscribeToTopic(name, type);
    }
  });
  // Unsubscribe from topics that are no longer present
  oldTopics.forEach((type, name) => {
    if (!newTopics.has(name)) {
      unsubscribeFromTopic(name);
    }
  });
}, { deep: true });


// This function renders a nested list recursively for messages
// Using JSON.stringify for now for simplicity in display
const renderMessage = (data) => {
  return data; // The pre tag will handle formatting
};
</script>
