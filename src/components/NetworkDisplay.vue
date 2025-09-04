<template>
  <div class="flex items-center space-x-2">
    <span class="text-gray-400">Ping:</span>
    <span :class="pingClass">
      {{ typeof ping === 'number' ? `${ping}ms` : ping }}
    </span>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, computed } from 'vue';
import { useMainStore } from '../stores/store';

const mainStore = useMainStore();
const ping = ref('N/A');
const BAD_PING_THRESHOLD = 150; // Threshold for a "bad" ping in ms
let intervalId;

const pingClass = computed(() => {
  if (typeof ping.value !== 'number') {
    return 'text-red-500'; 
  }
  return ping.value > BAD_PING_THRESHOLD ? 'text-yellow-500' : 'text-green-500'; 
});

onMounted(() => {
  intervalId = setInterval(async () => {
    if (mainStore.server) {
      const startTime = performance.now();
      try {
        // Use a HEAD request for minimal data transfer, or GET if HEAD is not supported by the ROS server
        await fetch(`http://${mainStore.server}`, { method: 'HEAD', mode: 'no-cors' });
        const endTime = performance.now();
        ping.value = Math.round(endTime - startTime);
      } catch (error) {
        console.error('Ping failed:', error);
        ping.value = 'Error';
      }
    } else {
      ping.value = 'N/A';
    }
  }, 1000);
});

onUnmounted(() => {
  if (intervalId) {
    clearInterval(intervalId);
  }
});
</script>
