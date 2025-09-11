<template>
  <div class="min-h-screen flex items-center justify-center bg-gray-100 p-4">
    <div class="bg-white p-8 rounded-lg shadow-md w-full max-w-md">
      <h2 class="text-2xl font-bold mb-6 text-center text-gray-800">ROS Connection</h2>

      <form @submit.prevent="connectRos">
        <div class="mb-4">
          <label for="ip" class="block text-gray-700 text-sm font-bold mb-2">
            ROS Master IP:
          </label>
          <input
            type="text"
            id="ip"
            v-model="ip"
            @input="handleIpChange"
            required
            class="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
          />
        </div>
        <div class="mb-6">
          <label for="port" class="block text-gray-700 text-sm font-bold mb-2">
            Port:
          </label>
          <input
            type="text"
            id="port"
            v-model="port"
            @input="handlePortChange"
            placeholder="Enter ROS Master Server Port"
            required
            class="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
          />
        </div>
        <button
          type="submit"
          :disabled="mainStore.loading"
          class="!bg-green-500 !hover:bg-green-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline w-full disabled:opacity-50"
        >
          {{ mainStore.loading ? 'Connecting...' : 'Connect' }}
        </button>
        <div v-if="mainStore.status === null && !mainStore.loading" class="mt-4 text-center text-sm text-gray-600">
          <!-- Initial state, no message -->
        </div>
        <div v-else-if="mainStore.loading" class="mt-4 text-center text-sm text-blue-600">
          Loading...
        </div>
        <div v-else :class="{'text-green-600': mainStore.status === 'Connected', 'text-red-600': mainStore.status === 'Disconnected'}" class="mt-4 text-center text-sm">
          {{ mainStore.message }}
        </div>
      </form>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, watch } from 'vue';
import { useRouter } from 'vue-router';
import { useMainStore } from '../stores/store';
import { useROS } from '../composables/useRos';

const ip = ref<string>('');
const router = useRouter();
const port = ref<string>('9090');

const mainStore = useMainStore();
const { initializeROS, isConnected } = useROS(); 

onMounted(() => {
  const storedIp = localStorage.getItem('ip');
  const storedPort = localStorage.getItem('port');
  if (storedIp) ip.value = storedIp;
  if (storedPort) port.value = storedPort;
});

const handleIpChange = (e: Event) => {
  ip.value = (e.target as HTMLInputElement).value;
};

const handlePortChange = (e: Event) => {
  port.value = (e.target as HTMLInputElement).value;
};


const connectRos = () => {
  // Only close if connected
  if (mainStore.ros && mainStore.status === 'Connected') {
    mainStore.ros.close();
    console.log('ROS connection closed');
  }

  localStorage.setItem('ip', ip.value);
  localStorage.setItem('port', port.value);

  initializeROS(ip.value, port.value);
};

watch(isConnected, (newStatus) => {
  if (newStatus) { // isConnected is a boolean, so true means connected
    router.push('/data');
  }
});
</script>
