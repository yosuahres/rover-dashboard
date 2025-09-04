<template>
  <div class="border border-gray-300 rounded-lg shadow-md p-4 flex flex-col h-full">
    <div class="flex justify-between items-center mb-3">
      <h2 class="text-lg font-semibold text-gray-800">{{ cardTitle }}</h2>
      <button @click="toggleInputForm" class="p-1 rounded-full hover:bg-gray-200 focus:outline-none focus:ring-2 focus:ring-indigo-500">
        <svg v-if="showInputForm" xmlns="http://www.w3.org/2000/svg" class="h-5 w-5 text-gray-600" viewBox="0 0 20 20" fill="currentColor">
          <path fill-rule="evenodd" d="M14.707 12.707a1 1 0 01-1.414 0L10 9.414l-3.293 3.293a1 1 0 01-1.414-1.414l4-4a1 1 0 011.414 0l4 4a1 1 0 010 1.414z" clip-rule="evenodd" />
        </svg>
        <svg v-else xmlns="http://www.w3.org/2000/svg" class="h-5 w-5 text-gray-600" viewBox="0 0 20 20" fill="currentColor">
          <path fill-rule="evenodd" d="M5.293 7.293a1 1 0 011.414 0L10 10.586l3.293-3.293a1 1 0 111.414 1.414l-4 4a1 1 0 01-1.414 0l-4-4a1 1 0 010-1.414z" clip-rule="evenodd" />
        </svg>
      </button>
    </div>

    <div v-if="showInputForm" class="mb-4">
      <label :for="`video-topic-input-${_uid}`" class="block text-sm font-medium text-gray-700 mb-1">Topic:</label>
      <input
        type="text"
        :id="`video-topic-input-${_uid}`"
        v-model="currentTopic"
        placeholder="/camera/image_raw"
        class="block w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm"
      />
    </div>

    <div :class="['flex-grow', 'relative', 'bg-gray-100', 'rounded-md', 'overflow-hidden', 'flex', 'items-center', 'justify-center', { 'h-full': !showInputForm }]">
      <img v-if="videoStreamUrl" :src="videoStreamUrl" :alt="cardTitle + ' Video Stream'" class="w-full h-full object-contain" />
      <p v-else class="p-4 text-gray-500 text-center">No video stream available or topic not set.</p>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, defineProps, getCurrentInstance } from 'vue';
import { useROS } from '../composables/useRos.js';

const props = defineProps({
  cardTitle: {
    type: String,
    required: true,
  },
});

const { server } = useROS();
const { uid: _uid } = getCurrentInstance(); // For unique IDs in template

const currentTopic = ref('');
const showInputForm = ref(true);

const toggleInputForm = () => {
  showInputForm.value = !showInputForm.value;
};

const videoStreamUrl = computed(() => {
  if (currentTopic.value && server.value) {
    return `http://${server.value}:8080/stream?topic=${currentTopic.value}`;
  }
  return null;
});
</script>

