<template>
  <div
    :class="['border', 'rounded-lg', 'shadow-md', 'p-4', 'flex', 'flex-col', 'h-full', 'cursor-pointer', { 'border-indigo-500 ring-2 ring-indigo-500': isSelected, 'border-gray-300': !isSelected }]"
    @click="selectCard"
  >
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
        class="block text-black w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-indigo-500 focus:border-indigo-500 sm:text-sm"
      />
    </div>

    <div :class="['flex-grow', 'relative', 'bg-gray-100', 'rounded-md', 'overflow-hidden', 'flex', 'items-center', 'justify-center', { 'h-full': !showInputForm }]">
      <img
        v-if="displayMode === 'image' && frameSrc"
        :src="frameSrc"
        :alt="cardTitle + ' Video Stream'"
        class="max-w-full max-h-full object-contain"
      />
      <canvas
        v-else-if="displayMode === 'canvas'"
        ref="canvasEl"
        class="max-w-full max-h-full"
      />
      <p v-else class="p-4 text-gray-500 text-center">
        {{ frameError || 'No video stream available or topic not set.' }}
      </p>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, defineProps, getCurrentInstance, watchEffect, nextTick } from 'vue';
import { useMainStore } from '../stores/store.js';

const props = defineProps({
  cardTitle: {
    type: String,
    required: true,
  },
  cardId: {
    type: String,
    required: true,
  },
  isSelected: {
    type: Boolean,
    default: false,
  },
});

const emit = defineEmits(['card-selected']);

const mainStore = useMainStore();
const { uid: _uid } = getCurrentInstance(); // For unique IDs in template

const bytesToBase64 = (bytes) => {
  if (!bytes?.length) {
    return '';
  }
  const chunkSize = 0x8000;
  let binary = '';
  for (let offset = 0; offset < bytes.length; offset += chunkSize) {
    const chunk = bytes.slice(offset, offset + chunkSize);
    binary += String.fromCharCode(...chunk);
  }
  return btoa(binary);
};

const currentTopic = computed({
  get() {
    return mainStore.cameraTopics[props.cardId] ?? '';
  },
  set(value) {
    mainStore.setCameraTopic(props.cardId, value?.trim() ?? '');
  },
});
const showInputForm = ref(true);
const frameSrc = ref(null);
const frameError = ref(null);
const displayMode = ref('idle');
const canvasEl = ref(null);

const toggleInputForm = () => {
  showInputForm.value = !showInputForm.value;
};

const selectCard = () => {
  emit('card-selected', props.cardId);
};

const base64ToUint8Array = (base64) => {
  const binary = atob(base64);
  const len = binary.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
};

const drawRawImage = (message) => {
  const canvas = canvasEl.value;
  if (!canvas) {
    frameError.value = 'Canvas belum siap.';
    displayMode.value = 'error';
    return;
  }

  const { width, height } = message;
  if (!width || !height) {
    frameError.value = 'Dimensi gambar tidak valid.';
    displayMode.value = 'error';
    return;
  }

  let dataArray;
  if (typeof message.data === 'string') {
    if (!message.data) {
      frameError.value = 'Data gambar kosong.';
      displayMode.value = 'error';
      return;
    }
    dataArray = base64ToUint8Array(message.data);
  } else if (Array.isArray(message.data)) {
    dataArray = new Uint8Array(message.data);
  } else {
    frameError.value = 'Format data gambar tidak dikenal.';
    displayMode.value = 'error';
    return;
  }

  const encoding = (message.encoding || '').toLowerCase();
  const expectedLength = message.step ? message.step * height : dataArray.length;
  if (expectedLength && dataArray.length < expectedLength) {
    frameError.value = 'Data gambar tidak lengkap.';
    displayMode.value = 'error';
    return;
  }

  const ctx = canvas.getContext('2d');
  if (!ctx) {
    frameError.value = 'Gagal mendapatkan konteks canvas.';
    displayMode.value = 'error';
    return;
  }

  canvas.width = width;
  canvas.height = height;

  const imageData = ctx.createImageData(width, height);
  const dest = imageData.data;

  if (encoding === 'rgb8') {
    for (let i = 0, j = 0; i + 2 < dataArray.length && j + 3 < dest.length; i += 3, j += 4) {
      dest[j] = dataArray[i];
      dest[j + 1] = dataArray[i + 1];
      dest[j + 2] = dataArray[i + 2];
      dest[j + 3] = 255;
    }
  } else if (encoding === 'bgr8') {
    for (let i = 0, j = 0; i + 2 < dataArray.length && j + 3 < dest.length; i += 3, j += 4) {
      dest[j] = dataArray[i + 2];
      dest[j + 1] = dataArray[i + 1];
      dest[j + 2] = dataArray[i];
      dest[j + 3] = 255;
    }
  } else if (encoding === 'mono8' || encoding === '8uc1') {
    for (let i = 0, j = 0; i < dataArray.length && j + 3 < dest.length; i += 1, j += 4) {
      const value = dataArray[i];
      dest[j] = value;
      dest[j + 1] = value;
      dest[j + 2] = value;
      dest[j + 3] = 255;
    }
  } else {
    frameError.value = `Encoding ${message.encoding} belum didukung.`;
    displayMode.value = 'error';
    return;
  }

  ctx.putImageData(imageData, 0, 0);
};

watchEffect(() => {
  frameError.value = null;
  frameSrc.value = null;
  displayMode.value = 'idle';

  const topic = currentTopic.value;
  if (!topic) {
    return;
  }

  const topicType = mainStore.topics.get(topic);
  const message = mainStore.messages.get(topic);

  if (!topicType || !message) {
    return;
  }

  if (topicType === 'sensor_msgs/msg/CompressedImage' || topicType === 'sensor_msgs/CompressedImage') {
    const format = (message.format || 'jpeg').toLowerCase();
    const mimeType = format.includes('png') ? 'image/png' : 'image/jpeg';

    if (typeof message.data === 'string' && message.data.length > 0) {
      frameSrc.value = `data:${mimeType};base64,${message.data}`;
      displayMode.value = 'image';
      return;
    }
    if (Array.isArray(message.data) && message.data.length > 0) {
      frameSrc.value = `data:${mimeType};base64,${bytesToBase64(message.data)}`;
      displayMode.value = 'image';
      return;
    }
    frameError.value = 'Compressed image payload kosong.';
    return;
  }

  if (topicType === 'sensor_msgs/msg/Image' || topicType === 'sensor_msgs/Image') {
    displayMode.value = 'canvas';
    nextTick(() => drawRawImage(message));
    return;
  }

  frameError.value = `Encoding ${topicType} belum didukung.`;
});
</script>
