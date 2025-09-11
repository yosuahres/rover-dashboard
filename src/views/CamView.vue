<template>
  <div class="p-4 flex">
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-1 flex-grow">
      <VideoStreamCard
        cardTitle="Camera 1"
        cardId="camera1"
        :isSelected="selectedCardId === 'camera1'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 2"
        cardId="camera2"
        :isSelected="selectedCardId === 'camera2'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 3"
        cardId="camera3"
        :isSelected="selectedCardId === 'camera3'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 4"
        cardId="camera4"
        :isSelected="selectedCardId === 'camera4'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 5"
        cardId="camera5"
        :isSelected="selectedCardId === 'camera5'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 6"
        cardId="camera6"
        :isSelected="selectedCardId === 'camera6'"
        @card-selected="handleCardSelected"
      />
      <VideoStreamCard
        cardTitle="Camera 7"
        cardId="camera7"
        :isSelected="selectedCardId === 'camera7'"
        @card-selected="handleCardSelected"
      />
      <!-- !TOBECHANGE more to go if needed -->
    </div>
    <div v-if="selectedCardId" class="w-80 ml-4 p-4 border border-gray-300 rounded-lg shadow-md">
      <StreamController :selectedCardId="selectedCardId" @close-controller="handleCloseController" />
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue';
import VideoStreamCard from '../components/VideoStreamCard.vue';
import StreamController from '../components/StreamController.vue';

const selectedCardId = ref(null);

const handleCardSelected = (cardId) => {
  selectedCardId.value = cardId;
};

const handleCloseController = () => {
  selectedCardId.value = null;
};
</script>
