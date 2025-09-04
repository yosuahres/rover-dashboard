<template>
  <div class="p-4">
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
      <VideoStreamCard cardTitle="Camera 1" />
      <VideoStreamCard cardTitle="Camera 2" />
      <VideoStreamCard cardTitle="Camera 3" />
      <VideoStreamCard cardTitle="Camera 4" />
      <VideoStreamCard cardTitle="Camera 5" />
      <VideoStreamCard cardTitle="Camera 6" />
    </div>
  </div>
</template>

<script setup>
import VideoStreamCard from '../components/VideoStreamCard.vue';
</script>
