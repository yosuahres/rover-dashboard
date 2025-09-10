<template>
  <div ref="viewerContainer" class="rviz-viewer"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import { useROS } from '../composables/useRos';
import * as ROS2D from '@ken20020209/ros2d';

const viewerContainer = ref(null);
let mapViewer = null;
let mapGridClient = null;

const { ros, isConnected } = useROS();

const initRviz = () => {
  if (viewerContainer.value && ros.value && isConnected.value) {
    // Initialize the 2D viewer
    mapViewer = new ROS2D.Viewer({
      divID: viewerContainer.value.id,
      width: viewerContainer.value.clientWidth,
      height: viewerContainer.value.clientHeight,
    });

    // Setup the map client.
    mapGridClient = new ROS2D.OccupancyGridClient({
      ros: ros.value,
      rootObject: mapViewer.scene,
      continuous: true,
    });

    // Scale the canvas to fit to the map
    mapGridClient.on('change', () => {
      mapViewer.scaleToDimensions(mapGridClient.currentGrid.width, mapGridClient.currentGrid.height);
      mapViewer.shift(mapGridClient.currentGrid.pose.position.x, mapGridClient.currentGrid.pose.position.y);
    });

    // Handle window resize
    window.addEventListener('resize', resizeViewer);
  }
};

const resizeViewer = () => {
  if (mapViewer && viewerContainer.value) {
    mapViewer.resize(viewerContainer.value.clientWidth, viewerContainer.value.clientHeight);
  }
};

const destroyRviz = () => {
  if (mapViewer) {
    document.getElementById(viewerContainer.value.id).innerHTML = ''; // Clears the map div
    mapViewer = null;
  }
  if (mapGridClient) {
    mapGridClient.unsubscribe();
    mapGridClient = null;
  }
  window.removeEventListener('resize', resizeViewer);
};

onMounted(() => {
  // Ensure the container has an ID for ROS2D.Viewer
  if (viewerContainer.value) {
    viewerContainer.value.id = 'map';
  }
  watch([ros, isConnected], ([newRos, newIsConnected]) => {
    if (newRos && newIsConnected && !mapViewer) {
      initRviz();
    } else if ((!newRos || !newIsConnected) && mapViewer) {
      destroyRviz();
    }
  }, { immediate: true });
});

onUnmounted(() => {
  destroyRviz();
});
</script>

<style scoped>
.rviz-viewer {
  width: 100%;
  height: 100%;
  background-color: #333333; 
}
</style>
