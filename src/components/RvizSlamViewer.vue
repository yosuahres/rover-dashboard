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
  console.log('initRviz called');
  if (viewerContainer.value && ros.value && isConnected.value) {
    console.log('Viewer container, ROS, and connection are available.');
    console.log('viewerContainer.value.id:', viewerContainer.value.id);
    console.log('viewerContainer.value.clientWidth:', viewerContainer.value.clientWidth);
    console.log('viewerContainer.value.clientHeight:', viewerContainer.value.clientHeight);

    // Initialize the 2D viewer
    mapViewer = new ROS2D.Viewer({
      divID: viewerContainer.value.id,
      width: viewerContainer.value.clientWidth,
      height: viewerContainer.value.clientHeight,
    });
    console.log('ROS2D.Viewer initialized:', mapViewer);

    // Setup the map client.
    mapGridClient = new ROS2D.OccupancyGridClient({
      ros: ros.value,
      rootObject: mapViewer.scene,
      continuous: true,
      topic: '/map' // Explicitly define the topic
    });
    console.log('ROS2D.OccupancyGridClient initialized:', mapGridClient);

    // Scale the canvas to fit to the map
    mapGridClient.on('change', () => {
      console.log('mapGridClient "change" event fired. Map data received.');
      console.log('currentGrid width:', mapGridClient.currentGrid.width);
      console.log('currentGrid height:', mapGridClient.currentGrid.height);
      console.log('currentGrid pose x:', mapGridClient.currentGrid.pose.position.x);
      console.log('currentGrid pose y:', mapGridClient.currentGrid.pose.position.y);

      mapViewer.scaleToDimensions(mapGridClient.currentGrid.width, mapGridClient.currentGrid.height);
      mapViewer.shift(mapGridClient.currentGrid.pose.position.x, mapGridClient.currentGrid.pose.position.y);
    });

    // Handle window resize
    window.addEventListener('resize', resizeViewer);
  } else {
    console.log('initRviz: Prerequisites not met. viewerContainer:', viewerContainer.value, 'ros:', ros.value, 'isConnected:', isConnected.value);
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
    console.log('Viewer container ID set to:', viewerContainer.value.id);
  }
  watch([ros, isConnected], ([newRos, newIsConnected]) => {
    console.log('ROS or isConnected changed. newRos:', newRos, 'newIsConnected:', newIsConnected, 'mapViewer exists:', !!mapViewer);
    if (newRos && newIsConnected) { // If connected, try to initialize
      if (!mapViewer) { // Only initialize if not already initialized
        console.log('Connection established and mapViewer not initialized. Calling initRviz().');
        initRviz();
      } else {
        console.log('Connection established but mapViewer already exists. Skipping initRviz().');
      }
    } else { // If not connected, destroy
      console.log('Connection lost or not established. Calling destroyRviz().');
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
