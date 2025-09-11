<template>
  <div ref="viewerContainer" class="rviz-viewer"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import { useROS } from '../composables/useRos';
import * as ROS3D from 'ros3d';
import * as THREE from 'three';

const viewerContainer = ref(null);
let viewer = null; // Renamed from mapViewer to viewer for generic 3D viewer
let occupancyGridClient = null; // Renamed from mapGridClient for clarity

const { ros, isConnected } = useROS();

const initRviz = () => {
  console.log('initRviz called for ROS3D');
  if (viewerContainer.value && ros.value && isConnected.value) {
    console.log('Viewer container, ROS, and connection are available for ROS3D.');
    console.log('viewerContainer.value.id:', viewerContainer.value.id);
    console.log('viewerContainer.value.clientWidth:', viewerContainer.value.clientWidth);
    console.log('viewerContainer.value.clientHeight:', viewerContainer.value.clientHeight);

    // Initialize the 3D viewer
    viewer = new ROS3D.Viewer({
      divID: viewerContainer.value.id,
      width: viewerContainer.value.clientWidth,
      height: viewerContainer.value.clientHeight,
      antialias: true,
      background: '#333333', // Set background color
      cameraPose: { x: 3, y: 3, z: 3 } // Initial camera position
    });
    console.log('ROS3D.Viewer initialized:', viewer);

    // Add a grid to the scene
    viewer.scene.add(new THREE.GridHelper(10, 10));

    // Setup the OccupancyGrid client.
    occupancyGridClient = new ROS3D.OccupancyGrid({
      ros: ros.value,
      topic: '/map',
      rootObject: viewer.scene,
      continuous: true
    });
    console.log('ROS3D.OccupancyGrid initialized:', occupancyGridClient);

    // Handle window resize
    window.addEventListener('resize', resizeViewer);
  } else {
    console.log('initRviz (ROS3D): Prerequisites not met. viewerContainer:', viewerContainer.value, 'ros:', ros.value, 'isConnected:', isConnected.value);
  }
};

const resizeViewer = () => {
  if (viewer && viewerContainer.value) {
    viewer.resize(viewerContainer.value.clientWidth, viewerContainer.value.clientHeight);
  }
};

const destroyRviz = () => {
  if (viewer) {
    // Dispose of Three.js resources
    viewer.scene.traverse((object) => {
      if (object.geometry) object.geometry.dispose();
      if (object.material) {
        if (Array.isArray(object.material)) {
          object.material.forEach(material => material.dispose());
        } else {
          object.material.dispose();
        }
      }
      if (object.texture) object.texture.dispose();
    });
    viewer.renderer.dispose();
    document.getElementById(viewerContainer.value.id).innerHTML = ''; // Clears the div
    viewer = null;
  }
  if (occupancyGridClient) {
    // ROS3D.OccupancyGrid doesn't have an explicit unsubscribe, it cleans up with the viewer
    occupancyGridClient = null;
  }
  window.removeEventListener('resize', resizeViewer);
  console.log('ROS3D viewer destroyed.');
};

onMounted(() => {
  // Ensure the container has an ID for ROS3D.Viewer
  if (viewerContainer.value) {
    viewerContainer.value.id = 'map-3d-viewer'; // Changed ID to avoid conflict and be more descriptive
    console.log('Viewer container ID set to:', viewerContainer.value.id);
  }
  watch([ros, isConnected], ([newRos, newIsConnected]) => {
    console.log('ROS or isConnected changed. newRos:', newRos, 'newIsConnected:', newIsConnected, 'viewer exists:', !!viewer);
    if (newRos && newIsConnected) { // If connected, try to initialize
      if (!viewer) { // Only initialize if not already initialized
        console.log('Connection established and viewer not initialized. Calling initRviz().');
        initRviz();
      } else {
        console.log('Connection established but viewer already exists. Skipping initRviz().');
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
