<template>
  <div ref="viewerContainer" class="rviz-viewer"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import * as ROS3D from 'ros3d';
import * as THREE from 'three'; // Added THREE import
import * as ROSLIB from 'roslib'; // Added ROSLIB import
import { useROS } from '../composables/useRos';

const viewerContainer = ref(null);
let viewer = null;
let tfClient = null;
let grid = null;
let urdfClient = null;
let occupancyGridClient = null; // Added occupancyGridClient
let mapTopic = null; // Added mapTopic for debugging

const { ros, isConnected } = useROS();

const initRviz = () => {
  if (viewerContainer.value && ros.value && isConnected.value) {
    // Initialize the 3D viewer
    viewer = new ROS3D.Viewer({
      divID: viewerContainer.value.id,
      width: viewerContainer.value.clientWidth,
      height: viewerContainer.value.clientHeight,
      antialias: true,
      background: '#cccccc', 
      fixedFrame: 'odom' //reference frame
    });

    // Add a grid to the viewer
    viewer.addObject(new ROS3D.Grid({ 
      color:'#0181c4', 
      cellSize: 0.5, 
      num_cells: 20
    }));

    // Initialize TFClient
    tfClient = new ROSLIB.TFClient({ 
      ros: ros.value,
      angularThres: 0.01, 
      transThres: 0.01, 
      rate: 10.0,
    });

    // Setup the URDF client.
    urdfClient = new ROS3D.UrdfClient({ 
      ros: ros.value,
      param: 'robot_description', 
      tfClient: tfClient,
      path: location.origin + location.pathname, 
      rootObject: viewer.scene, 
      loader: ROS3D.COLLADA_LOADER_2 
    });

    // Setup the OccupancyGrid client.
    occupancyGridClient = new ROS3D.OccupancyGrid({
      ros: ros.value,
      topic: '/map',
      rootObject: viewer.scene,
      continuous: true
    });

    // Debugging: Add a ROSLIB.Topic listener for /map
    mapTopic = new ROSLIB.Topic({
      ros: ros.value,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    });

    mapTopic.subscribe((message) => {
      console.log('Received /map message:', message);
      if (!message || !message.info) {
        console.error('Received /map message is undefined or missing info property:', message);
      }
    });

    // Handle window resize
    window.addEventListener('resize', resizeViewer);
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
  if (tfClient) {
    tfClient.unsubscribe();
    tfClient = null;
  }
  if (grid) {
    grid = null;
  }
  if (urdfClient) {
    urdfClient = null;
  }
  if (occupancyGridClient) {
    occupancyGridClient = null;
  }
  if (mapTopic) {
    mapTopic.unsubscribe();
    mapTopic = null;
  }
  window.removeEventListener('resize', resizeViewer);
};

onMounted(() => {
  // Ensure the container has an ID for ROS3D.Viewer
  if (viewerContainer.value) {
    viewerContainer.value.id = 'rviz-viewer-container';
  }
  watch([ros, isConnected], ([newRos, newIsConnected]) => {
    if (newRos && newIsConnected && !viewer) {
      initRviz();
    } else if ((!newRos || !newIsConnected) && viewer) {
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
