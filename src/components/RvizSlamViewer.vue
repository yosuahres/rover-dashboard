<template>
  <div ref="viewerContainer" class="rviz-viewer"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import * as ROS3D from 'ros3d';
import { useROS } from '../composables/useRos';

const viewerContainer = ref(null);
let viewer = null;
let tfClient = null;
let grid = null;
let occupancyGridClient = null;
let pointCloud2 = null;

const { ros, isConnected } = useROS();

const initRviz = () => {
  if (viewerContainer.value && ros.value && isConnected.value) {
    // Initialize the 3D viewer
    viewer = new ROS3D.Viewer({
      divID: viewerContainer.value.id,
      width: viewerContainer.value.clientWidth,
      height: viewerContainer.value.clientHeight,
      antialias: true,
      background: '#333333',
    });

    // Initialize TFClient
    tfClient = new ROS3D.TFClient({
      ros: ros.value,
      fixedFrame: '/odom',
      angularThresh: 0.01,
      transThresh: 0.01,
      rate: 10.0,
    });

    // Add a grid to the viewer
    grid = new ROS3D.Grid({
      ros: ros.value,
      tfClient: tfClient, 
      size: 10,
      cellSize: 1,
      lineWidth: 1,
      color: 0xcccccc,
    });
    viewer.addObject(grid);

    // Add an OccupancyGrid display for SLAM maps
    occupancyGridClient = new ROS3D.OccupancyGrid({
      ros: ros.value,
      tfClient: tfClient,
      topic: '/map',
      color: 0x0062ff, 
      opacity: 0.7,
    });
    viewer.addObject(occupancyGridClient);

    // Add a PointCloud2 display for lidar data
    pointCloud2 = new ROS3D.PointCloud2({
      ros: ros.value,
      tfClient: tfClient,
      topic: '/camera/depth/points', // !CHANGEDTHIS
      material: { size: 0.05, color: 0xff00ff },
    });
    viewer.addObject(pointCloud2);

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
    viewer.destroy();
    viewer = null;
  }
  if (tfClient) {
    tfClient.unsubscribe();
    tfClient = null;
  }
  if (grid) {
    grid = null;
  }
  if (occupancyGridClient) {
    occupancyGridClient = null;
  }
  if (pointCloud2) {
    pointCloud2 = null;
  }
  window.removeEventListener('resize', resizeViewer);
};

onMounted(() => {
  // Ensure the container has an ID for ROS3D.Viewer
  if (viewerContainer.value) {
    viewerContainer.value.id = 'rviz-slam-viewer-container';
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
