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
let urdf = null;

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

    // Initialize TFClient
    tfClient = new ROS3D.TFClient({
      ros: ros.value,
      fixedFrame: '/odom', //!CHANGEDTHIS
      angularThresh: 0.01,
      transThresh: 0.01,
      rate: 10.0,
    });

    // Add a URDF model to the viewer
    urdf = new ROS3D.UrdfModel({
      ros: ros.value,
      tfClient: tfClient,
      path: `http://${ros.value.url.split('ws://')[1].split(':')[0]}:9090/urdf/`, 
      color: 0x00ff00,
      opacity: 1.0,
      collision: false,
    });
    viewer.addObject(urdf);

    // Add an OccupancyGrid display for SLAM maps
    const occupancyGridClient = new ROS3D.OccupancyGrid({
      ros: ros.value,
      tfClient: tfClient,
      topic: '/map', // !CHANGEDTHIS
      color: 0x0062ff, 
      opacity: 0.7,
    });
    viewer.addObject(occupancyGridClient);

    // Example: Add a PointCloud2 display (uncomment and configure if needed)
    // const pointCloud2 = new ROS3D.PointCloud2({
    //   ros: ros.value,
    //   tfClient: tfClient,
    //   topic: '/camera/depth/points', // Replace with your point cloud topic
    //   material: { size: 0.05, color: 0xff00ff },
    // });
    // viewer.addObject(pointCloud2);

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
  if (urdf) {
    urdf = null;
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
