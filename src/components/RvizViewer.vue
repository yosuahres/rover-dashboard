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
let urdfClient = null;

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
  if (urdfClient) { // Changed to urdfClient
    urdfClient = null;
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
