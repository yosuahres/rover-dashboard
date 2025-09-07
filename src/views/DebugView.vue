<template>
  <div class="p-4 bg-gray-100 min-h-screen">

    <div class="container mx-auto">
      <div class="grid grid-cols-1 md:grid-cols-4 gap-4">

        <!-- Robot Info Panel -->
        <div class="md:col-span-1">
          <div class="bg-white shadow-md rounded-lg p-6" id="robot-info-box">
            <h2 class="text-xl text-black font-semibold mb-4">Robot Info</h2>
            <div class="space-y-2">
              <p class="text-black"><strong>Velocity:</strong> <span id="info-velocity">{{ robotVelocity.toFixed(2) }}</span> m/s</p>
              <p class="text-black"><strong>Steering:</strong> <span id="info-steering">{{ robotSteering.toFixed(2) }}</span> rad</p>
              <p class="text-black"><strong>Lookahead:</strong> <span id="info-lookahead">0.0</span> m</p>
            </div>
          </div>
        </div>

        <!-- Configuration Panel -->
        <div class="md:col-span-4">
          <div class="bg-white shadow-md rounded-lg p-6" id="config-panel">
            <h2 class="text-xl font-semibold text-blue-600 mb-4">Configuration Panel</h2>

            <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div v-for="(name, index) in configurationNames" :key="index" class="flex flex-col">
                <label class="block text-gray-700 text-sm font-bold mb-2" :for="`config-${index + 1}`">{{ name }}</label>
                <input :id="`config-${index + 1}`" v-model.number="configurations[index]"
                  class="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline focus:border-blue-500"
                  type="number" min="-100" />
              </div>
            </div>

            <div class="flex space-x-2 mt-6">
              <button @click="onSaveConfiguration" class="px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-600">Apply</button>
              <button @click="onResetConfig" class="px-4 py-2 bg-yellow-500 text-white rounded-md hover:bg-yellow-600">Reset</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue';
import { useMainStore } from '../stores/store';
import { useROS } from '../composables/useRos';
import Chart from 'chart.js/auto';

const mainStore = useMainStore();
const { isConnected } = useROS(); 

// Reactive state for UI elements
const robotVelocity = ref(0.0);
const robotSteering = ref(0.0);
const configurations = ref(Array(19).fill(0)); 

const configurationNames = [
  "k_p_wheel",
  "k_i_wheel", 
  "k_d_wheel", 
  "k_d_steering", 
  "wheel_radius",
  "encoder_ppr", 
  "cnt_to_meter", 
  "max_steering_deg", 
  "min_steering_deg",
  "max_steering_pwm",
   "min_steering_pwm", 
   "mid_steering_pwm",
  "max_wheel_velocity_pwm", 
  "min_wheel_velocity_pwm", 
  "max_wheel_integral_pwm",
  "min_wheel_integral_pwm", 
  "wheel_base", 
  "tuning", 
  "K_model"
];

// Chart instances
let rosChartVelocityInstance = null;
let rosChartSteeringInstance = null;
const dataWindow = 100; // Reduced for better performance in a small chart

// Chart data
const velocityChartData = ref(Array(dataWindow).fill(0));
const steeringChartData = ref(Array(dataWindow).fill(0));
const chartLabels = ref(Array(dataWindow).fill(''));
const t0 = Date.now(); // Record when page loaded (in ms)

// Functions for ROS communication
const setupRosSubscribers = () => {
  if (!mainStore.isConnected || !mainStore.configListener) {
    console.warn("ROS not connected or listeners not initialized.");
    return;
  }

  mainStore.configListener.subscribe(msg => {
    configurations.value = msg.data.slice(0, configurationNames.length);
    console.log("Received configuration from ROS:", msg.data);
  });

  mainStore.robotVelSubscriber.subscribe(msg => {
    const tStep = ((Date.now() - t0) / 1000).toFixed(2);
    velocityChartData.value.push(msg.data);
    velocityChartData.value.shift();
    chartLabels.value.push(tStep);
    chartLabels.value.shift();
    if (rosChartVelocityInstance) {
      rosChartVelocityInstance.data.labels = chartLabels.value;
      rosChartVelocityInstance.data.datasets[0].data = velocityChartData.value;
      rosChartVelocityInstance.update('none');
    }
  });

  mainStore.robotVelInfoSubscriber.subscribe(msg => {
    robotVelocity.value = msg.data;
  });

  mainStore.robotSteeringSubscriber.subscribe(msg => {
    robotSteering.value = msg.data;
    const tStep = ((Date.now() - t0) / 1000).toFixed(2);
    steeringChartData.value.push(msg.data * (180 / Math.PI)); // Convert rad to deg
    steeringChartData.value.shift();
    if (rosChartSteeringInstance) {
      rosChartSteeringInstance.data.labels = chartLabels.value;
      rosChartSteeringInstance.data.datasets[0].data = steeringChartData.value;
      rosChartSteeringInstance.update('none');
    }
  });
};

const cleanupRosSubscribers = () => {
  if (mainStore.configListener) mainStore.configListener.unsubscribe();
  if (mainStore.robotVelSubscriber) mainStore.robotVelSubscriber.unsubscribe();
  if (mainStore.robotSteeringSubscriber) mainStore.robotSteeringSubscriber.unsubscribe();
  if (mainStore.robotVelInfoSubscriber) mainStore.robotVelInfoSubscriber.unsubscribe();
};


const onSaveConfiguration = () => {
  if (mainStore.topicConfiguration) {
    mainStore.topicConfiguration.publish({ data: configurations.value });
    console.log("Published configuration:", configurations.value);
  } else {
    console.warn("Configuration topic not initialized.");
  }
};

const onResetConfig = () => {
  configurations.value = Array(configurationNames.length).fill(0);
  if (mainStore.topicConfiguration) {
    mainStore.topicConfiguration.publish({ data: configurations.value });
    console.log("Reset configuration to zeros");
  } else {
    console.warn("Configuration topic not initialized.");
  }
};

// Keyboard controls for velocity and steering
let uiTargetVelocity = 0;
let uiTargetSteering = 0;

const handleKeyDown = (event) => {
  if (!mainStore.topicVelocityAndSteering) {
    console.warn("Velocity and Steering topic not initialized.");
    return;
  }

  switch (event.key) {
    case 'w': uiTargetVelocity += 0.1; break;
    case 's': uiTargetVelocity -= 0.1; break;
    case 'j': uiTargetVelocity = 2.0; break;
    case 'g': uiTargetVelocity = -1.0; break;
    case 'm': uiTargetSteering -= 0.1; break;
    case 'n': uiTargetSteering = 0.0; break;
    case 'b': uiTargetSteering += 0.1; break;
    case ' ':
      uiTargetVelocity = 0.0;
      uiTargetSteering = 0.0;
      break;
    case 'Enter':
      onSaveConfiguration();
      return; // Don't publish velocity/steering for Enter
  }
  mainStore.topicVelocityAndSteering.publish({ data: [uiTargetVelocity, uiTargetSteering] });
};

// Lifecycle hooks
onMounted(() => {
  // Initialize charts
  const ctxVel = document.getElementById('rosChartVelocity').getContext('2d');
  rosChartVelocityInstance = new Chart(ctxVel, {
    type: 'line',
    data: {
      labels: chartLabels.value,
      datasets: [{
        label: 'Velocity (m/s)',
        data: velocityChartData.value,
        fill: false,
        borderColor: 'rgb(52,152,219)',
        tension: 0.1
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { display: true, title: { display: true, text: 't' } },
        y: {
          min: -0.5, max: 1.5,
          ticks: { font: { size: 12 } },
          grid: { color: '#eee' }
        }
      },
      responsive: true,
      maintainAspectRatio: false,
    }
  });

  const ctxSteer = document.getElementById('rosChartSteering').getContext('2d');
  rosChartSteeringInstance = new Chart(ctxSteer, {
    type: 'line',
    data: {
      labels: chartLabels.value,
      datasets: [{
        label: 'Steering (deg)',
        data: steeringChartData.value,
        fill: false,
        borderColor: 'rgb(219, 52, 52)',
        tension: 0.1,
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { display: true, title: { display: true, text: 't' } },
        y: {
          min: -45, max: 45,
          ticks: { font: { size: 12 } },
          grid: { color: '#eee' }
        }
      },
      responsive: true,
      maintainAspectRatio: false,
    }
  });

  // Add keyboard listener
  window.addEventListener('keydown', handleKeyDown);

  // Watch for ROS connection status
  watch(isConnected, (newVal) => {
    if (newVal) {
      setupRosSubscribers();
    } else {
      cleanupRosSubscribers();
    }
  }, { immediate: true }); // Run immediately to check initial connection
});

onUnmounted(() => {
  cleanupRosSubscribers();

  if (rosChartVelocityInstance) rosChartVelocityInstance.destroy();
  if (rosChartSteeringInstance) rosChartSteeringInstance.destroy();

  window.removeEventListener('keydown', handleKeyDown);
});
</script>

<style scoped>
/* Add any component-specific styles here if needed, though Tailwind should handle most */
</style>
