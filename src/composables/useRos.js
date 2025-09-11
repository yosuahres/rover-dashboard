import { ref, readonly, computed, watch } from 'vue'; // Added ref and watch
import ROSLIB from 'roslib';
import { useMainStore } from '../stores/store';

export function useROS() {
  const mainStore = useMainStore();
  const rosRef = ref(null); // Local ref for ROS instance

  // Watch for changes in mainStore.ros and update rosRef
  watch(() => mainStore.ros, (newRos) => {
    rosRef.value = newRos;
  }, { immediate: true });

  function initializeROS(ip, port) {
    const url = `ws://${ip}:${port}`;
    mainStore.setLoading(true);
    mainStore.setServer(ip);

    const rosConnection = new ROSLIB.Ros({ url });

    mainStore.setRos(rosConnection);

    rosConnection.on('connection', () => {
      mainStore.setMessage(`Connected to ROS master: ${url}`);
      mainStore.setLoading(false);
      mainStore.setStatus('Connected');
      initializeRosTopics(rosConnection);
    });

    rosConnection.on('error', (error) => {
      mainStore.setMessage(`Error connecting to ROS: ${url} - ${error.message || error}`);
      mainStore.setLoading(false);
      mainStore.setStatus('Disconnected');
      console.error('ROS connection error:', error);
    });

    rosConnection.on('close', () => {
      console.log('ROS connection closed');
      mainStore.setMessage(`Closed connection to ROS master: ${url}`);
      mainStore.setLoading(false);
      mainStore.clearAllRosData();
    });
  }

  function initializeRosTopics(rosInstance) {
    if (!rosInstance) {
      console.warn("ROS instance not provided for topic initialization.");
      return;
    }

    // Publishers
    const topicConfiguration = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/web/config/configuration',
      messageType: 'std_msgs/Float32MultiArray'
    });
    mainStore.setTopicConfiguration(topicConfiguration);

    const topicVelocityAndSteering = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/master/ui_target_velocity_and_steering',
      messageType: 'std_msgs/Float32MultiArray'
    });
    mainStore.setTopicVelocityAndSteering(topicVelocityAndSteering);


    // Subscribers
    const configListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/web/config/configuration_init',
      messageType: 'std_msgs/Float32MultiArray'
    });
    mainStore.setConfigListener(configListener);

    const robotVelSubscriber = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/motor_main/velocity_feedback',
      messageType: 'std_msgs/Float32'
    });
    mainStore.setRobotVelSubscriber(robotVelSubscriber);

    const robotVelInfoSubscriber = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/master/target_speed',
      messageType: 'std_msgs/Float32'
    });
    mainStore.setRobotVelInfoSubscriber(robotVelInfoSubscriber);

    const robotSteeringSubscriber = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/master/target_steering',
      messageType: 'std_msgs/Float32'
    });
    mainStore.setRobotSteeringSubscriber(robotSteeringSubscriber);

    // Request initial config
    const reqConfig = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/web/config/request_config',
      messageType: 'std_msgs/Int16'
    });
    reqConfig.publish({});
  }

  // Helper to compare two Maps so we only update if topics truly changed.
  const areMapsEqual = (a, b) => {
    if (a.size !== b.size) return false;
    for (const [key, val] of a.entries()) {
      if (!b.has(key) || b.get(key) !== val) return false;
    }
    return true;
  };

  function updateTopics() {
    if (mainStore.ros && mainStore.isConnected) {
      mainStore.ros.getTopics((result) => {
        const newMap = new Map();
        for (let i = 0; i < result.topics.length; i++) {
          const topicName = result.topics[i];
          // Ignore /rosout if desired
          if (topicName === '/rosout') continue;
          newMap.set(topicName, result.types[i]);
        }

        if (!areMapsEqual(mainStore.topics, newMap)) {
          mainStore.setTopics(newMap);
          mainStore.setMessage(`Found ${mainStore.topics.size} topics.`);
        }
      }, (error) => {
        mainStore.setMessage(`Error listing topics: ${error.message || error}`);
        console.error('Error listing topics:', error);
      });
    } else {
      mainStore.setMessage('ROS not connected. Cannot list topics.');
    }
  }

  function updateNodes() {
    if (mainStore.ros && mainStore.isConnected) {
      mainStore.ros.getNodes((result) => {
        if (JSON.stringify(mainStore.nodes) !== JSON.stringify(result)) {
          mainStore.setNodes(result);
          mainStore.setMessage(`Found ${mainStore.nodes.length} nodes.`);
        }
      }, (error) => {
        mainStore.setMessage(`Error listing nodes: ${error.message || error}`);
        console.error('Error listing nodes:', error);
      });
    } else {
      mainStore.setMessage('ROS not connected. Cannot list nodes.');
    }
  }

  function subscribeToTopic(topicName, topicType) {
    if (!mainStore.ros || !mainStore.isConnected) {
      mainStore.setMessage('ROS not connected. Cannot subscribe.');
      return;
    }
    if (mainStore.subscriptions.has(topicName)) {
      // Already subscribed
      return;
    }

    // if wanted to ignore image topics
    // if (topicType === 'sensor_msgs/msg/Image' || topicType === 'sensor_msgs/msg/CompressedImage' || topicType === 'theora_image_transport/msg/Packet') {
    //   return;
    // }

    const subscriber = new ROSLIB.Topic({
      ros: mainStore.ros,
      name: topicName,
      messageType: topicType,
      throttle_rate: 100,
    });

    subscriber.subscribe((msg) => {
      mainStore.setMessages(topicName, msg);
    });

    mainStore.addSubscription(topicName, subscriber);
    mainStore.setMessage(`Subscribed to topic: ${topicName}`);
  }

  function unsubscribeFromTopic(topicName) {
    mainStore.removeSubscription(topicName);
    mainStore.setMessage(`Unsubscribed from topic: ${topicName}`);
  }

  function setRosParameter(nodeName, paramName, paramValue) {
    if (!mainStore.ros || !mainStore.isConnected) {
      console.error("ROS not connected. Cannot set parameter.");
      return;
    }

    const setParamClient = new ROSLIB.Service({
      ros: mainStore.ros,
      name: '/rosapi/set_param',
      serviceType: 'rosapi/SetParam'
    });

    const request = new ROSLIB.ServiceRequest({
      name: `/${nodeName}/${paramName}`,
      value: JSON.stringify(paramValue)
    });

    setParamClient.call(request, function(result) {
      console.log(`Parameter ${paramName} set to ${paramValue}:`, result);
      mainStore.setMessage(`Parameter ${paramName} set to ${paramValue}`);
    }, function(error) {
      console.error(`Error setting parameter ${paramName}:`, error);
      mainStore.setMessage(`Error setting parameter ${paramName}: ${error.message || error}`);
    });
  }

  return {
    ros: readonly(rosRef), // Return the local reactive ref
    loading: readonly(mainStore.loading),
    server: readonly(mainStore.server),
    status: readonly(mainStore.status),
    message: readonly(mainStore.message),
    topics: readonly(mainStore.topics),
    nodes: readonly(mainStore.nodes),
    messages: readonly(mainStore.messages),
    isConnected: computed(() => mainStore.isConnected),
    initializeROS,
    updateTopics,
    updateNodes,
    subscribeToTopic,
    unsubscribeFromTopic,
    setRosParameter, // Expose this function
    initializeRosTopics, // Expose this function
  };
}
