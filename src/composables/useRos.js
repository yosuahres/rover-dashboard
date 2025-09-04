import { readonly } from 'vue';
import ROSLIB from 'roslib';
import { useMainStore } from '../stores/store';

export function useROS() {
  const mainStore = useMainStore();

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

    // Exclude image topics as per user's example
    if (topicType === 'sensor_msgs/msg/Image' || topicType === 'sensor_msgs/msg/CompressedImage' || topicType === 'theora_image_transport/msg/Packet') {
      return;
    }

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

  return {
    ros: readonly(mainStore.ros),
    loading: readonly(mainStore.loading),
    server: readonly(mainStore.server),
    status: readonly(mainStore.status),
    message: readonly(mainStore.message),
    topics: readonly(mainStore.topics),
    nodes: readonly(mainStore.nodes),
    messages: readonly(mainStore.messages),
    isConnected: readonly(mainStore.isConnected),
    initializeROS,
    updateTopics,
    updateNodes,
    subscribeToTopic,
    unsubscribeFromTopic,
  };
}
