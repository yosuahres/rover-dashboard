import { ref, readonly, provide, inject } from 'vue';
import ROSLIB from 'roslib';

const ROS_KEY = Symbol('ros');

export function createROS() {
  const ros = ref(null);
  const loading = ref(false);
  const status = ref(null); // 'Connected' | 'Disconnected' | null
  const message = ref(null);
  const server = ref('');
  const topics = ref(new Map()); // Map<string, string> for topicName -> topicType
  const nodes = ref([]); // string[] for node names
  const messages = ref(new Map()); // Map<string, any> for topicName -> latestMessage
  const subscriptions = ref(new Map()); // Map<string, ROSLIB.Topic> for topicName -> subscriber

  function initializeROS(ip, port) {
    const url = `ws://${ip}:${port}`;
    loading.value = true;
    server.value = ip;

    const rosConnection = new ROSLIB.Ros({ url });

    ros.value = rosConnection;

    rosConnection.on('connection', () => {
      message.value = `Connected to ROS master: ${url}`;
      loading.value = false;
      status.value = 'Connected';
    });

    rosConnection.on('error', (error) => {
      message.value = `Error connecting to ROS: ${url} - ${error.message || error}`;
      loading.value = false;
      status.value = 'Disconnected';
      console.error('ROS connection error:', error);
    });

    rosConnection.on('close', () => {
      console.log('ROS connection closed');
      message.value = `Closed connection to ROS master: ${url}`;
      loading.value = false;
      status.value = 'Disconnected';
      // Clear all subscriptions and data when connection closes
      subscriptions.value.forEach(sub => sub.unsubscribe());
      subscriptions.value.clear();
      topics.value.clear();
      nodes.value = [];
      messages.value.clear();
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
    if (ros.value && ros.value.isConnected) {
      ros.value.getTopics((result) => {
        const newMap = new Map();
        for (let i = 0; i < result.topics.length; i++) {
          const topicName = result.topics[i];
          // Ignore /rosout if desired
          if (topicName === '/rosout') continue;
          newMap.set(topicName, result.types[i]);
        }

        if (!areMapsEqual(topics.value, newMap)) {
          topics.value = newMap;
          message.value = `Found ${topics.value.size} topics.`;
        }
      }, (error) => {
        message.value = `Error listing topics: ${error.message || error}`;
        console.error('Error listing topics:', error);
      });
    } else {
      message.value = 'ROS not connected. Cannot list topics.';
    }
  }

  function updateNodes() {
    if (ros.value && ros.value.isConnected) {
      ros.value.getNodes((result) => {
        if (JSON.stringify(nodes.value) !== JSON.stringify(result)) {
          nodes.value = result;
          message.value = `Found ${nodes.value.length} nodes.`;
        }
      }, (error) => {
        message.value = `Error listing nodes: ${error.message || error}`;
        console.error('Error listing nodes:', error);
      });
    } else {
      message.value = 'ROS not connected. Cannot list nodes.';
    }
  }

  function subscribeToTopic(topicName, topicType) {
    if (!ros.value || !ros.value.isConnected) {
      message.value = 'ROS not connected. Cannot subscribe.';
      return;
    }
    if (subscriptions.value.has(topicName)) {
      // Already subscribed
      return;
    }

    // Exclude image topics as per user's example
    if (topicType === 'sensor_msgs/msg/Image' || topicType === 'sensor_msgs/msg/CompressedImage' || topicType === 'theora_image_transport/msg/Packet') {
      return;
    }

    const subscriber = new ROSLIB.Topic({
      ros: ros.value,
      name: topicName,
      messageType: topicType,
      throttle_rate: 100,
    });

    subscriber.subscribe((msg) => {
      messages.value.set(topicName, msg);
    });

    subscriptions.value.set(topicName, subscriber);
    message.value = `Subscribed to topic: ${topicName}`;
  }

  function unsubscribeFromTopic(topicName) {
    if (subscriptions.value.has(topicName)) {
      const subscriber = subscriptions.value.get(topicName);
      subscriber.unsubscribe();
      subscriptions.value.delete(topicName);
      messages.value.delete(topicName);
      message.value = `Unsubscribed from topic: ${topicName}`;
    }
  }

  const rosState = {
    ros: readonly(ros), // Expose the raw ros object for direct calls like getTopics/getNodes
    loading: readonly(loading),
    server: readonly(server),
    status: readonly(status),
    message: readonly(message),
    topics: readonly(topics),
    nodes: readonly(nodes),
    messages: readonly(messages),
    initializeROS,
    updateTopics,
    updateNodes,
    subscribeToTopic,
    unsubscribeFromTopic,
  };

  provide(ROS_KEY, rosState);

  return rosState;
}

export function useROS() {
  const rosState = inject(ROS_KEY);
  if (!rosState) {
    throw new Error('useROS must be used within a createROS setup.');
  }
  return rosState;
}
