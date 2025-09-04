import { defineStore } from 'pinia';
import ROSLIB from 'roslib';

export const useMainStore = defineStore('main', {
  state: () => ({
    ros: null,
    loading: false,
    status: null, // 'Connected' | 'Disconnected' | null
    message: null,
    server: '',
    topics: new Map(), // Map<string, string> for topicName -> topicType
    nodes: [], // string[] for node names
    messages: new Map(), // Map<string, any> for topicName -> latestMessage
    subscriptions: new Map(), // Map<string, ROSLIB.Topic> for topicName -> subscriber
  }),
  getters: {
    isConnected: (state) => state.status === 'Connected',
  },
  actions: {
    setRos(rosInstance) {
      this.ros = rosInstance;
    },
    setLoading(isLoading) {
      this.loading = isLoading;
    },
    setStatus(newStatus) {
      this.status = newStatus;
    },
    setMessage(newMessage) {
      this.message = newMessage;
    },
    setServer(newServer) {
      this.server = newServer;
    },
    setTopics(newTopics) {
      this.topics = newTopics;
    },
    setNodes(newNodes) {
      this.nodes = newNodes;
    },
    setMessages(topicName, message) {
      this.messages.set(topicName, message);
    },
    addSubscription(topicName, subscriber) {
      this.subscriptions.set(topicName, subscriber);
    },
    removeSubscription(topicName) {
      if (this.subscriptions.has(topicName)) {
        const subscriber = this.subscriptions.get(topicName);
        subscriber.unsubscribe();
        this.subscriptions.delete(topicName);
        this.messages.delete(topicName);
      }
    },
    clearAllRosData() {
      this.subscriptions.forEach(sub => sub.unsubscribe());
      this.subscriptions.clear();
      this.topics.clear();
      this.nodes = [];
      this.messages.clear();
      this.ros = null;
      this.status = 'Disconnected';
      this.loading = false;
    },
  },
});
