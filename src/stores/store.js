import { defineStore } from 'pinia';

export const useMainStore = defineStore('main', {
  state: () => ({
    // state data
    count: 0,
  }),
  getters: {
    // get data
    doubleCount: (state) => state.count * 2,
  },
  actions: {
    // action output
    increment() {
      this.count++;
    },
  },
});
