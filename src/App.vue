<template>
  <div class="min-h-screen w-full flex flex-col bg-gray-100">
    <nav class="bg-gray-100 p-4 flex items-center justify-between px-6">
      <div class="flex flex-grow justify-evenly mr-8">
        <router-link
          to="/main"
          class="text-lg font-medium text-gray-700 hover:text-purple-600 relative px-2 pb-1 router-link flex flex-col items-center w-full"
          active-class="font-bold text-purple-600 after:content-[''] after:block after:w-full after:h-1 after:bg-purple-600 after:rounded after:mt-1 after:transition-transform after:scale-x-100 after:origin-left"
          exact-active-class="font-bold text-purple-600 after:scale-x-100"
        >
          Main
        </router-link>
        <router-link
          to="/drive"
          class="text-lg font-medium text-gray-700 hover:text-purple-600 relative px-2 pb-1 router-link flex flex-col items-center w-full"
          active-class="font-bold text-purple-600 after:content-[''] after:block after:w-full after:h-1 after:bg-purple-600 after:rounded after:mt-1 after:transition-transform after:scale-x-100 after:origin-left"
          exact-active-class="font-bold text-purple-600 after:scale-x-100"
        >
          Debug
        </router-link>
        <router-link
          to="/pid_debug"
          class="text-lg font-medium text-gray-700 hover:text-purple-600 relative px-2 pb-1 router-link flex flex-col items-center w-full"
          active-class="font-bold text-purple-600 after:content-[''] after:block after:w-full after:h-1 after:bg-purple-600 after:rounded after:mt-1 after:transition-transform after:scale-x-100 after:origin-left"
          exact-active-class="font-bold text-purple-600 after:scale-x-100"
        >
          Camera
        </router-link>
        <router-link
          to="/science"
          class="text-lg font-medium text-gray-700 hover:text-purple-600 relative px-2 pb-1 router-link flex flex-col items-center w-full"
          active-class="font-bold text-purple-600 after:content-[''] after:block after:w-full after:h-1 after:bg-purple-600 after:rounded after:mt-1 after:transition-transform after:scale-x-100 after:origin-left"
          exact-active-class="font-bold text-purple-600 after:scale-x-100"
        >
          Science
        </router-link>
        <router-link
          to="/misc"
          class="text-lg font-medium text-gray-700 hover:text-purple-600 relative px-2 pb-1 router-link flex flex-col items-center w-full"
          active-class="font-bold text-purple-600 after:content-[''] after:block after:w-full after:h-1 after:bg-purple-600 after:rounded after:mt-1 after:transition-transform after:scale-x-100 after:origin-left"
          exact-active-class="font-bold text-purple-600 after:scale-x-100"
        >
          Terminal
        </router-link>
      </div>
      <button class="bg-red-500 hover:bg-red-600 text-white font-bold py-2 px-4 rounded">
        Disconnect
      </button>
    </nav>
    <router-view class="flex-grow h-full" />
  </div>
</template>
