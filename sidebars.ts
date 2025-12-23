import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Textbook sidebar for the Physical AI & Humanoid Robotics book
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Quarter Overview',
      items: ['overview/intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module1/intro',
        'module1/nodes-topics-services',
        'module1/rclpy',
        'module1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity',
      items: [
        'module2/intro',
        'module2/physics',
        'module2/sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        'module3/intro',
        'module3/isaac-sim',
        'module3/ros-integration',
        'module3/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module4/intro',
        'module4/whisper',
        'module4/llms-planning',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/intro'],
    },
  ],
};

export default sidebars;
