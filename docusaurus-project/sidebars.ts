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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  // Using manual structure to ensure proper mobile dropdown display
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'prerequisites', 'glossary', 'roadmap'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/chapter01-introduction-to-ros2',
        'module1/chapter02-rclpy-basics',
        'module1/chapter03-urdf-modeling',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Physics Simulation & Rendering',
      items: [
        'module2/chapter1-gazebo-physics-simulation',
        'module2/chapter2-unity-high-fidelity-rendering',
        'module2/chapter3-sensor-simulation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: Visual SLAM & Navigation',
      items: [
        'module3/chapter1-isaac-sim-photorealistic',
        'module3/chapter2-isaac-ros-vslam-navigation',
        'module3/chapter3-nav2-humanoid-path-planning',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: LLMs & Cognitive Planners',
      items: [
        'module4/chapter1-whisper-voice-commands',
        'module4/chapter2-llm-cognitive-planners',
        'module4/chapter3-capstone-autonomous-humanoid',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
