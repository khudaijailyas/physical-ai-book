import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';


const sidebars = {
   tutorialsidebar:[
    'intro',
    'physical-ai-introduction',
    'part-1-foundations',
{
      type: 'category',
      label: 'ROS 2 – The Robotic Nervous System',
      items: [
        'ros2-introduction',
        'ros2-architecture',
        'ros2-nodes-topics-services',
        'ros2-python-agents',
        'ros2-launch-parameters',
        'urdf-for-humanoids',
      ],
    },
    {
  type: 'category',
  label: 'Digital Twins & Simulation',
  items: [
    'digital-twins-introduction',
    'gazebo-simulation',
    'webots-simulation',
    'isaac-simulation',
    'ros2-simulation-integration',
    'digital-twins-labs',
  ],
},
{
  type: 'category',
  label: 'The AI Robot Brain (NVIDIA Isaac)',
  items: [
    'isaac-introduction',
    'isaac-sim-basics',
    'isaac-perception-sensors',
    'isaac-planning-control',
    'ros2-isaac-integration',
    'isaac-labs',
  ],
},
{
  type: 'category',
  label: 'Vision–Language–Action',
  items: [
    'vla-introduction',
    'vision-for-robots',
    'language-models-robotics',
    'vision-language-models',
    'action-policy-execution',
    'end-to-end-vla',
  ],
},
{
  type: 'category',
  label: 'Humanoid Systems',
  items: [
    'humanoid-introduction',
    'humanoid-anatomy-actuators',
    'humanoid-kinematics-locomotion',
    'humanoid-balance-control',
    'humanoid-sensors-perception',
    'humanoid-simulation-testing',
  ],
},
{
  type: 'category',
  label: 'Conversational Robotics',
  items: [
    'conversational-robotics-introduction',
    'speech-recognition-robots',
    'natural-language-understanding',
    'dialogue-management-systems',
    'speech-synthesis-robots',
    'conversational-robots-applications',
  ],
},
{
  type: 'category',
  label: 'Capstone Project',
  items: [
    'capstone-overview',
    'capstone-problem-scope',
    'capstone-system-design',
    'capstone-implementation',
    'capstone-evaluation',
  ],
},
{
  type: 'category',
  label: 'Hardware & Lab Architecture',
  items: [
    'lab-architecture-overview',
    'robot-hardware-components',
    'sensors-actuators-setup',
    'compute-networking-power',
    'lab-layout-safety',
    'deployment-maintenance-ethics',
  ],
},

  ],
};

export default sidebars;
