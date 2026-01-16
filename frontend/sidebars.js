// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: ['module1/intro', 'module1/basics', 'module1/nodes', 'module1/topics', 'module1/projects'],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: ['module2/intro', 'module2/gazebo', 'module2/unity', 'module2/integration', 'module2/projects'],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      items: ['module3/intro', 'module3/isaac', 'module3/perception', 'module3/navigation', 'module3/projects'],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: ['module4/intro', 'module4/vla', 'module4/integration', 'module4/projects', 'module4/conclusion'],
    },
  ],
};

export default sidebars;