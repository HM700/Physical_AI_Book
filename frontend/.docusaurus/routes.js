import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Physical-AI-Book/layout',
    component: ComponentCreator('/Physical-AI-Book/layout', '608'),
    exact: true
  },
  {
    path: '/Physical-AI-Book/docs',
    component: ComponentCreator('/Physical-AI-Book/docs', '232'),
    routes: [
      {
        path: '/Physical-AI-Book/docs',
        component: ComponentCreator('/Physical-AI-Book/docs', '7b7'),
        routes: [
          {
            path: '/Physical-AI-Book/docs',
            component: ComponentCreator('/Physical-AI-Book/docs', '20c'),
            routes: [
              {
                path: '/Physical-AI-Book/docs/about',
                component: ComponentCreator('/Physical-AI-Book/docs/about', '3ef'),
                exact: true
              },
              {
                path: '/Physical-AI-Book/docs/conclusion',
                component: ComponentCreator('/Physical-AI-Book/docs/conclusion', '2b5'),
                exact: true
              },
              {
                path: '/Physical-AI-Book/docs/development-setup',
                component: ComponentCreator('/Physical-AI-Book/docs/development-setup', '7c2'),
                exact: true
              },
              {
                path: '/Physical-AI-Book/docs/faq',
                component: ComponentCreator('/Physical-AI-Book/docs/faq', 'c0d'),
                exact: true
              },
              {
                path: '/Physical-AI-Book/docs/intro',
                component: ComponentCreator('/Physical-AI-Book/docs/intro', '1df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module1/basics',
                component: ComponentCreator('/Physical-AI-Book/docs/module1/basics', '2a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module1/intro',
                component: ComponentCreator('/Physical-AI-Book/docs/module1/intro', '2dd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module1/nodes',
                component: ComponentCreator('/Physical-AI-Book/docs/module1/nodes', '414'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module1/projects',
                component: ComponentCreator('/Physical-AI-Book/docs/module1/projects', 'bce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module1/topics',
                component: ComponentCreator('/Physical-AI-Book/docs/module1/topics', '860'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module2/gazebo',
                component: ComponentCreator('/Physical-AI-Book/docs/module2/gazebo', '20c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module2/integration',
                component: ComponentCreator('/Physical-AI-Book/docs/module2/integration', 'caf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module2/intro',
                component: ComponentCreator('/Physical-AI-Book/docs/module2/intro', '826'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module2/projects',
                component: ComponentCreator('/Physical-AI-Book/docs/module2/projects', '5ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module2/unity',
                component: ComponentCreator('/Physical-AI-Book/docs/module2/unity', '865'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module3/intro',
                component: ComponentCreator('/Physical-AI-Book/docs/module3/intro', '1ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module3/isaac',
                component: ComponentCreator('/Physical-AI-Book/docs/module3/isaac', 'ca4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module3/navigation',
                component: ComponentCreator('/Physical-AI-Book/docs/module3/navigation', '6a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module3/perception',
                component: ComponentCreator('/Physical-AI-Book/docs/module3/perception', '761'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module3/projects',
                component: ComponentCreator('/Physical-AI-Book/docs/module3/projects', '3d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module4/conclusion',
                component: ComponentCreator('/Physical-AI-Book/docs/module4/conclusion', '30e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module4/integration',
                component: ComponentCreator('/Physical-AI-Book/docs/module4/integration', '726'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module4/intro',
                component: ComponentCreator('/Physical-AI-Book/docs/module4/intro', '13e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module4/projects',
                component: ComponentCreator('/Physical-AI-Book/docs/module4/projects', 'ad9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Physical-AI-Book/docs/module4/vla',
                component: ComponentCreator('/Physical-AI-Book/docs/module4/vla', '019'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/Physical-AI-Book/',
    component: ComponentCreator('/Physical-AI-Book/', '2d6'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
