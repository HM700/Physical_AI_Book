import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/layout/',
    component: ComponentCreator('/layout/', '16a'),
    exact: true
  },
  {
    path: '/docs/',
    component: ComponentCreator('/docs/', 'f56'),
    routes: [
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', '060'),
        routes: [
          {
            path: '/docs/',
            component: ComponentCreator('/docs/', 'e94'),
            routes: [
              {
                path: '/docs/about/',
                component: ComponentCreator('/docs/about/', 'a69'),
                exact: true
              },
              {
                path: '/docs/conclusion/',
                component: ComponentCreator('/docs/conclusion/', 'e4c'),
                exact: true
              },
              {
                path: '/docs/development-setup/',
                component: ComponentCreator('/docs/development-setup/', '18b'),
                exact: true
              },
              {
                path: '/docs/faq/',
                component: ComponentCreator('/docs/faq/', '947'),
                exact: true
              },
              {
                path: '/docs/intro/',
                component: ComponentCreator('/docs/intro/', 'e44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/basics/',
                component: ComponentCreator('/docs/module1/basics/', '41b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/intro/',
                component: ComponentCreator('/docs/module1/intro/', '914'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/nodes/',
                component: ComponentCreator('/docs/module1/nodes/', '48f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/projects/',
                component: ComponentCreator('/docs/module1/projects/', '0ef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/topics/',
                component: ComponentCreator('/docs/module1/topics/', 'a87'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo/',
                component: ComponentCreator('/docs/module2/gazebo/', 'b5f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/integration/',
                component: ComponentCreator('/docs/module2/integration/', 'b00'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/intro/',
                component: ComponentCreator('/docs/module2/intro/', 'c95'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/projects/',
                component: ComponentCreator('/docs/module2/projects/', 'bfb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity/',
                component: ComponentCreator('/docs/module2/unity/', 'a8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/intro/',
                component: ComponentCreator('/docs/module3/intro/', 'eec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac/',
                component: ComponentCreator('/docs/module3/isaac/', 'fdb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/navigation/',
                component: ComponentCreator('/docs/module3/navigation/', '49f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/perception/',
                component: ComponentCreator('/docs/module3/perception/', '72e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/projects/',
                component: ComponentCreator('/docs/module3/projects/', 'bd1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/conclusion/',
                component: ComponentCreator('/docs/module4/conclusion/', '5a4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/integration/',
                component: ComponentCreator('/docs/module4/integration/', 'bbc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/intro/',
                component: ComponentCreator('/docs/module4/intro/', '682'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/projects/',
                component: ComponentCreator('/docs/module4/projects/', 'd4d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/vla/',
                component: ComponentCreator('/docs/module4/vla/', '94e'),
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
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
