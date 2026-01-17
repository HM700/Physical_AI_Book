import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/layout',
    component: ComponentCreator('/layout', '1f9'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '475'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '4c6'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '149'),
            routes: [
              {
                path: '/docs/about',
                component: ComponentCreator('/docs/about', '3fb'),
                exact: true
              },
              {
                path: '/docs/conclusion',
                component: ComponentCreator('/docs/conclusion', '0aa'),
                exact: true
              },
              {
                path: '/docs/development-setup',
                component: ComponentCreator('/docs/development-setup', 'f49'),
                exact: true
              },
              {
                path: '/docs/faq',
                component: ComponentCreator('/docs/faq', '489'),
                exact: true
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/basics',
                component: ComponentCreator('/docs/module1/basics', '314'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/intro',
                component: ComponentCreator('/docs/module1/intro', 'f8b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/nodes',
                component: ComponentCreator('/docs/module1/nodes', '1a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/projects',
                component: ComponentCreator('/docs/module1/projects', '996'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/topics',
                component: ComponentCreator('/docs/module1/topics', 'ac6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo',
                component: ComponentCreator('/docs/module2/gazebo', '12c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/integration',
                component: ComponentCreator('/docs/module2/integration', '893'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/intro',
                component: ComponentCreator('/docs/module2/intro', '1bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/projects',
                component: ComponentCreator('/docs/module2/projects', '338'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity',
                component: ComponentCreator('/docs/module2/unity', '084'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/intro',
                component: ComponentCreator('/docs/module3/intro', '937'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac',
                component: ComponentCreator('/docs/module3/isaac', '61a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/navigation',
                component: ComponentCreator('/docs/module3/navigation', 'ee6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/perception',
                component: ComponentCreator('/docs/module3/perception', '89d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/projects',
                component: ComponentCreator('/docs/module3/projects', '634'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/conclusion',
                component: ComponentCreator('/docs/module4/conclusion', 'cd3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/integration',
                component: ComponentCreator('/docs/module4/integration', 'c91'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/intro',
                component: ComponentCreator('/docs/module4/intro', 'b26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/projects',
                component: ComponentCreator('/docs/module4/projects', 'da2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/vla',
                component: ComponentCreator('/docs/module4/vla', '37b'),
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
