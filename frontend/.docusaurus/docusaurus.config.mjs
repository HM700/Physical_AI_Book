// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Book',
  tagline: 'Learn Physical AI: AI systems that perceive, reason, and act in the physical world',
  favicon: 'img/favicon.ico',

  // ✅ MUST match your Vercel domain
  url: 'https://physical-ai-book-rust-zeta.vercel.app',

  // ✅ MUST be "/" because Vercel rootDir = frontend
  baseUrl: '/',

  trailingSlash: false,

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: 'docs',
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/HM700/Physical_AI_Book/edit/master/frontend/',
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/HM700/Physical_AI_Book/edit/master/frontend/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          href: 'https://github.com/HM700/Physical_AI_Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            { label: 'Module 1: ROS 2', to: '/docs/module1/intro' },
            { label: 'Module 2: Digital Twin', to: '/docs/module2/intro' },
            { label: 'Module 3: AI-Robot Brain', to: '/docs/module3/intro' },
            { label: 'Module 4: Vision-Language-Action', to: '/docs/module4/intro' },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/HM700/Physical_AI_Book',
            },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Physical AI Book.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;
