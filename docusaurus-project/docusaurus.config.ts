import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Future of Work — Agents + Humans + Robots',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'facebook',
  projectName: 'docusaurus',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&family=Fira+Code:wght@400;500;600&display=swap',
      },
    },
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.ts'),
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: { type: ['rss', 'atom'], xslt: true },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    // Plugin to inject environment variables into the browser
    () => ({
      name: 'inject-environment-variables',
      configureWebpack: (config: any) => {
        // Load environment variables from .env file in docusaurus-project directory
        const path = require('path');
        require('dotenv').config({ path: path.resolve(__dirname, '.env') });

        const apiUrl = process.env.CHAT_API_BASE_URL || 'http://localhost:8000';

        const envVars = {
          'process.env.CHAT_API_BASE_URL': JSON.stringify(apiUrl),
        };

        console.log('🔧 Webpack DefinePlugin - Injecting CHAT_API_BASE_URL:', apiUrl);

        return {
          resolve: {
            fallback: {
              process: require.resolve('process'),
              buffer: require.resolve('buffer'),
            },
          },
          plugins: [
            ...(config.plugins || []),
            new (require('webpack')).DefinePlugin(envVars),
            new (require('webpack')).ProvidePlugin({
              process: 'process/browser',
              Buffer: ['buffer', 'Buffer'],
            }),
          ],
        };
      },
    }),
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      hideOnScroll: true,
      title: 'Physical AI & Humanoid Robotics',
      logo: { alt: 'Logo', src: 'img/robot.svg' },
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Introduction' },
        { to: '/blog', label: 'Blog', position: 'left' },
        { href: 'https://github.com/anthropic/humanoids-robotics-book', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'Prerequisites', to: '/docs/prerequisites' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Discord', href: 'https://discord.gg/your-discord-invite' },
            { label: 'GitHub', href: 'https://github.com/anthropic/humanoids-robotics-book' },
            { label: 'Email', href: 'mailto:info@physicalai.com' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Blog', to: '/blog' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Project — Built with Docusaurus.`,
    },
    prism: { theme: prismThemes.github, darkTheme: prismThemes.dracula },
  } satisfies Preset.ThemeConfig,
};

export default config;