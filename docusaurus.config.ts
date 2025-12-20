import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';



const config: Config = {
  title: 'Master Physical AI & Humanoid Robotics',
  tagline: 'Learn, Build, and Innovate with AI-Powered Robots',
  favicon: 'img/favicon.ico',

  
  future: {
    v4: true, 
  },

  
  url: 'https://khudaijailyas.github.io',
  
  
  baseUrl: '/',

 
  
  organizationName: 'khudaijailyas', 
  projectName: 'physical-ai-book', 
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  

  
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          
        
          editUrl:
            'https://github.com/khudaijailyas/physical-ai-book/tree/main/book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          
          editUrl:
            'https://github.com/khudaijailyas/physical-ai-book/edit/main/blog/',
          
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
   
    image: 'img/physical-ai-social-card.png',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
  title: 'AI Spec Book',
  logo: {
    alt: 'AI Spec Logo',
    src: 'img/logo.png',
  },
  items: [
    { to: '/docs/physical-ai-introduction', label: 'Book', position: 'left' },
    { to: '/blog', label: 'Blog', position: 'left' },
    
   
  ],
},

    footer: {
  style: 'dark', 
  links: [
    {
      title: 'Docs',
      items: [
        { label: 'Getting Started', to: '/docs/intro' },
        { label: 'Spec-Kit Plus', href: 'https://github.com/panaversity/spec-kit-plus/' },
        { label: 'Claude Code', href: 'https://www.claude.com/product/claude-code' },
      ],
    },
    {
      title: 'Community',
      items: [
        { label: 'Instagram', href: 'https://www.instagram.com/khudaija._.ilyas?igsh=MWl3d2lvYjZwNXRrMg==' },
        { label: 'Discord', href: 'https://discord.gg/zRHbwn57' },
        { label: 'Linkdin', href: 'https://www.linkedin.com/in/khudaija-ilyas-8a09a22b8' },
      ],
    },
    {
      title: 'More',
      items: [
        { label: 'Blog', to: '/blog' },
        { label: 'GitHub', href: 'https://github.com/khudaijailyas/physical-ai-book' },
      ],
    },
  ],
  copyright: `© ${new Date().getFullYear()} Khudaija Ilyas. Built with Docusaurus.`,
},

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
