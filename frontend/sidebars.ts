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
  // Manual sidebar for the ROS 2 Fundamentals course
  tutorialSidebar: [
    'intro',
    'quickstart',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'module-1/intro',
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3'
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      collapsed: true,
      items: [
        'index-page',
        'quickstart',
        'summary',
        'glossary',
        'api-reference',
        'troubleshooting',
        'best-practices'
      ],
    },
  ],
};

export default sidebars;
