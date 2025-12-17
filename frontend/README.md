# ROS 2 Fundamentals for Humanoid Robotics

This repository contains the course materials for "ROS 2 Fundamentals for Humanoid Robotics" - a comprehensive course for AI and software engineering students transitioning into humanoid robotics.

## About This Course

This course teaches students how to use ROS 2 (Robot Operating System 2) as the "nervous system" connecting AI agents to physical robot bodies. Students will learn:

- ROS 2 communication fundamentals (nodes, topics, services, and actions)
- How to bridge Python-based AI agents to ROS controllers
- Robot body representation using URDF (Unified Robot Description Format)

## Prerequisites

- Basic programming knowledge in Python
- Familiarity with command-line interfaces
- Understanding of basic robotics concepts (optional but helpful)

## Getting Started

1. Install Node.js (version 20 or higher)
2. Clone this repository
3. Navigate to the project directory
4. Install dependencies: `npm install`
5. Start the development server: `npm start`

The course will be available at `http://localhost:3000`.

## Course Structure

The course is organized into modules, with Module 1 covering ROS 2 fundamentals:

- Chapter 1: ROS 2 Communication Fundamentals
- Chapter 2: Python Agents to Robot Control
- Chapter 3: Robot Body Representation (URDF)

## Contributing

If you find issues with the course content or have suggestions for improvements, please open an issue or submit a pull request.

## License

This course material is provided as open educational content.

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

### Local Deployment

To build and serve the site locally for testing:

```bash
npm run build
npm run serve
```

The site will be available at `http://localhost:3000`.

### GitHub Pages Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

### Custom Deployment

For custom deployment to other platforms:

1. Build the static site: `npm run build`
2. The built site will be in the `build/` directory
3. Deploy the contents of the `build/` directory to your web server

### Environment Configuration

For different deployment environments, you may need to adjust the `baseUrl` in `docusaurus.config.ts`:

- For GitHub Pages: `baseUrl: '/your-repo-name/'`
- For custom domains: `baseUrl: '/'`
- For subdirectories: `baseUrl: '/your-subdirectory/'`
