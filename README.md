# Physical AI & Humanoid Robotics Textbook

This repository contains the source code for the Physical AI & Humanoid Robotics textbook, built with Docusaurus. The textbook provides comprehensive coverage of embodied intelligence, combining artificial intelligence with robotic systems.

## Overview

This textbook covers the essential technologies and concepts needed to build intelligent, embodied systems that can perceive, reason, and act in the physical world. The content is organized into four comprehensive modules and a capstone project:

- **Quarter Overview**: Introduction to Physical AI concepts and course structure
- **Module 1**: ROS 2 - Robot Operating System fundamentals (nodes, topics, services, rclpy, URDF)
- **Module 2**: Gazebo & Unity - Physics simulation and sensor integration
- **Module 3**: NVIDIA Isaac - GPU-accelerated perception and control with Isaac Sim, ROS, and Nav2
- **Module 4**: Vision-Language-Action (VLA) Models - Using Whisper and LLMs for robotic planning
- **Capstone Project**: Integration of all concepts in a practical humanoid robotics application

## Features

- **Dark Futuristic Robotics Theme**: Custom design with animations and visual effects
- **Responsive Design**: Works on all device sizes
- **TypeScript Support**: Type-safe JavaScript for all custom components
- **Comprehensive Content**: Covers the complete robotics development stack
- **Interactive Elements**: Code examples and visualizations

## Technology Stack

- **Docusaurus v3**: Static site generation with React-based documentation
- **TypeScript**: Type-safe JavaScript for all custom components
- **ROS 2**: Robot Operating System for communication and control
- **Gazebo & Unity**: Simulation environments for testing
- **NVIDIA Isaac**: GPU-accelerated perception and control
- **VLA (Variable Length Actions)**: Flexible action representation for manipulation
- **Vercel**: Production deployment and hosting

## Getting Started

### Installation

```bash
npm install
```

### Development

```bash
npm start
```

This command starts a local development server and opens the site in your browser. Most changes are reflected live without having to restart the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

### Deployment

The site is configured for deployment on Vercel. Simply push to the repository and Vercel will automatically build and deploy the site.

## Project Structure

```
Physical-AI-Book-RAG/
├── docs/                    # Docusaurus documentation pages
│   ├── overview/           # Quarter overview content
│   ├── module1/            # ROS 2 module content
│   ├── module2/            # Gazebo & Unity module content
│   ├── module3/            # NVIDIA Isaac module content
│   ├── module4/            # VLA module content
│   └── capstone/           # Capstone project content
├── src/                    # Custom Docusaurus components
│   ├── components/         # React components
│   ├── css/               # Custom CSS styles
│   └── pages/             # Custom pages
├── static/                 # Static assets
├── docusaurus.config.ts    # Docusaurus configuration
├── sidebars.ts            # Navigation sidebars
└── package.json           # Project dependencies
```

## Contributing

This project follows the Spec-Kit Plus methodology with the following workflow:

1. **Specification**: Create feature specifications in `/specs/[feature-name]/spec.md`
2. **Planning**: Generate architecture plans with `/sp.plan`
3. **Tasks**: Generate implementable tasks with `/sp.tasks`
4. **Implementation**: Follow the generated tasks
5. **Documentation**: Update documentation as you implement

## License

This project is licensed under the MIT License.
