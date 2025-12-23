# Quickstart Guide: Docusaurus Book - Physical AI & Humanoid Robotics

## Overview
This guide provides a step-by-step approach to getting the Physical AI & Humanoid Robotics textbook platform up and running for development.

## Prerequisites
- Node.js 18+ installed
- Yarn package manager
- Git version control system
- Text editor or IDE with TypeScript support

## Setup Instructions

### 1. Clone and Initialize
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install dependencies
yarn install
```

### 2. Start Development Server
```bash
# Start the development server
yarn start

# This will start the Docusaurus server and open the site in your browser
# The site will automatically reload when content changes
```

### 3. Project Structure Navigation
```bash
# Main content files are in the docs/ directory
docs/
├── overview/          # Quarter overview content
├── module-1/          # ROS 2 content
├── module-2/          # Gazebo & Unity content
├── module-3/          # NVIDIA Isaac content
├── module-4/          # VLA content
└── capstone/          # Capstone project content

# Custom components are in src/
src/
├── components/        # Reusable React components
├── pages/            # Custom page components
└── theme/            # Custom theme components

# Static assets are in static/
static/
├── img/              # Images and icons
└── assets/           # Other static assets
```

### 4. Adding New Content
```bash
# To add a new page, create a Markdown file in the appropriate module directory
docs/module-1/new-topic.md

# Use the following frontmatter template:
---
title: Title of the Page
description: Brief description of the content
sidebar_position: X
---

# Page Title

Content goes here...
```

### 5. Custom Components
```bash
# To create interactive elements, use custom MDX components
import { InteractiveCode } from '@site/src/components/InteractiveCode';

<InteractiveCode>
  ```ts
  // Your TypeScript code example here
  ```
</InteractiveCode>
```

### 6. Building for Production
```bash
# Build the static site
yarn build

# The built site will be in the build/ directory
# Deploy this directory to your hosting platform (e.g., Vercel)
```

### 7. Running Tests
```bash
# Run all tests
yarn test

# Run tests in watch mode
yarn test:watch

# Run accessibility tests
yarn test:accessibility
```

## Common Tasks

### Creating a New Module
1. Create a new directory in `docs/` (e.g., `docs/module-5/`)
2. Add an `index.md` file for the module introduction
3. Create individual topic files for each lesson
4. Update `sidebars.ts` to include the new module in the navigation

### Customizing the Theme
1. Modify `src/css/custom.css` for global styles
2. Create custom theme components in `src/theme/`
3. Add animations in `src/css/animations.css`

### Adding Code Examples
1. Use standard Markdown code blocks with language specification
2. For interactive examples, use the `InteractiveCode` component
3. Ensure all TypeScript examples follow the project's type safety standards

## Development Workflow

1. **Content Creation**: Add new content to the appropriate module in `docs/`
2. **Component Development**: Create reusable components in `src/components/`
3. **Theme Customization**: Modify appearance in `src/theme/` and `src/css/`
4. **Testing**: Run tests to ensure functionality and accessibility
5. **Preview**: Use `yarn start` to preview changes locally
6. **Build**: Use `yarn build` to create production-ready site
7. **Deploy**: Push to repository for automatic deployment to Vercel

## Troubleshooting

### Common Issues
- **Page not loading**: Check that the file is in the correct directory and has proper frontmatter
- **Styles not applying**: Verify that custom CSS is imported in `docusaurus.config.ts`
- **Component not found**: Ensure the component is properly exported and imported

### Performance Tips
- Use code splitting for large components
- Optimize images and assets in the static directory
- Use Docusaurus' built-in image optimization features