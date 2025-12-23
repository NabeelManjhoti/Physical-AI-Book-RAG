# Physical AI & Humanoid Robotics Book

This project contains an authoritative textbook on Physical AI & Humanoid Robotics that bridges the gap between artificial intelligence and embodied systems. The content focuses on the theoretical foundations and practical implementations of AI systems that interact with the physical world through robotic platforms.

## Project Structure

```
Physical-AI-Book-RAG/
├── .specify/                 # Spec-Kit Plus configuration
│   ├── memory/              # Project memory (constitution, etc.)
│   └── templates/           # Template files for specs, plans, tasks
├── specs/                   # Feature specifications
├── history/                 # Prompt History Records and ADRs
│   └── prompts/             # PHRs organized by stage
├── docs/                    # Docusaurus documentation pages
├── src/                     # Custom Docusaurus components
├── static/                  # Static assets
├── blog/                    # Blog posts (if any)
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts             # Navigation sidebars
└── package.json            # Project dependencies
```

## Technology Stack

- **Docusaurus v3**: Static site generation with React-based documentation
- **TypeScript**: Type-safe JavaScript for all custom components
- **ROS 2**: Robot Operating System for communication and control
- **Gazebo & Unity**: Simulation environments for testing
- **NVIDIA Isaac**: GPU-accelerated perception and control
- **VLA (Variable Length Actions)**: Flexible action representation for manipulation
- **Vercel**: Production deployment and hosting

## Getting Started

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Start development server:**
   ```bash
   npm start
   ```

3. **Build for production:**
   ```bash
   npm run build
   ```

## Contributing

This project follows the Spec-Kit Plus methodology with the following workflow:

1. **Specification**: Create feature specifications in `/specs/[feature-name]/spec.md`
2. **Planning**: Generate architecture plans with `/sp.plan`
3. **Tasks**: Generate implementable tasks with `/sp.tasks`
4. **Implementation**: Follow the generated tasks
5. **Documentation**: Update documentation as you implement

## Project Constitution

The project is governed by the constitution in `.specify/memory/constitution.md`. All contributions must align with the core principles:

1. Embodied Intelligence Focus
2. Technical Excellence
3. Multi-Platform Integration
4. Educational Accessibility
5. Practical Application Focus
6. Docusaurus Documentation Excellence

## Development Guidelines

- All code examples must use TypeScript
- Follow the dark futuristic robotics theme with animations
- Maintain responsive layout for all devices
- Ensure consistent navigation with identical navicon/favicon
- Include Home and Textbook links in the navbar

## Deployment

The site is deployed to Vercel. To deploy manually:

```bash
vercel --prod
```

## Architecture Decision Records (ADRs)

Significant architectural decisions are documented in `history/adr/`.

## Prompt History Records (PHRs)

All development prompts and responses are recorded in `history/prompts/` organized by stage.