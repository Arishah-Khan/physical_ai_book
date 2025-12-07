# Humanoid Robotics Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-07

## Active Technologies

- Docusaurus (static site generator for documentation)
- ROS 2 Humble Hawksbill (Robot Operating System)
- Gazebo (physics simulation environment)
- Unity (visualization platform)
- NVIDIA Isaac Sim (photorealistic simulation)
- Navigation2 (Nav2) stack for navigation
- OpenAI Whisper (speech recognition)
- Large Language Models (cognitive task planning)
- Markdown (content format)
- JavaScript/TypeScript (web components)
- Python 3.8+ (ROS 2 examples)
- Qdrant Cloud (vector database for RAG)
- Better-Auth (authentication system)
- Neon Postgres (database)

## Project Structure

```text
docusaurus-project/
├── docs/
│   ├── module1/ (ROS 2 fundamentals)
│   ├── module2/ (Digital twin simulation)
│   ├── module3/ (AI-powered navigation)
│   ├── module4/ (Vision-language-action robotics)
│   ├── prerequisites.md
│   ├── roadmap.md
│   ├── glossary.md
│   └── references.md
├── src/ (custom React components)
├── static/ (static assets)
├── docusaurus.config.js
├── sidebars.js
└── package.json
```

## Commands

### Docusaurus Commands
- `npm run start` - Start development server
- `npm run build` - Build static site
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to GitHub Pages

### ROS 2 Commands
- `source /opt/ros/humble/setup.bash` - Source ROS 2 environment
- `mkdir -p ~/ros2_ws/src` - Create ROS 2 workspace
- `colcon build` - Build ROS 2 packages
- `ros2 run <package> <executable>` - Run ROS 2 nodes

### Development Workflow
- Create new chapter: Add markdown file with proper front-matter to appropriate module
- Update navigation: Modify sidebars.js to include new content
- Add custom components: Place in src/components/ and import in markdown files

## Code Style

### Markdown/Docusaurus
- Use proper front-matter with sidebar_position, title, and description
- Follow Docusaurus content guidelines
- Include hands-on exercises with code examples
- Add assessment checklists for each chapter

### Python (ROS 2)
- Follow ROS 2 Python client library (rclpy) conventions
- Use proper node structure with lifecycle management
- Implement proper error handling and logging
- Follow PEP 8 style guidelines

### JavaScript/React
- Use functional components with hooks
- Follow Docusaurus component conventions
- Maintain accessibility standards
- Use TypeScript where possible for better type safety

## Recent Changes

- Feature 001-docusaurus-humanoids-book: Created comprehensive textbook structure with 4 modules covering ROS 2, simulation, AI navigation, and VLA robotics. Implemented modular architecture with hands-on exercises and assessment checklists.

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->