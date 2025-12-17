# Humanoid Robotics Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-07

## Active Technologies

- Docusaurus (static site generator for documentation)
- ROS 2 Humble Hawksbill (Robot Operating System)
- Gazebo (physics simulation environment)
- Unity (visualization generation)
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
- OpenAI Agents SDK (intelligent agent framework)
- FastAPI (web framework for backend services)
- Google Generative AI (embedding generation)
- Pydantic (data validation and settings management)

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

backend/
├── services/
│   ├── __init__.py
│   ├── qdrant_service.py          # Real-time Qdrant operations
│   └── embedding_service.py       # Real-time embedding operations
├── agents/
│   ├── __init__.py
│   ├── tools.py                   # Function tools for agents
│   └── rag_agent.py               # Agent definitions as variables
├── api/
│   ├── __init__.py
│   ├── models.py                  # Pydantic API models
│   └── main.py                    # FastAPI application
├── config/
│   ├── __init__.py
│   └── settings.py                # Extended configuration
├── requirements.txt               # Updated dependencies
└── main.py                        # Application entry point
```

## Commands

### Docusaurus Commands
- `npm run start` - Start development server
- `npm run build` - Build static site
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to GitHub Pages

### Backend Service Commands
- `cd backend && python main.py` - Start the RAG chatbot service
- `cd backend && uvicorn api.main:app --reload` - Start with auto-reload
- `cd backend && python -m pytest` - Run backend tests

### ROS 2 Commands
- `source /opt/ros/humble/setup.bash` - Source ROS 2 environment
- `mkdir -p ~/ros2_ws/src` - Create ROS 2 workspace
- `colcon build` - Build ROS 2 packages
- `ros2 run <package> <executable>` - Run ROS 2 nodes

### Development Workflow
- Create new chapter: Add markdown file with proper front-matter to appropriate module
- Update navigation: Modify sidebars.js to include new content
- Add custom components: Place in src/components/ and import in markdown files
- Add backend features: Implement in backend/ following service/agent/api pattern

## Code Style

### Markdown/Docusaurus
- Use proper front-matter with sidebar_position, title, and description
- Follow Docusaurus content guidelines
- Include hands-on exercises with code examples
- Add assessment checklists for each chapter

### Python (ROS 2 and Backend Services)
- Follow ROS 2 Python client library (rclpy) conventions for ROS 2 code
- Use FastAPI patterns for web endpoints
- Implement proper error handling and logging
- Follow PEP 8 style guidelines
- Use Pydantic for data validation
- Apply dependency injection patterns for service layers
- Use async/await for I/O operations

### JavaScript/React
- Use functional components with hooks
- Follow Docusaurus component conventions
- Maintain accessibility standards
- Use TypeScript where possible for better type safety

## Recent Changes

- Feature 001-docusaurus-humanoids-book: Created comprehensive textbook structure with 4 modules covering ROS 2, simulation, AI navigation, and VLA robotics. Implemented modular architecture with hands-on exercises and assessment checklists.

- Feature 1-openai-agents-sdk-implementation: Implemented RAG chatbot service using OpenAI Agents SDK, FastAPI, and Qdrant vector database. Added specialized agents for greetings, documentation queries, and selected text processing with REST API endpoints.

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->