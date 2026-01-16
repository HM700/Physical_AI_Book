# Physical AI Book

An educational system teaching Physical AI concepts through four progressive modules:

1. **Module 1: The Robotic Nervous System (ROS 2)** - Learn the foundational middleware layer that allows AI software to control humanoid robots in simulated physical environments
2. **Module 2: Digital Twin (Gazebo & Unity)** - Create digital twins of humanoid robots and their environments to validate control logic, physics, and interactions
3. **Module 3: AI-Robot Brain (NVIDIA Isaac)** - Build the AI brain for humanoid robots using NVIDIA Isaac Sim and Isaac ROS with photorealistic simulation and synthetic data
4. **Module 4: Vision-Language-Action (VLA)** - Integrate LLMs, speech recognition, and cognitive planning to enable humanoid robots to perform tasks based on human instructions

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Setup Instructions](#setup-instructions)
- [Development](#development)
- [Modules](#modules)
- [Architecture](#architecture)
- [Contributing](#contributing)
- [License](#license)

## Overview

The Physical AI Book is a comprehensive educational platform designed to teach students and developers how to build AI systems that perceive, reason, and act in the physical world. The curriculum follows a simulation-first approach, allowing learners to experiment with humanoid robots in safe, controlled environments before transitioning to real hardware.

Key features:
- Progressive learning modules from basic ROS 2 concepts to advanced Vision-Language-Action systems
- Integrated Docusaurus documentation with interactive examples
- RAG-powered chatbot for personalized learning assistance
- Simulation-first approach using Gazebo, Unity, and NVIDIA Isaac Sim
- Real-world applications and practical exercises

## Project Structure

```
Physical_AI_Book/
├── frontend/                 # Docusaurus documentation site
│   ├── docs/                 # Module documentation
│   │   ├── module1/          # ROS 2 fundamentals
│   │   ├── module2/          # Digital Twin (Gazebo & Unity)
│   │   ├── module3/          # AI-Robot Brain (NVIDIA Isaac)
│   │   └── module4/          # Vision-Language-Action
│   ├── blog/                 # Educational blog posts
│   ├── src/                  # Custom React components
│   ├── static/               # Static assets
│   ├── docusaurus.config.js  # Site configuration
│   └── package.json          # Frontend dependencies
├── backend/                  # Backend services and APIs
│   ├── src/
│   │   ├── models/           # Data models for each module
│   │   ├── services/         # Business logic services
│   │   │   ├── ros2/         # ROS 2 integration services
│   │   │   ├── simulation/   # Simulation services
│   │   │   ├── ai/           # AI/ML services
│   │   │   └── chat/         # Chatbot and RAG services
│   │   └── api/              # API endpoints
│   ├── tests/                # Backend tests
│   ├── pyproject.toml        # Python project configuration
│   └── README.md             # Backend documentation
├── docker-compose.yml        # Multi-service orchestration
├── Dockerfile               # Multi-stage Docker build
├── .env.example             # Environment variables template
├── spec/                    # Specifications
│   ├── constitution/        # Project constitution
│   └── modules/             # Individual module specs
├── history/                 # Decision records and history
├── infra/                   # Infrastructure as code
└── README.md                # This file
```

## Prerequisites

### Hardware Requirements
- **CPU**: 4+ cores (8+ recommended)
- **RAM**: 8GB+ (16GB+ recommended)
- **GPU**: Dedicated graphics card (NVIDIA preferred for Isaac Sim and CUDA acceleration)
- **Storage**: 50GB+ free space for simulation environments

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **ROS 2**: Humble Hawksbill LTS (Python 3.10 compatible)
- **Simulation**: Gazebo Garden, Unity Hub (optional), NVIDIA Isaac Sim (optional)
- **Python**: 3.11+ with pip
- **Node.js**: 18+ with npm
- **Docker**: 20.10+ with Docker Compose v2+
- **Git**: 2.25+

### Additional Tools
- Docker Desktop (Windows/Mac)
- Visual Studio Code with Python and ROS extensions
- CUDA Toolkit (for GPU acceleration, optional)

## Setup Instructions

### Quick Start with Docker (Recommended)

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-org/Physical_AI_Book.git
   cd Physical_AI_Book
   ```

2. **Copy environment template and configure:**
   ```bash
   cp .env.example .env
   # Edit .env with your specific configuration
   ```

3. **Build and start services:**
   ```bash
   docker-compose up --build
   ```

4. **Access the services:**
   - Documentation site: http://localhost:3000
   - Backend API: http://localhost:8000
   - Admin panel: http://localhost:3001 (if applicable)

### Manual Setup

#### Backend Setup

1. **Install Python dependencies:**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\\Scripts\\activate
   pip install --upgrade pip
   pip install -e .
   ```

2. **Configure environment variables:**
   ```bash
   cp .env.example .env
   # Edit .env with your specific configuration
   ```

3. **Start backend service:**
   ```bash
   uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
   ```

#### Frontend Setup

1. **Install Node.js dependencies:**
   ```bash
   cd frontend
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

### Environment Configuration

Create a `.env` file in the project root with the following variables:

```bash
# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
DATABASE_URL=postgresql://user:password@localhost/dbname
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=your_openai_api_key_here

# Frontend Configuration
REACT_APP_BACKEND_URL=http://localhost:8000
REACT_APP_CHAT_ENABLED=true

# Simulation Configuration
ROS_DOMAIN_ID=0
GAZEBO_WORLD_PATH=/path/to/worlds
```

## Development

### Running Services

#### Development Mode
```bash
# Terminal 1: Start backend
cd backend
uvicorn src.api.main:app --reload

# Terminal 2: Start frontend
cd frontend
npm start
```

#### Using Docker Compose
```bash
# Build and start all services
docker-compose up --build

# Start specific services
docker-compose up backend frontend

# Run in detached mode
docker-compose up -d
```

### Testing

#### Backend Tests
```bash
cd backend
# Run all tests
pytest tests/

# Run tests with coverage
pytest tests/ --cov=src

# Run specific test file
pytest tests/test_auth.py

# Run tests with verbose output
pytest tests/ -v
```

#### Frontend Tests
```bash
cd frontend
npm test
```

### Building Documentation
```bash
cd frontend
npm run build
```

### Development Scripts
We provide convenience scripts for common development tasks:

- **Setup development environment**:
  - Windows: `scripts\setup_dev_env.bat`
  - Unix/Linux/macOS: `scripts/setup_dev_env.sh`

- **Run all tests**: `cd backend && pytest tests/ --cov=src`
- **Format code**: `cd backend && black src/ tests/ && cd frontend && npm run format` (if formatter is configured)
- **Check code quality**: `cd backend && flake8 src/ && mypy src/`

## Modules

### Module 1: ROS 2 Fundamentals
Learn to implement ROS 2 nodes in Python, build humanoid URDF models, and demonstrate command → movement in simulation.
- **Duration**: 2-3 weeks
- **Topics**: Nodes, topics, services, actions, TF transforms
- **Projects**: Basic publisher/subscriber, URDF robot model, simple navigation

### Module 2: Digital Twin
Set up Gazebo & Unity simulation environments, integrate sensors and environment physics, and connect with Module 1 ROS 2 nodes.
- **Duration**: 3-4 weeks
- **Topics**: Physics simulation, sensor integration, environment modeling
- **Projects**: Gazebo world creation, sensor fusion, simulation-physical mapping

### Module 3: AI-Robot Brain
Add advanced perception and navigation, generate synthetic data in Isaac Sim, and use Isaac ROS + Nav2 for path planning.
- **Duration**: 4-5 weeks
- **Topics**: Perception pipelines, navigation, synthetic data generation
- **Projects**: Object detection, path planning, SLAM implementation

### Module 4: Vision-Language-Action
Implement Whisper voice-to-text pipeline, use LLM to map natural language → ROS actions, and execute tasks in simulation.
- **Duration**: 4-5 weeks
- **Topics**: Speech recognition, LLM integration, action planning
- **Projects**: Voice-controlled robot, natural language interface, task execution

## Architecture

### Technology Stack
- **Frontend**: React, Docusaurus, TypeScript
- **Backend**: Python, FastAPI, Pydantic
- **Database**: PostgreSQL (Neon), Vector DB (Qdrant)
- **Simulation**: Gazebo Garden, ROS 2 Humble
- **AI/ML**: OpenAI API, Whisper, Custom ML models
- **Containerization**: Docker, Docker Compose
- **Documentation**: Markdown, MDX

### Service Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │◄──►│    Backend       │◄──►│  Databases      │
│ (Docusaurus)    │    │   (FastAPI)      │    │ (PostgreSQL,   │
│                 │    │                  │    │  Qdrant)        │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                       ┌──────────────┐
                       │  Simulation  │
                       │  Environment │
                       └──────────────┘
```

## Contributing

This project follows Spec-Driven Development principles. All changes must be specified before implementation:

1. **Fork the repository**
2. **Create a feature branch** (`git checkout -b feature/amazing-feature`)
3. **Follow the specification-driven approach**:
   - Define requirements in `spec/` directory
   - Create implementation plan
   - Generate tasks
   - Implement following the tasks
4. **Add tests for new functionality**
5. **Update documentation**
6. **Submit a pull request**

### Code Standards
- Follow PEP 8 for Python code
- Use TypeScript for type safety in frontend
- Maintain consistent documentation style
- Write comprehensive tests

## License

MIT License - see LICENSE file for details.

## Support

For support, please open an issue in the GitHub repository or contact the development team.