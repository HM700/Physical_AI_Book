---
sidebar_position: 98
---

# Development Setup

This guide explains how to set up your development environment for contributing to the Physical AI Book project.

## Prerequisites

Before starting development, ensure you have the following installed:

- **Python 3.11+** - Backend development
- **Node.js 18+** - Frontend development
- **npm** - Package manager for frontend
- **Git** - Version control
- **Docker** - Containerization (optional but recommended)
- **ROS 2 Humble** - For robotics simulation (optional for core development)

## Quick Setup

### Automated Setup (Recommended)

We provide setup scripts for both Windows and Unix-like systems:

**On Windows:**
```bash
scripts\setup_dev_env.bat
```

**On Unix/Linux/macOS:**
```bash
chmod +x scripts/setup_dev_env.sh
./scripts/setup_dev_env.sh
```

### Manual Setup

#### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install the package in development mode:
```bash
pip install --upgrade pip
pip install -e .
pip install -e ".[dev]"  # Installs development dependencies
```

#### Frontend Setup

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

## Environment Variables

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
```

## Running the Applications

### Backend (API Server)

```bash
cd backend
source venv/bin/activate  # Activate virtual environment
uvicorn src.api.main:app --reload
```

The backend will be available at `http://localhost:8000`.

### Frontend (Documentation Site)

```bash
cd frontend
npm start
```

The frontend will be available at `http://localhost:3000`.

### With Docker (Recommended for Production)

```bash
docker-compose up --build
```

## Development Commands

### Backend

- **Run tests**: `cd backend && pytest`
- **Format code**: `cd backend && black src/ tests/`
- **Lint code**: `cd backend && flake8 src/ tests/`
- **Type check**: `cd backend && mypy src/`

### Frontend

- **Run development server**: `cd frontend && npm start`
- **Build for production**: `cd frontend && npm run build`
- **Run tests**: `cd frontend && npm test`

## Project Structure

```
Physical_AI_Book/
├── backend/                 # Backend services and APIs
│   ├── src/
│   │   ├── models/         # Data models
│   │   ├── services/       # Business logic
│   │   └── api/            # API endpoints
│   ├── tests/              # Backend tests
│   └── pyproject.toml      # Python project configuration
├── frontend/               # Docusaurus documentation site
│   ├── docs/               # Documentation content
│   ├── src/                # Custom components
│   ├── static/             # Static assets
│   └── package.json        # Frontend dependencies
├── scripts/                # Utility scripts
├── docker-compose.yml      # Multi-service orchestration
└── README.md               # Project overview
```

## Code Style

### Python
- Follow PEP 8 style guide
- Use type hints for all functions
- Write docstrings for public functions

### JavaScript/React
- Use prettier for code formatting (configured in `.prettierrc`)
- Follow React best practices
- Use TypeScript for type safety where possible

## Pre-commit Hooks

We use pre-commit hooks to ensure code quality. Install them with:

```bash
pip install pre-commit
pre-commit install
```

## Troubleshooting

### Common Issues

1. **Port already in use**: Make sure ports 8000 (backend) and 3000 (frontend) are available.

2. **Dependency conflicts**: Make sure you're using the virtual environment for backend development.

3. **Missing environment variables**: Ensure your `.env` file is properly configured.

### Getting Help

If you encounter issues during setup, please check our [FAQ](./faq.md) or reach out to the community for assistance.