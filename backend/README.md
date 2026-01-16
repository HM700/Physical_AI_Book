# Physical AI Book Backend

Backend services for the Physical AI Book educational platform, providing APIs for the documentation site, simulation integration, and AI services.

## Overview

The backend is built with FastAPI and provides:
- RESTful APIs for course content and user progress
- Integration with simulation environments (Gazebo, Isaac Sim)
- AI services including RAG chatbot functionality
- Data models for each learning module
- Authentication and user management

## Project Structure

```
backend/
├── src/
│   ├── models/           # Data models for each module
│   │   ├── __init__.py
│   │   ├── base.py       # Base model definitions
│   │   ├── user.py       # User model
│   │   ├── course.py     # Course content model
│   │   ├── progress.py   # User progress model
│   │   └── chat.py       # Chat interaction model
│   ├── services/         # Business logic services
│   │   ├── __init__.py
│   │   ├── auth.py       # Authentication service
│   │   ├── course.py     # Course content service
│   │   ├── progress.py   # Progress tracking service
│   │   ├── chat.py       # Chatbot service
│   │   ├── ros2/         # ROS 2 integration services
│   │   ├── simulation/   # Simulation services
│   │   └── ai/           # AI/ML services
│   └── api/              # API endpoints
│       ├── __init__.py
│       ├── main.py       # Main API router
│       ├── auth.py       # Authentication endpoints
│       ├── courses.py    # Course content endpoints
│       ├── progress.py   # Progress tracking endpoints
│       └── chat.py       # Chatbot endpoints
├── tests/                # Unit and integration tests
├── pyproject.toml        # Python project configuration
├── .env                  # Environment variables (not tracked)
├── .env.example          # Environment variables template
└── README.md             # This file
```

## Setup

### Prerequisites
- Python 3.11+
- pip
- Virtual environment (recommended)

### Installation

1. **Create a virtual environment:**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -e .
   ```

3. **Configure environment variables:**
   ```bash
   cp .env.example .env
   # Edit .env with your specific configuration
   ```

4. **Run the development server:**
   ```bash
   uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
   ```

## Development

### Running Tests

```bash
# Run all tests
pytest tests/

# Run tests with coverage
pytest tests/ --cov=src

# Run specific test file
pytest tests/test_auth.py
```

### Code Quality

```bash
# Format code with black
black src/

# Check linting with flake8
flake8 src/

# Type checking with mypy
mypy src/
```

### API Documentation

The API automatically generates interactive documentation at:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `BACKEND_HOST` | Host to bind the server to | `0.0.0.0` |
| `BACKEND_PORT` | Port to bind the server to | `8000` |
| `DATABASE_URL` | Database connection string | `postgresql://...` |
| `QDRANT_URL` | Qdrant vector database URL | `http://localhost:6333` |
| `OPENAI_API_KEY` | OpenAI API key for chatbot | `...` |
| `JWT_SECRET_KEY` | Secret key for JWT tokens | `...` |

## Architecture

### Models
- SQLAlchemy models for database entities
- Pydantic models for API requests/responses
- Data validation and serialization

### Services
- Business logic layer
- Integration with external services
- Data processing and transformation

### API
- FastAPI routers
- Authentication and authorization
- API documentation and validation

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Run the test suite
6. Submit a pull request

### Code Standards
- Follow PEP 8 style guide
- Write type hints for all functions
- Include docstrings for public functions
- Use meaningful variable and function names