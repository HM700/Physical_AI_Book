#!/bin/bash
# Setup script for Physical AI Book development environment on Unix-like systems

echo "Setting up Physical AI Book development environment..."

# Check if Python 3.11+ is installed
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed or not in PATH. Please install Python 3.11 or higher."
    exit 1
fi

# Check Python version
version=$(python3 --version 2>&1 | cut -d' ' -f2)
major=$(echo $version | cut -d'.' -f1)
minor=$(echo $version | cut -d'.' -f2)

if [ "$major" -lt 3 ] || ([ "$major" -eq 3 ] && [ "$minor" -lt 11 ]); then
    echo "Python 3.11 or higher is required. Current version: $version"
    exit 1
fi

echo "Python version $version is OK."

# Setup backend
echo "Setting up backend..."
cd backend
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -e .
pip install -e ".[dev]"
echo "Backend setup complete."

# Go back to root
cd ..

# Setup frontend
echo "Setting up frontend..."
cd frontend
npm install
echo "Frontend setup complete."

cd ..

echo ""
echo "Development environment setup complete!"
echo ""
echo "To activate the backend environment, run: source backend/venv/bin/activate"
echo "To start the backend: uvicorn src.api.main:app --reload"
echo "To start the frontend: cd frontend && npm start"
echo ""

# Make the script executable
chmod +x scripts/setup_dev_env.sh