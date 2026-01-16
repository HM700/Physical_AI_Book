@echo off
REM Setup script for Physical AI Book development environment on Windows

echo Setting up Physical AI Book development environment...

REM Check if Python 3.11+ is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Python is not installed or not in PATH. Please install Python 3.11 or higher.
    pause
    exit /b 1
)

REM Check Python version
for /f "tokens=2" %%i in ('python --version') do set version=%%i
for /f "delims=. tokens=1" %%i in ("%version%") do set major=%%i
for /f "delims=. tokens=2" %%i in ("%version%") do set minor=%%i

if %major% lss 3 (
    echo Python 3.11 or higher is required. Current version: %version%
    pause
    exit /b 1
)

if %major% equ 3 if %minor% lss 11 (
    echo Python 3.11 or higher is required. Current version: %version%
    pause
    exit /b 1
)

echo Python version %version% is OK.

REM Setup backend
echo Setting up backend...
cd backend
python -m venv venv
call venv\Scripts\activate
pip install --upgrade pip
pip install -e .
pip install -e ".[dev]"
echo Backend setup complete.

REM Go back to root
cd ..

REM Setup frontend
echo Setting up frontend...
cd frontend
npm install
echo Frontend setup complete.

cd ..

echo.
echo Development environment setup complete!
echo.
echo To activate the backend environment, run: backend\venv\Scripts\activate
echo To start the backend: uvicorn src.api.main:app --reload
echo To start the frontend: cd frontend && npm start
echo.
pause