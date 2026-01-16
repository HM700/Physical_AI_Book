# Physical AI Book Quickstart Guide

## Overview

This guide provides the essential steps to set up and run the Physical AI Book system, including all four modules: ROS 2, Digital Twin, AI-Robot Brain, and Vision-Language-Action.

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **Hardware**: 8GB+ RAM, 4+ CPU cores, Dedicated GPU (NVIDIA preferred for Isaac Sim)
- **Software**:
  - Docker and Docker Compose
  - Python 3.11+
  - Git
  - Node.js 18+ and npm/yarn

## Installation Steps

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-book.git
cd physical-ai-book
```

### 2. Install System Dependencies

#### For Ubuntu/Debian:
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install locales
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Install ROS 2 keys and repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install simulation tools (minimum setup for quickstart)
sudo apt install python3-pip
pip3 install colcon-build
```

#### For Windows with WSL2:
```powershell
# Enable WSL2 and install Ubuntu from Microsoft Store
# Then follow Ubuntu instructions inside WSL2
```

### 3. Set Up Development Environment

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Install Python dependencies
pip3 install -r requirements.txt  # From the repository root
```

### 4. Configure Backend Services

```bash
# Navigate to backend directory
cd backend

# Install Python dependencies
pip3 install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration (API keys, database URLs, etc.)

# Initialize database (if using one)
python3 -m src.database.init
```

### 5. Build and Run the System

#### Option A: Using Docker (Recommended for beginners)
```bash
# From repository root
docker-compose up --build
```

#### Option B: Native installation (Advanced users)
```bash
# Terminal 1: Start backend
cd backend
python3 -m src.api.main

# Terminal 2: Start frontend (Docusaurus)
cd frontend
npm install
npm start

# Terminal 3: Start simulation environment (separate for each module)
# For Module 1 (ROS 2):
source /opt/ros/humble/setup.bash
cd ~/physical_ai_ws
colcon build
source install/setup.bash
ros2 launch module1_launch demo.launch.py

# For Module 2 (Digital Twin - Gazebo):
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros_empty_world.launch.py

# For Module 3 (Isaac Sim):
# Follow NVIDIA Isaac Sim installation guide
# Then launch simulation environment
```

### 6. Access the Application

- **Frontend (Docusaurus)**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **RAG Chatbot**: Available within the documentation pages
- **Simulation GUI**: Opens automatically when simulation modules are launched

## Module-Specific Quickstarts

### Module 1: ROS 2 Fundamentals
```bash
# After sourcing ROS 2 environment
cd ~/physical_ai_ws/src/module1_ros2_examples
colcon build
source install/setup.bash

# Run a basic publisher-subscriber demo
ros2 run module1_examples talker
# In another terminal:
ros2 run module1_examples listener
```

### Module 2: Digital Twin (Gazebo)
```bash
# Launch a simple robot in Gazebo
ros2 launch module2_gazebo simple_robot.launch.py
```

### Module 3: AI-Robot Brain (Isaac Sim)
```bash
# Ensure Isaac Sim is properly installed
# Launch Isaac Sim environment
ros2 launch module3_isaac navigation_demo.launch.py
```

### Module 4: Vision-Language-Action
```bash
# Requires OpenAI API key in environment
export OPENAI_API_KEY="your-api-key"

# Run voice command demo
python3 -m src.module4_vla.voice_to_action_demo
```

## Testing Your Setup

### Basic System Test
```bash
# Test backend connectivity
curl http://localhost:8000/health

# Test frontend
open http://localhost:3000  # or visit in browser
```

### Module 1 Test
```bash
# Verify ROS 2 installation
ros2 topic list
# Should show topics like /parameter_events, /rosout
```

### RAG Chatbot Test
1. Visit the Docusaurus site
2. Look for the chatbot widget
3. Ask a question about Physical AI concepts
4. Verify responses are contextually relevant

## Troubleshooting

### Common Issues

1. **ROS 2 not found**: Ensure you've sourced the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Port already in use**: Kill processes using ports 3000 or 8000:
   ```bash
   lsof -ti:3000 | xargs kill
   lsof -ti:8000 | xargs kill
   ```

3. **Simulation fails to launch**: Check GPU drivers and X11 forwarding if using remote desktop.

4. **RAG chatbot not responding**: Verify API keys are set and vector database is running.

### Verification Commands

```bash
# Check ROS 2 installation
ros2 --version

# Check Python dependencies
python3 -c "import rclpy; import openai; import fastapi; print('Dependencies OK')"

# Check simulation availability
gz --version  # For Gazebo Garden
```

## Next Steps

1. Proceed to Module 1: [ROS 2 Fundamentals](../docs/module1/index.md)
2. Complete the basic tutorials
3. Experiment with provided simulation scenarios
4. Try modifying examples to understand the concepts
5. Move to Module 2 once Module 1 is complete

## Getting Help

- Check the [Troubleshooting Guide](../docs/troubleshooting.md)
- Review the [FAQ](../docs/faq.md)
- Join our [Community Forum](link-to-forum)
- Submit an issue on [GitHub](link-to-github)