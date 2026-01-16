# API Contracts: Physical AI Book

## Overview

This document defines the API contracts for the Physical AI Book system, covering all modules and services including simulation, RAG chatbot, and educational content delivery.

## Base URL
`http://localhost:8000/api/v1` (development) or `https://physical-ai-book.example.com/api/v1` (production)

## Authentication

All endpoints require authentication using Bearer tokens, except for public documentation endpoints.

## Module 1: ROS 2 Fundamentals API

### POST /modules/ros2/nodes
Create a new ROS 2 node

**Request Body:**
```json
{
  "node_name": "string",
  "node_type": "publisher|subscriber|service_client|service_server",
  "topics": ["string"],
  "parameters": {}
}
```

**Response (201 Created):**
```json
{
  "node_id": "string",
  "node_name": "string",
  "status": "running|stopped",
  "created_at": "datetime"
}
```

### GET /modules/ros2/nodes/{node_id}/status
Get status of a ROS 2 node

**Response (200 OK):**
```json
{
  "node_id": "string",
  "status": "running|stopped|error",
  "last_message_count": "integer",
  "uptime_seconds": "integer"
}
```

### POST /modules/ros2/topics/publish
Publish a message to a topic

**Request Body:**
```json
{
  "topic_name": "string",
  "message_type": "string",
  "data": {}
}
```

**Response (200 OK):**
```json
{
  "success": "boolean",
  "message_id": "string",
  "timestamp": "datetime"
}
```

## Module 2: Digital Twin Simulation API

### POST /modules/digital-twin/simulations
Launch a new simulation environment

**Request Body:**
```json
{
  "simulation_type": "gazebo|unity",
  "world_file": "string",
  "robot_model": "string",
  "duration_limit": "integer",
  "initial_conditions": {}
}
```

**Response (201 Created):**
```json
{
  "simulation_id": "string",
  "status": "launching|running|failed",
  "connection_details": {
    "host": "string",
    "port": "integer"
  }
}
```

### GET /modules/digital-twin/simulations/{simulation_id}/state
Get current state of simulation

**Response (200 OK):**
```json
{
  "simulation_id": "string",
  "status": "running|paused|stopped",
  "robot_positions": [
    {
      "robot_id": "string",
      "position": {"x": "float", "y": "float", "z": "float"},
      "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
    }
  ],
  "timestamp": "datetime"
}
```

## Module 3: AI-Robot Brain API

### POST /modules/ai-brain/navigation/plan
Create a navigation plan for the robot

**Request Body:**
```json
{
  "robot_id": "string",
  "goal_position": {"x": "float", "y": "float", "z": "float"},
  "map_id": "string",
  "planner_type": "nav2|custom"
}
```

**Response (200 OK):**
```json
{
  "plan_id": "string",
  "status": "success|failure|partial",
  "waypoints": [
    {"x": "float", "y": "float", "z": "float"}
  ],
  "estimated_duration": "float"
}
```

### POST /modules/ai-brain/perception/detect
Process sensor data for object detection

**Request Body:**
```json
{
  "sensor_type": "camera|lidar|imu",
  "sensor_data": {},
  "detection_types": ["object", "obstacle", "landmark"]
}
```

**Response (200 OK):**
```json
{
  "detection_results": [
    {
      "object_type": "string",
      "confidence": "float",
      "position": {"x": "float", "y": "float", "z": "float"},
      "bounding_box": {"min_x": "float", "max_x": "float", "min_y": "float", "max_y": "float"}
    }
  ]
}
```

## Module 4: Vision-Language-Action API

### POST /modules/vla/voice-to-action
Convert voice command to robot actions

**Request Body:**
```json
{
  "audio_data": "base64_encoded_string",
  "audio_format": "wav|mp3|flac",
  "target_robot": "string",
  "context": "string"  // Current environment/situation
}
```

**Response (200 OK):**
```json
{
  "command_id": "string",
  "transcription": "string",
  "parsed_intent": "string",
  "action_sequence": [
    {
      "action_type": "move|grasp|navigate|speak",
      "parameters": {},
      "estimated_duration": "float"
    }
  ],
  "confidence": "float"
}
```

### POST /modules/vla/execute-actions
Execute a sequence of actions on the robot

**Request Body:**
```json
{
  "robot_id": "string",
  "actions": [
    {
      "action_type": "move|grasp|navigate|speak",
      "parameters": {},
      "timeout": "float"
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "execution_id": "string",
  "status": "queued|executing|completed|failed",
  "action_results": [
    {
      "action_id": "string",
      "status": "success|failed|cancelled",
      "result_data": {}
    }
  ]
}
```

## RAG Chatbot API

### POST /rag/chat/start-session
Start a new chat session

**Request Body:**
```json
{
  "user_id": "string",
  "initial_context": "string"
}
```

**Response (201 Created):**
```json
{
  "session_id": "string",
  "created_at": "datetime",
  "expires_at": "datetime"
}
```

### POST /rag/chat/messages
Send a message to the chatbot

**Request Body:**
```json
{
  "session_id": "string",
  "message": "string",
  "selected_text": "string"  // Optional: user-selected text for context
}
```

**Response (200 OK):**
```json
{
  "response_id": "string",
  "message": "string",
  "sources": [
    {
      "document_id": "string",
      "title": "string",
      "relevance_score": "float",
      "excerpt": "string"
    }
  ],
  "timestamp": "datetime"
}
```

### GET /rag/documents/search
Search for documents in the knowledge base

**Query Parameters:**
- `q`: Search query string
- `module_filter`: Optional module ID to filter results
- `limit`: Number of results (default: 10)

**Response (200 OK):**
```json
{
  "results": [
    {
      "document_id": "string",
      "title": "string",
      "content_preview": "string",
      "module_id": "string",
      "relevance_score": "float"
    }
  ],
  "total_results": "integer"
}
```

## Simulation Recording and Playback API

### POST /simulations/recordings
Start recording a simulation session

**Request Body:**
```json
{
  "simulation_id": "string",
  "recording_name": "string",
  "include_sensors": ["camera", "lidar", "imu"],
  "include_robot_state": "boolean"
}
```

**Response (201 Created):**
```json
{
  "recording_id": "string",
  "status": "recording|stopped",
  "start_time": "datetime"
}
```

### GET /simulations/recordings/{recording_id}/download
Download a recorded simulation

**Response (200 OK):**
- Content-Type: application/octet-stream
- File containing recorded simulation data

## User Progress Tracking API

### POST /users/{user_id}/progress
Update user progress for a specific module/component

**Request Body:**
```json
{
  "module_id": "string",
  "component_type": "lesson|exercise|demo",
  "component_id": "string",
  "completion_percentage": "float",
  "status": "started|in_progress|completed",
  "time_spent": "integer"
}
```

**Response (200 OK):**
```json
{
  "progress_id": "string",
  "updated_at": "datetime",
  "next_recommendation": {
    "module_id": "string",
    "component_id": "string",
    "type": "string"
  }
}
```

### GET /users/{user_id}/progress
Get user's overall progress

**Response (200 OK):**
```json
{
  "user_id": "string",
  "overall_completion": "float",
  "modules_completed": ["string"],
  "current_module": "string",
  "total_time_spent": "integer",
  "achievements": ["string"],
  "recommendations": [
    {
      "module_id": "string",
      "title": "string",
      "difficulty": "string",
      "estimated_time": "integer"
    }
  ]
}
```

## Error Responses

All error responses follow this structure:

**Response (4xx/5xx):**
```json
{
  "error_code": "string",
  "message": "string",
  "details": {},
  "timestamp": "datetime"
}
```

### Common Error Codes:
- `INVALID_INPUT`: Request data doesn't match expected format
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `AUTHENTICATION_FAILED`: Invalid or expired authentication token
- `SIMULATION_ERROR`: Simulation environment error
- `RATE_LIMIT_EXCEEDED`: Too many requests in a time window
- `INTERNAL_ERROR`: Unexpected server error