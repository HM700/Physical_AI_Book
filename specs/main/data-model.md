# Data Model: Physical AI Book

## Overview

This document defines the data models for the Physical AI Book system, encompassing the educational content, simulation data, user interactions, and RAG system components.

## Core Entities

### 1. Module
Represents a Physical AI learning module (ROS 2, Digital Twin, AI-Robot Brain, VLA)

**Fields:**
- id: String (unique identifier)
- name: String (module name)
- description: String (brief description)
- duration: Integer (estimated completion time in hours)
- prerequisites: Array[String] (module IDs that must be completed first)
- learning_objectives: Array[String] (specific objectives learners will achieve)
- status: Enum['draft', 'published', 'archived']

**Relationships:**
- Contains many Lessons
- Contains many Exercises
- Connected to SimulationEnvironments

### 2. Lesson
Individual lesson within a module containing educational content

**Fields:**
- id: String (unique identifier)
- module_id: String (foreign key to Module)
- title: String (lesson title)
- content: String (Markdown content)
- duration: Integer (estimated reading time in minutes)
- order: Integer (sequence within module)
- prerequisite_lessons: Array[String] (IDs of lessons that must be completed first)

**Relationships:**
- Belongs to one Module
- Contains many Exercises
- Connected to SimulationScenarios

### 3. Exercise
Interactive exercise for hands-on learning

**Fields:**
- id: String (unique identifier)
- lesson_id: String (foreign key to Lesson)
- title: String (exercise title)
- description: String (instructions)
- type: Enum['simulation', 'coding', 'analysis', 'quiz']
- difficulty: Enum['beginner', 'intermediate', 'advanced']
- estimated_time: Integer (time to complete in minutes)
- validation_criteria: String (how to validate completion)

**Relationships:**
- Belongs to one Lesson
- Connected to SimulationScenario

### 4. SimulationScenario
Configuration for simulation-based exercises

**Fields:**
- id: String (unique identifier)
- name: String (scenario name)
- description: String (what the scenario demonstrates)
- environment_type: Enum['gazebo', 'unity', 'isaac_sim']
- robot_model: String (URDF or model file reference)
- initial_conditions: Object (starting positions, states)
- success_criteria: String (conditions for successful completion)
- simulation_files: Array[String] (paths to simulation assets)

**Relationships:**
- Connected to many Exercises
- Connected to many Lessons

### 5. SimulationResult
Record of user's simulation execution

**Fields:**
- id: String (unique identifier)
- exercise_id: String (foreign key to Exercise)
- user_id: String (learner identifier)
- timestamp: DateTime (when executed)
- environment_config: Object (specific parameters used)
- robot_states: Array[Object] (recorded robot positions/states over time)
- sensor_data: Array[Object] (sensor readings during simulation)
- success_status: Boolean (whether success criteria were met)
- execution_log: String (detailed execution information)

**Relationships:**
- Belongs to one Exercise
- Belongs to one User

### 6. RAGDocument
Content document for RAG system

**Fields:**
- id: String (unique identifier)
- title: String (document title)
- content: String (full text content)
- source_url: String (original location in documentation)
- module_id: String (which module this content belongs to)
- lesson_id: String (which lesson this content belongs to)
- embedding: Array[Float] (vector representation for similarity search)
- metadata: Object (additional context like page numbers, sections)
- created_at: DateTime (timestamp of ingestion)

**Relationships:**
- Connected to many ChatMessages (as context source)

### 7. ChatMessage
Individual message in the RAG-enabled chatbot conversation

**Fields:**
- id: String (unique identifier)
- session_id: String (chat session identifier)
- sender: Enum['user', 'assistant']
- content: String (message text)
- timestamp: DateTime
- context_documents: Array[String] (IDs of RAG documents used)
- source_module: String (relevant module ID)
- source_lesson: String (relevant lesson ID)

**Relationships:**
- Belongs to one ChatSession
- Connected to many RAGDocuments

### 8. ChatSession
Complete conversation session with the RAG chatbot

**Fields:**
- id: String (unique identifier)
- user_id: String (learner identifier)
- start_time: DateTime
- last_activity: DateTime
- topic_focus: String (current discussion topic/module)
- active_module: String (module currently being discussed)

**Relationships:**
- Contains many ChatMessages

### 9. UserProgress
Track learner progress through the Physical AI Book

**Fields:**
- id: String (unique identifier)
- user_id: String (learner identifier)
- module_id: String (module being tracked)
- lesson_id: String (lesson being tracked)
- exercise_id: String (exercise being tracked)
- completion_percentage: Float (0-100)
- last_accessed: DateTime
- time_spent: Integer (minutes spent on this component)
- achievements: Array[String] (completed milestones)

**Relationships:**
- Belongs to one User
- Connected to one Module/Lesson/Exercise

### 10. SimulationEnvironment
Runtime environment for executing simulations

**Fields:**
- id: String (unique identifier)
- name: String (environment name)
- type: Enum['gazebo', 'unity', 'isaac_sim']
- version: String (version of simulation software)
- capabilities: Array[String] (supported features)
- status: Enum['available', 'busy', 'maintenance']
- resource_requirements: Object (CPU, GPU, memory requirements)

**Relationships:**
- Executes many SimulationScenarios

## State Transitions

### Module States
- `draft` → `published`: When module content is complete and reviewed
- `published` → `archived`: When module is deprecated

### Exercise Completion States
- `not_started` → `in_progress`: When user begins exercise
- `in_progress` → `completed_success`: When success criteria met
- `in_progress` → `completed_failed`: When exercise attempted but failed
- `completed_*` → `reset`: When user resets exercise

### Simulation Result States
- `pending` → `running` → `completed`: Normal execution flow
- `running` → `error`: When simulation encounters error

## Validation Rules

1. **Module Prerequisites**: A module cannot be accessed until all prerequisites are completed
2. **Lesson Sequencing**: Lessons must be completed in order within a module
3. **Exercise Dependencies**: Exercises may have specific prerequisite lessons
4. **Simulation Validation**: Simulation results must meet success criteria to be marked as successful
5. **RAG Content Integrity**: Documents must be linked to valid modules/lessons
6. **User Progress Consistency**: Progress tracking must reflect actual completion status

## Indexes & Performance Considerations

1. **Module Access**: Index on module_id for quick lookup
2. **User Progress**: Composite index on (user_id, module_id) for progress tracking
3. **RAG Search**: Vector index on embedding field for similarity search
4. **Chat Sessions**: Index on session_id and timestamp for conversation retrieval
5. **Simulation Results**: Index on exercise_id and user_id for result lookup