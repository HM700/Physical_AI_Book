---
sidebar_position: 2
---

# Vision-Language-Action (VLA) Systems

Vision-Language-Action (VLA) systems represent the integration of computer vision, natural language processing, and robotic action execution. These systems enable robots to understand human instructions, perceive their environment, and execute complex tasks.

## Introduction to VLA Systems

VLA systems combine three critical components:
- **Vision**: Understanding the visual environment
- **Language**: Interpreting natural language instructions
- **Action**: Executing appropriate robotic behaviors

## OpenAI Whisper Integration

Whisper is OpenAI's automatic speech recognition (ASR) system that can convert spoken language to text.

### Setting Up Whisper

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import whisper
import torch
import numpy as np
import io
import wave

class WhisperSpeechToTextNode(Node):
    def __init__(self):
        super().__init__('whisper_speech_to_text')

        # Subscriptions
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10
        )

        # Publishers
        self.text_pub = self.create_publisher(String, '/speech_text', 10)
        self.interpretation_pub = self.create_publisher(String, '/interpreted_command', 10)

        # Initialize Whisper model
        # Options: tiny, base, small, medium, large
        self.model = whisper.load_model("base")

        # Audio processing parameters
        self.sample_rate = 16000  # Whisper expects 16kHz audio
        self.audio_buffer = []
        self.buffer_size = 16000  # 1 second buffer at 16kHz

    def audio_callback(self, msg):
        """Process incoming audio data."""
        # Convert audio data to numpy array
        audio_np = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Add to buffer
        self.audio_buffer.extend(audio_np)

        # Process when buffer is full
        if len(self.audio_buffer) >= self.buffer_size:
            # Process audio with Whisper
            audio_segment = np.array(self.audio_buffer[:self.buffer_size])

            # Transcribe using Whisper
            result = self.model.transcribe(audio_segment)
            transcribed_text = result["text"].strip()

            if transcribed_text:  # Only publish if there's actual text
                # Publish transcribed text
                text_msg = String()
                text_msg.data = transcribed_text
                self.text_pub.publish(text_msg)

                # Interpret the command
                interpreted_cmd = self.interpret_command(transcribed_text)

                # Publish interpreted command
                cmd_msg = String()
                cmd_msg.data = interpreted_cmd
                self.interpretation_pub.publish(cmd_msg)

                self.get_logger().info(f'Speech: "{transcribed_text}" -> Command: "{interpreted_cmd}"')

            # Keep remainder of buffer for next processing
            self.audio_buffer = self.audio_buffer[self.buffer_size:]

    def interpret_command(self, text):
        """Interpret natural language command into robot action."""
        text_lower = text.lower()

        # Simple rule-based interpretation
        if any(word in text_lower for word in ["go to", "move to", "navigate to", "walk to"]):
            # Extract location if possible
            location = self.extract_location(text_lower)
            if location:
                return f"NAVIGATE_TO:{location}"
            else:
                return "NAVIGATE:DEFAULT_LOCATION"

        elif any(word in text_lower for word in ["pick up", "grasp", "take", "grab"]):
            object_name = self.extract_object(text_lower)
            if object_name:
                return f"PICK_UP:{object_name}"
            else:
                return "PICK_UP:UNKNOWN_OBJECT"

        elif any(word in text_lower for word in ["put down", "place", "drop", "release"]):
            object_name = self.extract_object(text_lower)
            if object_name:
                return f"PLACE:{object_name}"
            else:
                return "PLACE:OBJECT"

        elif any(word in text_lower for word in ["follow me", "come with me", "follow"]):
            return "FOLLOW_HUMAN:START"

        elif any(word in text_lower for word in ["stop", "halt", "pause"]):
            return "STOP:ALL_ACTIONS"

        elif any(word in text_lower for word in ["find", "locate", "search for"]):
            object_name = self.extract_object(text_lower)
            if object_name:
                return f"FIND_OBJECT:{object_name}"
            else:
                return "FIND_OBJECT:ANY"

        else:
            # Use LLM for more complex interpretation
            return self.llm_interpret_command(text)

    def extract_location(self, text):
        """Extract location from text."""
        # Simple location extraction
        locations = ["kitchen", "bedroom", "living room", "office", "bathroom", "hallway", "dining room"]
        for loc in locations:
            if loc in text:
                return loc.upper().replace(" ", "_")
        return None

    def extract_object(self, text):
        """Extract object from text."""
        # Simple object extraction
        objects = ["cup", "book", "ball", "phone", "keys", "bottle", "box", "toy"]
        for obj in objects:
            if obj in text:
                return obj.upper()
        return None

    def llm_interpret_command(self, text):
        """Use LLM to interpret complex commands."""
        # In a real implementation, this would call an LLM
        # For now, return a default interpretation
        return f"COMPLEX_COMMAND:{text[:50]}..."  # Truncate for simplicity

def main(args=None):
    rclpy.init(args=args)
    stt_node = WhisperSpeechToTextNode()

    try:
        rclpy.spin(stt_node)
    except KeyboardInterrupt:
        pass
    finally:
        stt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Large Language Model Integration

Integrating LLMs to map natural language to robotic actions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
import re

class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge')

        # Initialize OpenAI client
        openai.api_key = "YOUR_OPENAI_API_KEY"  # Set in environment
        self.client = openai.OpenAI()

        # Subscriptions
        self.command_sub = self.create_subscription(
            String, '/interpreted_command', self.command_callback, 10
        )
        self.voice_command_sub = self.create_subscription(
            String, '/speech_text', self.voice_command_callback, 10
        )

        # Publishers
        self.robot_action_pub = self.create_publisher(String, '/robot_action', 10)
        self.navigation_goal_pub = self.create_publisher(Pose, '/move_base_simple/goal', 10)

        # Robot capabilities
        self.robot_capabilities = {
            "navigation": ["move_to", "go_to", "navigate_to", "travel_to"],
            "manipulation": ["pick_up", "place", "grasp", "release", "take", "put"],
            "interaction": ["greet", "follow", "wait", "stop"],
            "perception": ["find", "locate", "identify", "describe"]
        }

        # Location map (would come from semantic map)
        self.location_map = {
            "kitchen": {"x": 1.0, "y": 2.0, "z": 0.0},
            "bedroom": {"x": -1.0, "y": 3.0, "z": 0.0},
            "living_room": {"x": 0.0, "y": 0.0, "z": 0.0},
            "office": {"x": 2.0, "y": -1.0, "z": 0.0}
        }

    def command_callback(self, msg):
        """Process interpreted commands."""
        command = msg.data
        self.process_command(command)

    def voice_command_callback(self, msg):
        """Process raw voice commands with LLM interpretation."""
        raw_command = msg.data
        interpreted_command = self.interpret_with_llm(raw_command)
        self.process_command(interpreted_command)

    def interpret_with_llm(self, command_text):
        """Use LLM to interpret natural language command."""
        system_prompt = """
        You are a robot command interpreter. Convert natural language commands to structured robot commands.
        Respond with a JSON object containing:
        {
            "action": "navigation|manipulation|interaction|perception",
            "target": "specific object or location",
            "parameters": {...}
        }

        Example mappings:
        - "Go to the kitchen" -> {"action": "navigation", "target": "kitchen", "parameters": {}}
        - "Pick up the red cup" -> {"action": "manipulation", "target": "cup", "parameters": {"color": "red"}}
        - "Find my keys" -> {"action": "perception", "target": "keys", "parameters": {}}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Command: {command_text}"}
                ],
                temperature=0.1,
                max_tokens=200
            )

            # Parse the response
            response_text = response.choices[0].message.content.strip()

            # Extract JSON from response (in case it includes text around it)
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                parsed = json.loads(json_str)

                # Format as robot command
                if parsed['action'] == 'navigation':
                    return f"NAVIGATE_TO:{parsed['target'].upper().replace(' ', '_')}"
                elif parsed['action'] == 'manipulation':
                    return f"MANIPULATE:{parsed['target'].upper()}:{json.dumps(parsed['parameters'])}"
                elif parsed['action'] == 'perception':
                    return f"PERCEIVE:{parsed['target'].upper()}"
                else:
                    return f"INTERACTION:{parsed['target'].upper()}"
            else:
                return f"UNKNOWN_COMMAND:{command_text[:50]}"

        except Exception as e:
            self.get_logger().error(f'LLM interpretation error: {str(e)}')
            return f"ERROR_INTERPRETING:{command_text[:50]}"

    def process_command(self, command):
        """Process structured robot command."""
        self.get_logger().info(f'Processing command: {command}')

        if command.startswith('NAVIGATE_TO:'):
            location = command.split(':')[1]
            self.execute_navigation(location)
        elif command.startswith('PICK_UP:'):
            object_name = command.split(':')[1]
            self.execute_manipulation('pick_up', object_name)
        elif command.startswith('PLACE:'):
            object_name = command.split(':')[1]
            self.execute_manipulation('place', object_name)
        elif command.startswith('FIND_OBJECT:'):
            object_name = command.split(':')[1]
            self.execute_perception(object_name)
        else:
            self.get_logger().warn(f'Unknown command format: {command}')

    def execute_navigation(self, location):
        """Execute navigation to specified location."""
        if location.lower() in self.location_map:
            location_data = self.location_map[location.lower()]

            # Create goal pose
            goal_pose = Pose()
            goal_pose.position.x = location_data['x']
            goal_pose.position.y = location_data['y']
            goal_pose.position.z = location_data['z']
            # Set orientation to face forward (z-axis)
            goal_pose.orientation.z = 1.0
            goal_pose.orientation.w = 0.0

            # Publish navigation goal
            self.navigation_goal_pub.publish(goal_pose)

            # Publish action command
            action_msg = String()
            action_msg.data = f"NAVIGATE_TO:{location}"
            self.robot_action_pub.publish(action_msg)

            self.get_logger().info(f'Navigating to {location} at ({location_data["x"]}, {location_data["y"]})')
        else:
            self.get_logger().warn(f'Unknown location: {location}')

    def execute_manipulation(self, action, object_name):
        """Execute manipulation action."""
        # This would interface with robot manipulator
        action_msg = String()
        action_msg.data = f"MANIPULATION:{action}:{object_name}"
        self.robot_action_pub.publish(action_msg)

        self.get_logger().info(f'Executing manipulation: {action} {object_name}')

    def execute_perception(self, target_object):
        """Execute perception task."""
        action_msg = String()
        action_msg.data = f"PERCEPTION:FIND:{target_object}"
        self.robot_action_pub.publish(action_msg)

        self.get_logger().info(f'Looking for object: {target_object}')

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLMBridgeNode()

    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Cognitive Planning System

Creating a cognitive planning system that reasons about tasks:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
import json
from typing import List, Dict, Any

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')

        # Subscriptions
        self.high_level_command_sub = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10
        )
        self.action_feedback_sub = self.create_subscription(
            String, '/action_feedback', self.feedback_callback, 10
        )

        # Publishers
        self.low_level_command_pub = self.create_publisher(String, '/low_level_command', 10)
        self.navigation_goal_pub = self.create_publisher(Pose, '/move_base_simple/goal', 10)
        self.planning_status_pub = self.create_publisher(String, '/planning_status', 10)

        # World model (simplified)
        self.world_state = {
            "locations": {
                "kitchen": {"x": 1.0, "y": 2.0, "objects": ["cup", "plate"]},
                "bedroom": {"x": -1.0, "y": 3.0, "objects": ["book", "keys"]},
                "living_room": {"x": 0.0, "y": 0.0, "objects": ["sofa", "tv"]},
                "office": {"x": 2.0, "y": -1.0, "objects": ["computer", "pen"]}
            },
            "robot_location": "living_room",
            "carried_object": None,
            "tasks_completed": []
        }

        # Task plans database
        self.task_plans = {
            "bring_coffee": [
                {"action": "navigate", "target": "kitchen"},
                {"action": "find_object", "target": "cup"},
                {"action": "grasp", "target": "cup"},
                {"action": "navigate", "target": "living_room"},
                {"action": "place", "target": "table"}
            ],
            "find_keys": [
                {"action": "navigate", "target": "bedroom"},
                {"action": "find_object", "target": "keys"},
                {"action": "report_location", "target": "keys"}
            ],
            "clean_room": [
                {"action": "navigate", "target": "living_room"},
                {"action": "find_object", "target": "trash"},
                {"action": "collect_trash", "target": "all"},
                {"action": "navigate", "target": "kitchen"},
                {"action": "dispose_trash", "target": "bin"}
            ]
        }

        # Current plan and execution state
        self.current_plan = []
        self.current_step_index = 0
        self.planning_state = "IDLE"  # IDLE, PLANNING, EXECUTING, FAILED, COMPLETED

    def command_callback(self, msg):
        """Process high-level commands."""
        command = msg.data
        self.get_logger().info(f'Received high-level command: {command}')

        # Determine task type
        if command.startswith('BRING_'):
            self.plan_and_execute_task('bring_coffee')
        elif command.startswith('FIND_'):
            self.plan_and_execute_task('find_keys')
        elif command.startswith('CLEAN_'):
            self.plan_and_execute_task('clean_room')
        else:
            # Try to determine task from command
            task_type = self.infer_task_type(command)
            if task_type:
                self.plan_and_execute_task(task_type)

    def infer_task_type(self, command):
        """Infer task type from command."""
        command_lower = command.lower()

        if any(word in command_lower for word in ["bring", "get", "fetch", "carry"]):
            return "bring_coffee"  # Simplified mapping
        elif any(word in command_lower for word in ["find", "locate", "search"]):
            return "find_keys"
        elif any(word in command_lower for word in ["clean", "tidy", "organize"]):
            return "clean_room"
        else:
            return None

    def plan_and_execute_task(self, task_type):
        """Plan and execute a task."""
        if task_type in self.task_plans:
            self.current_plan = self.task_plans[task_type]
            self.current_step_index = 0
            self.planning_state = "EXECUTING"

            self.get_logger().info(f'Starting task: {task_type} with {len(self.current_plan)} steps')

            # Execute first step
            if self.current_plan:
                self.execute_next_step()
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            self.planning_state = "FAILED"

    def execute_next_step(self):
        """Execute the next step in the current plan."""
        if self.current_step_index >= len(self.current_plan):
            # Plan completed
            self.planning_state = "COMPLETED"
            self.publish_status(f"TASK_COMPLETED:{self.current_plan[0]['action']}")
            return

        step = self.current_plan[self.current_step_index]
        self.get_logger().info(f'Executing step {self.current_step_index + 1}: {step["action"]} {step.get("target", "")}')

        # Execute based on action type
        if step["action"] == "navigate":
            self.execute_navigation(step["target"])
        elif step["action"] == "find_object":
            self.execute_find_object(step["target"])
        elif step["action"] == "grasp":
            self.execute_grasp(step["target"])
        elif step["action"] == "place":
            self.execute_place(step["target"])
        elif step["action"] == "report_location":
            self.execute_report_location(step["target"])
        elif step["action"] == "collect_trash":
            self.execute_collect_trash(step["target"])
        elif step["action"] == "dispose_trash":
            self.execute_dispose_trash(step["target"])
        else:
            self.get_logger().error(f'Unknown action: {step["action"]}')
            self.planning_state = "FAILED"

    def execute_navigation(self, location):
        """Execute navigation to location."""
        if location in self.world_state["locations"]:
            loc_data = self.world_state["locations"][location]

            # Create and publish navigation goal
            goal_pose = Pose()
            goal_pose.position.x = loc_data["x"]
            goal_pose.position.y = loc_data["y"]
            goal_pose.position.z = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 1.0

            self.navigation_goal_pub.publish(goal_pose)

            # Update world state
            self.world_state["robot_location"] = location

            # Publish command
            cmd_msg = String()
            cmd_msg.data = f"NAVIGATE:{location}"
            self.low_level_command_pub.publish(cmd_msg)
        else:
            self.get_logger().error(f'Location not found: {location}')
            self.planning_state = "FAILED"

    def execute_find_object(self, target_object):
        """Execute object finding."""
        current_location = self.world_state["robot_location"]
        location_objects = self.world_state["locations"][current_location]["objects"]

        if target_object.lower() in [obj.lower() for obj in location_objects]:
            # Object found
            self.get_logger().info(f'Found {target_object} in {current_location}')

            cmd_msg = String()
            cmd_msg.data = f"FOUND_OBJECT:{target_object}_IN_{current_location.upper().replace(' ', '_')}"
            self.low_level_command_pub.publish(cmd_msg)
        else:
            # Object not in current location
            self.get_logger().info(f'{target_object} not found in {current_location}')

            # Could implement search in other locations
            cmd_msg = String()
            cmd_msg.data = f"OBJECT_NOT_FOUND:{target_object}"
            self.low_level_command_pub.publish(cmd_msg)
            self.planning_state = "FAILED"

    def execute_grasp(self, target_object):
        """Execute grasping action."""
        current_location = self.world_state["robot_location"]
        location_objects = self.world_state["locations"][current_location]["objects"]

        if target_object.lower() in [obj.lower() for obj in location_objects]:
            # Remove object from location and add to carried
            location_objects.remove(target_object.lower())
            self.world_state["carried_object"] = target_object

            cmd_msg = String()
            cmd_msg.data = f"GRASP_SUCCESS:{target_object}"
            self.low_level_command_pub.publish(cmd_msg)
        else:
            cmd_msg = String()
            cmd_msg.data = f"GRASP_FAILED:{target_object}_NOT_PRESENT"
            self.low_level_command_pub.publish(cmd_msg)
            self.planning_state = "FAILED"

    def execute_place(self, target_location):
        """Execute placing action."""
        if self.world_state["carried_object"]:
            carried_obj = self.world_state["carried_object"]

            # Add object to target location
            if target_location in self.world_state["locations"]:
                self.world_state["locations"][target_location]["objects"].append(carried_obj.lower())
                self.world_state["carried_object"] = None

                cmd_msg = String()
                cmd_msg.data = f"PLACE_SUCCESS:{carried_obj}_AT_{target_location.upper().replace(' ', '_')}"
                self.low_level_command_pub.publish(cmd_msg)
            else:
                cmd_msg = String()
                cmd_msg.data = f"PLACE_FAILED:UNKNOWN_LOCATION_{target_location}"
                self.low_level_command_pub.publish(cmd_msg)
                self.planning_state = "FAILED"
        else:
            cmd_msg = String()
            cmd_msg.data = f"PLACE_FAILED:NO_OBJECT_CARRIED"
            self.low_level_command_pub.publish(cmd_msg)
            self.planning_state = "FAILED"

    def feedback_callback(self, msg):
        """Process action feedback."""
        feedback = msg.data

        if feedback.startswith("ACTION_COMPLETED:"):
            # Move to next step
            self.current_step_index += 1
            self.execute_next_step()
        elif feedback.startswith("ACTION_FAILED:"):
            # Handle failure
            self.planning_state = "FAILED"
            self.publish_status(f"TASK_FAILED:{feedback}")
        elif feedback.startswith("NAVIGATION_COMPLETE"):
            # Navigation completed, move to next step
            self.current_step_index += 1
            self.execute_next_step()

    def publish_status(self, status):
        """Publish planning status."""
        status_msg = String()
        status_msg.data = status
        self.planning_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    planner = CognitivePlannerNode()

    # Example: trigger a high-level command
    # This would normally come from voice processing or high-level system
    def trigger_example():
        cmd_msg = String()
        cmd_msg.data = "BRING_COFFEE"
        planner.high_level_command_pub.publish(cmd_msg)

    # Schedule example after a delay
    planner.create_timer(2.0, trigger_example)

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## End-to-End VLA System Integration

Combining all components into a complete VLA system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from audio_common_msgs.msg import AudioData
import threading
import queue

class VLASystemNode(Node):
    def __init__(self):
        super().__init__('vla_system')

        # Initialize components
        self.whisper_node = WhisperSpeechToTextNode(self)
        self.llm_node = LLMBridgeNode(self)
        self.planner_node = CognitivePlannerNode(self)

        # Subscriptions for monitoring
        self.vla_status_sub = self.create_subscription(
            String, '/vla_system_status', self.status_callback, 10
        )

        # Publishers
        self.system_status_pub = self.create_publisher(String, '/vla_system_status', 10)

        # Internal queues for inter-component communication
        self.command_queue = queue.Queue()
        self.feedback_queue = queue.Queue()

        # System state
        self.system_state = "ACTIVE"  # ACTIVE, STANDBY, ERROR

        # Start processing threads
        self.processing_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.processing_thread.start()

        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_system)

    def status_callback(self, msg):
        """Receive status updates from components."""
        status = msg.data
        self.get_logger().info(f'Component status: {status}')

    def process_commands(self):
        """Process commands in separate thread."""
        while rclpy.ok() and self.system_state == "ACTIVE":
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get_nowait()
                    self.process_vla_command(command)
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'Error in command processing: {str(e)}')

    def process_vla_command(self, command):
        """Process a VLA command through the pipeline."""
        self.get_logger().info(f'Processing VLA command: {command}')

        # This would coordinate the flow:
        # 1. Speech to text (handled by whisper node)
        # 2. Language understanding (handled by llm node)
        # 3. Task planning (handled by planner node)
        # 4. Action execution (handled by robot controllers)

        # For now, just pass through
        pass

    def monitor_system(self):
        """Monitor system health."""
        # Check if all components are running
        # Publish system status
        status_msg = String()
        status_msg.data = f"VLA_SYSTEM_STATUS:{self.system_state}:COMPONENTS_OK"
        self.system_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    vla_system = VLASystemNode()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for VLA Systems

1. **Robust Speech Recognition**: Handle noisy environments and various accents
2. **Context Awareness**: Maintain context across multiple interactions
3. **Failure Handling**: Implement graceful degradation when components fail
4. **Safety First**: Ensure all actions are safe before execution
5. **Human-in-the-Loop**: Provide ways for humans to intervene

## Exercise

Create a complete VLA system that:
1. Accepts voice commands through Whisper
2. Interprets commands using an LLM
3. Plans multi-step tasks cognitively
4. Executes actions safely in simulation
5. Provides feedback and handles errors gracefully

## Summary

In this section, you learned:
- How to integrate Whisper for speech-to-text
- How to use LLMs for language understanding
- How to create cognitive planning systems
- How to combine all components into a VLA system
- Best practices for robust VLA implementations

In the next section, we'll explore projects that combine all VLA concepts.