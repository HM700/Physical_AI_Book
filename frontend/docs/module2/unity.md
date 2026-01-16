---
sidebar_position: 3
---

# Unity Simulation Environment

Unity is a powerful 3D development platform that can be used for creating high-fidelity robotic simulations. While traditionally used for gaming, Unity's physics engine and rendering capabilities make it excellent for robotics simulation.

## Introduction to Unity for Robotics

Unity provides several advantages for robotics simulation:
- High-fidelity graphics rendering
- Advanced physics simulation
- Rich asset ecosystem
- Cross-platform deployment
- VR/AR support

## Installing Unity for Robotics

Unity Robotics provides specialized tools and packages for robotics simulation:

1. Download Unity Hub from the Unity website
2. Install Unity Editor (2021.3 LTS or newer recommended)
3. Install the Unity Robotics Hub package
4. Import the Unity Robotics Simulation packages

## Unity Robotics Simulation (URS)

Unity Robotics Simulation (URS) bridges Unity with ROS 2, allowing:
- ROS 2 message publishing/subscribing
- Standard ROS 2 sensors (cameras, lidars, etc.)
- Physics simulation with Unity's engine
- Scene creation and environment design

### Setting up URS

1. Create a new Unity project
2. Import the ROS TCP Connector package
3. Import the Unity Robotics package
4. Configure the ROS connection settings

Example Unity C# script for ROS communication:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class UnityRobotController : MonoBehaviour
{
    ROSConnection ros;
    string rosTopicName = "unity_robot_cmd";

    // Robot movement parameters
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(rosTopicName);
    }

    void Update()
    {
        // Example: Send robot commands based on input
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Create a twist message
            TwistMsg twist = new TwistMsg();
            twist.linear = new Vector3Msg(linearSpeed, 0, 0);  // Move forward
            twist.angular = new Vector3Msg(0, 0, angularSpeed);  // Rotate

            // Publish the message
            ros.Publish(rosTopicName, twist);
        }
    }
}
```

## Creating Robot Models in Unity

Unity uses GameObjects with physics components to represent robots:

```csharp
using UnityEngine;

public class RobotModel : MonoBehaviour
{
    public Rigidbody baseBody;
    public WheelCollider[] wheels;

    [System.Serializable]
    public class WheelData
    {
        public Transform wheelTransform;
        public WheelCollider wheelCollider;
        public bool isMotorWheel;
        public bool isSteeringWheel;
    }

    public WheelData[] wheelData;

    void Start()
    {
        SetupWheels();
    }

    void SetupWheels()
    {
        foreach (var wheel in wheelData)
        {
            // Configure wheel collider properties
            wheel.wheelCollider.ConfigureVehicleSubsteps(5f, 24, 4);

            // Set up motor and steering parameters
            JointSpring spring = wheel.wheelCollider.suspensionSpring;
            spring.spring = 40000f;
            spring.damper = 4500f;
            spring.targetPosition = 0.5f;
            wheel.wheelCollider.suspensionSpring = spring;
        }
    }

    public void MoveRobot(float motorTorque, float steeringAngle)
    {
        foreach (var wheel in wheelData)
        {
            if (wheel.isMotorWheel)
            {
                wheel.wheelCollider.motorTorque = motorTorque;
            }

            if (wheel.isSteeringWheel)
            {
                wheel.wheelCollider.steerAngle = steeringAngle;
            }
        }
    }
}
```

## Adding Sensors to Unity Robots

Unity can simulate various sensors commonly used in robotics:

### Camera Sensor
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.Rendering;

public class UnityCameraSensor : MonoBehaviour
{
    public Camera sensorCamera;
    ROSConnection ros;
    string cameraTopic = "unity_camera";

    // Camera parameters
    public int imageWidth = 640;
    public int imageHeight = 480;

    RenderTexture renderTexture;
    Texture2D texture2D;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        sensorCamera.targetTexture = renderTexture;

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        // Capture image periodically
        if (Time.frameCount % 30 == 0) // Every 30 frames at 30 FPS = 1 Hz
        {
            // Render camera image to texture
            RenderTexture.active = renderTexture;
            texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
            texture2D.Apply();

            // Convert to bytes and publish
            byte[] imageData = texture2D.EncodeToPNG();

            // Create ROS image message (simplified - would need full implementation)
            // ImageMsg imgMsg = new ImageMsg();
            // imgMsg.width = (uint)imageWidth;
            // imgMsg.height = (uint)imageHeight;
            // imgMsg.data = imageData;
            // ros.Publish(cameraTopic, imgMsg);
        }
    }
}
```

## Unity-ROS Bridge Architecture

The Unity-ROS bridge works as follows:
1. Unity application connects to ROS network via TCP
2. Unity scripts publish/subscribe to ROS topics
3. Unity sensors output data in ROS message format
4. Unity physics engine simulates robot motion
5. ROS nodes can control Unity robots via topics/services

## Best Practices for Unity Robotics

- **Performance**: Optimize scenes for real-time simulation
- **Physics**: Use appropriate physics parameters for realistic behavior
- **Synchronization**: Manage timing between Unity and ROS clocks
- **Network**: Minimize network latency for responsive control
- **Validation**: Compare Unity simulation with real-world behavior

## Exercise

Create a Unity scene that:
1. Contains a simple robot model with wheels
2. Implements ROS communication for movement commands
3. Adds a camera sensor that publishes images to ROS
4. Creates a simple environment with obstacles
5. Implements basic collision detection

## Summary

In this section, you learned:
- How to set up Unity for robotics simulation
- How to create robot models in Unity
- How to add sensors to Unity robots
- How the Unity-ROS bridge works
- Best practices for Unity robotics

In the next section, we'll explore how to integrate Unity simulation with ROS 2 nodes.