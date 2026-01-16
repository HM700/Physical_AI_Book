---
sidebar_position: 3
---

# Perception Pipelines in Isaac Sim

Perception is a critical component of the AI-Robot brain, enabling robots to understand their environment. Isaac Sim provides advanced tools for developing and testing perception pipelines.

## Introduction to Perception in Robotics

Perception systems enable robots to:
- Detect and recognize objects
- Understand spatial relationships
- Estimate poses and trajectories
- Segment scenes into meaningful parts
- Process sensor data in real-time

## Isaac ROS Perception Components

Isaac ROS provides optimized perception components including:
- Stereo DNN
- Visual Slam
- Apriltag 3D
- ISAAC ROS Detection ROS
- ISAAC ROS Segmentation

### Stereo DNN Example

Stereo DNN provides real-time depth estimation:

```python
import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import torch

class StereoDNNProcessor(Node):
    def __init__(self):
        super().__init__('stereo_dnn_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions for stereo pair
        self.left_sub = self.create_subscription(
            Image, '/stereo/left/image_rect_color', self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            Image, '/stereo/right/image_rect_color', self.right_callback, 10
        )
        self.left_info_sub = self.create_subscription(
            CameraInfo, '/stereo/left/camera_info', self.left_info_callback, 10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo, '/stereo/right/camera_info', self.right_info_callback, 10
        )

        # Publisher for disparity map
        self.disparity_pub = self.create_publisher(DisparityImage, '/disparity_map', 10)

        # Store camera info
        self.left_info = None
        self.right_info = None

        # Initialize stereo matching algorithm
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    def left_callback(self, msg):
        """Process left camera image."""
        if self.right_image is not None:
            # Convert ROS images to OpenCV
            left_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            right_cv = self.bridge.imgmsg_to_cv2(self.right_image, desired_encoding='passthrough')

            # Convert to grayscale for stereo matching
            left_gray = cv2.cvtColor(left_cv, cv2.COLOR_RGB2GRAY)
            right_gray = cv2.cvtColor(right_cv, cv2.COLOR_RGB2GRAY)

            # Compute disparity
            disparity = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0

            # Create disparity message
            disp_msg = DisparityImage()
            disp_msg.header = msg.header
            disp_msg.image = self.bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
            disp_msg.f = self.left_info.K[0] if self.left_info else 1.0  # Focal length
            disp_msg.T = 0.1  # Baseline (example value)

            self.disparity_pub.publish(disp_msg)

    def right_callback(self, msg):
        """Store right camera image."""
        self.right_image = msg

    def left_info_callback(self, msg):
        """Store left camera info."""
        self.left_info = msg

    def right_info_callback(self, msg):
        """Store right camera info."""
        self.right_info = msg

def main(args=None):
    rclpy.init(args=args)
    processor = StereoDNNProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visual SLAM Implementation

Visual SLAM enables robots to simultaneously localize themselves and map their environment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_pose', 10)

        # Store camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # SLAM state
        self.previous_frame = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = []  # Store key poses
        self.map_points = []  # Store 3D map points

        # Feature detector
        self.detector = cv2.SIFT_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher()

        # Motion tracking
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming camera images for SLAM."""
        if self.camera_matrix is None:
            return  # Wait for camera info

        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        # Detect features
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)

        if self.previous_frame is not None and descriptors is not None:
            # Match features with previous frame
            matches = self.matcher.knnMatch(
                descriptors, self.previous_descriptors, k=2
            )

            # Apply Lowe's ratio test
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

            if len(good_matches) >= 10:  # Need sufficient matches
                # Extract matched points
                src_pts = np.float32([self.previous_keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # Estimate motion using Essential matrix
                E, mask = cv2.findEssentialMat(src_pts, dst_pts, self.camera_matrix,
                                              method=cv2.RANSAC, prob=0.999, threshold=1.0)

                if E is not None:
                    # Recover pose
                    _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)

                    # Update current pose
                    delta_T = np.eye(4)
                    delta_T[:3, :3] = R
                    delta_T[:3, 3] = t.flatten()

                    self.current_pose = self.current_pose @ delta_T

                    # Extract position and orientation
                    self.position = self.current_pose[:3, 3]

                    # Convert rotation matrix to quaternion
                    from tf_transformations import quaternion_from_matrix
                    self.orientation = quaternion_from_matrix(self.current_pose)

                    # Publish odometry
                    self.publish_odometry(msg.header)

        # Store current frame for next iteration
        self.previous_frame = gray
        self.previous_keypoints = keypoints
        self.previous_descriptors = descriptors

    def publish_odometry(self, header):
        """Publish odometry information."""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'camera_frame'

        # Set position
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])

        # Set orientation
        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Publish odometry
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    slam_node = VisualSLAMNode()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Object Detection Pipeline

Advanced object detection using Isaac Sim's synthetic data capabilities:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import torch
import torchvision
from torchvision import transforms
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # Load pre-trained model (YOLOv5, Detectron2, etc.)
        # In Isaac ROS, this would use optimized models
        self.model = self.load_detection_model()

        # Confidence threshold
        self.confidence_threshold = 0.5

        # Class names for Isaac Sim environment
        self.class_names = [
            'person', 'robot', 'table', 'chair', 'door',
            'box', 'cone', 'cylinder', 'cube', 'plane'
        ]

    def load_detection_model(self):
        """Load a pre-trained object detection model."""
        # In Isaac ROS, this would load an optimized model
        # For example, using Isaac ROS DNN detection
        # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # Placeholder for Isaac ROS optimized model
        # This would typically be a TensorRT optimized model
        return None

    def image_callback(self, msg):
        """Process image for object detection."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Preprocess image for detection
            input_tensor = self.preprocess_image(cv_image)

            # Perform detection
            with torch.no_grad():
                # detections = self.model(input_tensor)
                # Placeholder for Isaac ROS detection
                detections = self.fake_detection(cv_image)  # Replace with actual model call

            # Process detections
            detection_array = self.process_detections(detections, msg.header)

            # Publish detections
            self.detection_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {str(e)}')

    def preprocess_image(self, image):
        """Preprocess image for neural network."""
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((640, 640)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

        input_tensor = transform(image).unsqueeze(0)  # Add batch dimension
        return input_tensor

    def fake_detection(self, image):
        """Placeholder for actual detection - replace with Isaac ROS detection."""
        # This is a placeholder - in Isaac ROS, this would call the optimized detector
        # Return mock detections for now
        h, w = image.shape[:2]
        return [
            {
                'bbox': [int(w*0.3), int(h*0.3), int(w*0.2), int(h*0.2)],  # x, y, width, height
                'confidence': 0.8,
                'class_id': 1,  # robot
                'class_name': 'robot'
            }
        ]

    def process_detections(self, detections, header):
        """Convert detections to ROS message format."""
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            if det['confidence'] > self.confidence_threshold:
                detection_msg = Detection2D()

                # Bounding box
                bbox = det['bbox']
                detection_msg.bbox.center.x = float(bbox[0] + bbox[2]/2)  # center x
                detection_msg.bbox.center.y = float(bbox[1] + bbox[3]/2)  # center y
                detection_msg.bbox.size_x = float(bbox[2])  # width
                detection_msg.bbox.size_y = float(bbox[3])  # height

                # Classification
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(det['class_id'])
                hypothesis.hypothesis.score = det['confidence']

                detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Semantic Segmentation

Semantic segmentation provides pixel-level understanding of the environment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from PIL import Image as PILImage
import numpy as np

class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_segmentation')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.segmentation_pub = self.create_publisher(Image, '/segmentation', 10)

        # Load segmentation model
        self.model = self.load_segmentation_model()

        # Color map for visualization
        self.color_map = self.create_color_map()

    def load_segmentation_model(self):
        """Load a pre-trained segmentation model."""
        # In Isaac ROS, this would load an optimized segmentation model
        # For example: DeepLab, PSPNet, etc.
        # model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet50', pretrained=True)
        # model.eval()
        # return model

        # Placeholder for Isaac ROS optimized model
        return None

    def create_color_map(self):
        """Create a color map for different classes."""
        # Define colors for different object classes
        color_map = np.array([
            [128, 64, 128],    # road
            [244, 35, 232],    # sidewalk
            [70, 70, 70],      # building
            [102, 102, 156],   # wall
            [190, 153, 153],   # fence
            [153, 153, 153],   # pole
            [250, 170, 30],    # traffic light
            [220, 220, 0],     # traffic sign
            [107, 142, 35],    # vegetation
            [152, 251, 152],   # terrain
            [70, 130, 180],    # sky
            [220, 20, 60],     # person
            [255, 0, 0],       # rider
            [0, 0, 142],       # car
            [0, 0, 70],        # truck
            [0, 60, 100],      # bus
            [0, 80, 100],      # train
            [0, 0, 230],       # motorcycle
            [119, 11, 32],     # bicycle
        ], dtype=np.uint8)

        return color_map

    def image_callback(self, msg):
        """Process image for semantic segmentation."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Perform segmentation
            segmented_mask = self.perform_segmentation(cv_image)

            # Apply color map for visualization
            colored_segmentation = self.apply_color_map(segmented_mask)

            # Convert back to ROS image
            seg_msg = self.bridge.cv2_to_imgmsg(colored_segmentation, encoding='rgb8')
            seg_msg.header = msg.header

            # Publish segmentation
            self.segmentation_pub.publish(seg_msg)

        except Exception as e:
            self.get_logger().error(f'Error in semantic segmentation: {str(e)}')

    def perform_segmentation(self, image):
        """Perform semantic segmentation on the image."""
        # This is a placeholder - in Isaac ROS, this would call the optimized segmenter
        # Convert image to tensor and normalize
        input_tensor = transforms.ToTensor()(image).unsqueeze(0)

        # In Isaac ROS, this would be:
        # with torch.no_grad():
        #     outputs = self.model(input_tensor)
        #     predicted = torch.argmax(outputs['out'], dim=1)
        #     return predicted.squeeze().cpu().numpy()

        # For now, return a fake segmentation
        h, w = image.shape[:2]
        fake_mask = np.zeros((h, w), dtype=np.uint8)

        # Create some fake segmentation regions
        fake_mask[h//4:3*h//4, w//4:w//2] = 1  # Person region
        fake_mask[3*h//4:, :] = 2  # Road region
        fake_mask[:h//4, :] = 3  # Sky region

        return fake_mask

    def apply_color_map(self, mask):
        """Apply color map to segmentation mask."""
        # Map each class to its corresponding color
        h, w = mask.shape
        colored = np.zeros((h, w, 3), dtype=np.uint8)

        for class_id in np.unique(mask):
            colored[mask == class_id] = self.color_map[class_id % len(self.color_map)]

        return colored

def main(args=None):
    rclpy.init(args=args)
    seg_node = SemanticSegmentationNode()

    try:
        rclpy.spin(seg_node)
    except KeyboardInterrupt:
        pass
    finally:
        seg_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Perception Systems

1. **Robust Preprocessing**: Handle different lighting conditions and sensor noise
2. **Multi-Sensor Fusion**: Combine data from multiple sensors for better understanding
3. **Real-time Performance**: Optimize for real-time operation requirements
4. **Synthetic Data Training**: Use Isaac Sim's synthetic data generation for training
5. **Validation**: Compare synthetic-trained models with real-world performance

## Exercise

Create a complete perception pipeline that:
1. Subscribes to RGB and depth camera data
2. Performs object detection and semantic segmentation
3. Fuses sensor data for 3D object localization
4. Tracks objects over time
5. Publishes results in standard ROS 2 message formats

## Summary

In this section, you learned:
- How to implement stereo vision and depth estimation
- How to create visual SLAM systems
- How to build object detection pipelines
- How to implement semantic segmentation
- Best practices for robust perception systems

In the next section, we'll explore navigation and path planning in Isaac Sim.