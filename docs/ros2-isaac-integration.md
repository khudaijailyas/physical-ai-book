# Integrating ROS 2 with Isaac

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain how ROS 2 integrates with Isaac Sim and why this integration is valuable
- Set up and configure ROS 2 communication with Isaac Sim robots
- Publish sensor data from Isaac Sim to ROS 2 topics
- Send control commands from ROS 2 nodes to simulated robots
- Use ROS 2 navigation and manipulation packages with Isaac Sim
- Understand the bridge architecture connecting Isaac Sim and ROS 2
- Transfer ROS 2 code between simulation and real robots

## Concept Explanation

Integrating ROS 2 with Isaac Sim combines the power of industry-standard robotics software with photorealistic simulation and AI capabilities. This integration creates a unified workflow where you develop robot software once and use it in both simulation and on physical robots.

**Why ROS 2 and Isaac Sim Together**: ROS 2 (Robot Operating System 2) is the most widely used framework for building robot software. Isaac Sim is NVIDIA's advanced simulation platform with photorealistic rendering and GPU acceleration. Together, they provide an unbeatable combination: ROS 2's mature robotics libraries and ecosystem plus Isaac Sim's high-fidelity simulation and AI training capabilities.

**The Bridge Concept**: At the core of this integration is a "bridge" that connects the two systems. Think of it like a translator between two people speaking different languages. Isaac Sim speaks its own language (Omniverse and USD protocols), while ROS 2 uses its communication system (DDS - Data Distribution Service). The bridge translates between them, making Isaac Sim appear to ROS 2 as if it were a real robot.

**How the Bridge Works**: The bridge has two main responsibilities:

**Publishing Sensor Data**: When simulated sensors in Isaac Sim capture data—camera images, LIDAR scans, IMU readings—the bridge converts this data into ROS 2 messages and publishes them on ROS 2 topics. From a ROS 2 node's perspective, these messages look identical to messages from real sensors.

**Subscribing to Control Commands**: When your ROS 2 control nodes publish velocity commands, joint position commands, or other actuator instructions, the bridge receives these messages and translates them into actions in Isaac Sim, moving the simulated robot accordingly.

**Message Types and Topics**: ROS 2 organizes communication around topics and message types. Topics are named channels like `/camera/image` or `/cmd_vel`. Message types define the structure of data—`sensor_msgs/Image` for camera images, `geometry_msgs/Twist` for velocity commands, `sensor_msgs/LaserScan` for LIDAR data.

The bridge automatically handles standard message types. When you configure a camera in Isaac Sim with ROS 2 bridge enabled, it publishes `sensor_msgs/Image` messages. When you configure a differential drive robot, it subscribes to `geometry_msgs/Twist` messages for velocity control. This standardization means your ROS 2 code doesn't need to know it's talking to a simulator—it just uses standard ROS 2 interfaces.

**Action Servers and Services**: Beyond topics (continuous data streams), ROS 2 has actions (long-running tasks with feedback) and services (request-response interactions). The Isaac-ROS 2 bridge supports these too. You can send navigation goals as actions, request robot state as services, and receive progress feedback during long operations—all using standard ROS 2 patterns.

**Transform Frames (TF2)**: Robots have multiple coordinate frames—one for each sensor, joint, and link. ROS 2's TF2 system manages transformations between these frames. Isaac Sim publishes TF2 transforms for all robot components, so ROS 2 nodes can query "Where is the camera relative to the robot base?" or "What's the position of the gripper in world coordinates?" This coordinate management is crucial for sensor fusion and control.

**Isaac ROS Packages**: Beyond the basic bridge, NVIDIA provides Isaac ROS—a collection of ROS 2 packages optimized with GPU acceleration. These packages provide:

- **Perception**: Object detection, pose estimation, depth processing
- **Vision**: Image processing, feature detection, semantic segmentation  
- **SLAM**: Simultaneous localization and mapping
- **Navigation**: Path planning, obstacle avoidance

Isaac ROS packages run much faster than CPU-only alternatives because they leverage GPU parallelism. You can use them both in Isaac Sim for development and on real robots with NVIDIA Jetson computers for deployment.

**The Development Workflow**: The typical workflow using ROS 2 with Isaac Sim looks like this:

1. **Design**: Create your robot and environment in Isaac Sim
2. **Configure**: Enable ROS 2 bridge and configure topics/services
3. **Develop**: Write ROS 2 nodes that control the robot or process sensor data
4. **Test**: Run your ROS 2 nodes connected to Isaac Sim, testing thoroughly
5. **Deploy**: Take the same ROS 2 nodes and run them on a physical robot

The key insight is step 5—your code doesn't change between simulation and reality because both use the same ROS 2 interface.

**Launch Files**: ROS 2 uses launch files to start multiple nodes and configure systems. A typical launch file for Isaac Sim integration starts:
- Isaac Sim with the specified world and robot
- The ROS 2 bridge
- Your custom ROS 2 nodes (navigation, perception, control)
- Visualization tools like RViz
- Any other required systems

Launch files make complex system startup reproducible and easy.

**RViz Visualization**: RViz is ROS 2's standard visualization tool. It displays sensor data, robot models, transforms, paths, and more. When using Isaac Sim with ROS 2, you can visualize everything in RViz while the simulation runs in Isaac Sim. This dual visualization is powerful—Isaac Sim shows photorealistic rendering, RViz shows the data structure and processing that ROS 2 sees.

**Parameter Management**: ROS 2 nodes have parameters (configuration values) that can be set from launch files or changed at runtime. The Isaac-ROS 2 bridge exposes many Isaac Sim settings as ROS 2 parameters, so you can configure simulation behavior through standard ROS 2 tools without editing Isaac Sim files directly.

**Time Synchronization**: Simulations can run faster or slower than real-time. ROS 2 needs to know simulation time, not wall-clock time, for correct behavior. The bridge publishes simulation time on `/clock`, and ROS 2 nodes use this simulated time for timestamps and time-based logic. This ensures that algorithms behave correctly whether simulation runs fast, slow, or real-time.

**Domain IDs and Multiple Simulations**: ROS 2 uses domain IDs to separate different robot systems. You can run multiple Isaac Sim instances, each with different domain IDs, and they won't interfere with each other. This is useful for testing multi-robot scenarios or running multiple independent simulations on the same network.

**Quality of Service (QoS)**: ROS 2's QoS settings control message delivery reliability, durability, and deadlines. Different topics need different QoS—sensor data often uses "best effort" (fast but may drop messages) while command data uses "reliable" (guaranteed delivery). The bridge configures appropriate QoS for standard sensor and control topics, but you can customize these settings.

**Custom Message Types**: While the bridge handles standard messages automatically, you can also use custom message types. Define your message in a ROS 2 package, compile it, and the bridge can publish or subscribe to those custom messages. This flexibility lets you extend beyond standard interfaces when needed.

## Why This Matters

Understanding ROS 2 integration with Isaac Sim matters profoundly because it represents how professional robotics development happens today and provides capabilities unavailable in either system alone.

**Industry Standard Workflow**: The combination of ROS 2 and simulation is the industry standard for robotics development. Companies from small startups to major corporations (Amazon Robotics, BMW, Boston Dynamics, and many others) use ROS with simulation. Learning this integrated workflow isn't just academic—it's learning how robots are actually built professionally. Job postings for robotics engineers commonly require both ROS 2 and simulation experience.

**True Sim-to-Real Transfer**: The biggest promise of this integration is sim-to-real transfer that actually works. You write ROS 2 code, test it exhaustively in Isaac Sim, and deploy the same code to real robots. Because both use identical ROS 2 interfaces, the code literally doesn't change. You might tune some parameters (PID gains, thresholds) based on real-world behavior, but the fundamental software architecture and logic remain the same. This dramatically reduces development risk and time.

**Leveraging the ROS 2 Ecosystem**: ROS 2 has thousands of packages providing functionality for navigation, manipulation, perception, planning, and more. Many of these packages represent years of development by robotics experts. By integrating Isaac Sim with ROS 2, you immediately access this ecosystem. Need a sophisticated navigation stack? Use Nav2. Need to control a robot arm? Use MoveIt2. These proven, mature packages work in Isaac Sim just as they work on real robots.

**GPU-Accelerated Performance**: Isaac ROS packages provide GPU-accelerated implementations of common robotics algorithms. Vision processing that takes 100ms on a CPU might take 10ms on a GPU. For real-time robotics where reaction time matters, this 10x speedup can make the difference between possible and impossible. You develop with these fast implementations in simulation and deploy them to Jetson-equipped robots that maintain the same performance.

**Safe Development of Dangerous Behaviors**: Some robot behaviors are dangerous to develop on physical hardware. A robot learning to grasp at high speed might damage itself or surroundings through mistakes. An autonomous vehicle testing emergency maneuvers could cause accidents. Isaac Sim lets you develop these behaviors safely, using ROS 2 code that will transfer to real hardware once proven safe. You can simulate thousands of edge cases—sensor failures, mechanical failures, extreme conditions—without risk.

**Multi-Robot and Swarm Development**: Testing multi-robot systems with physical robots is expensive and logistically complex. You need multiple robots, space to operate them, and infrastructure to coordinate them. In Isaac Sim with ROS 2, you can simulate dozens of robots sharing the environment and communicating through ROS 2 topics. You develop coordination algorithms, test communication protocols, and verify system-wide behaviors—all before building a physical fleet.

**Continuous Integration and Testing**: Professional software development uses CI/CD (Continuous Integration/Continuous Deployment) where code changes trigger automatic testing. With Isaac Sim and ROS 2 integration, you can create automated tests that start Isaac Sim, launch your ROS 2 nodes, run test scenarios, and report results—all without human intervention. Every code commit is automatically tested in hundreds of simulated scenarios, catching bugs before they reach physical robots.

**Educational Accessibility**: Not everyone has access to expensive robot hardware. A student learning robotics might not be able to afford a $5,000 research robot. But with Isaac Sim (free for educational use) and ROS 2 (free and open source), that student can develop the exact same software professionals use, testing it on virtual versions of real robots. This democratizes robotics education, making world-class learning accessible worldwide.

**Rapid Prototyping and Iteration**: Want to test a new control algorithm? Modify your ROS 2 node, restart it, and test in seconds. Want to try a different sensor configuration? Reconfigure Isaac Sim and run the same ROS 2 code with the new sensors. This rapid iteration is impossible with physical hardware where changes might require hours of mechanical assembly or hardware procurement. The integration enables true agile development for robotics.

**Data Generation at Scale**: Training modern AI perception systems requires massive datasets—millions of labeled images. Collecting this data from real robots is prohibitively expensive and time-consuming. Isaac Sim with ROS 2 can generate and publish this data automatically. Run thousands of scenarios with randomized conditions, collect sensor data via ROS 2 topics, and store it for AI training. What would take months with real robots happens overnight in simulation.

**Team Collaboration**: In robotics teams, different people work on different subsystems—one person on perception, another on planning, another on control. With ROS 2 integration, these team members can work independently. The perception developer tests their ROS 2 perception node with Isaac Sim cameras. The planning developer tests their ROS 2 planning node with simulated maps. Later, the subsystems integrate seamlessly because they all use standard ROS 2 interfaces. The integration enables parallel development.

**Cost Reduction**: Physical robot testing is expensive. Robots break, consume power, require maintenance, and need supervised operation. Isaac Sim simulation costs only the electricity to run your computer. You can test 24/7 without supervision. For a company developing robots, this cost difference is substantial—potentially millions of dollars saved during development. The ROS 2 integration ensures that this cheaper simulation testing is actually valuable because code transfers to real hardware.

## Example

Let's work through a comprehensive example where you create a complete ROS 2-based robot system in Isaac Sim: a mobile manipulator that navigates to objects, identifies them using vision, and picks them up.

**The Goal**: Build a robot that receives high-level goals like "Pick up the red box in the warehouse," navigates to the location, identifies the box using a camera, plans a grasp, and executes the pickup—all using ROS 2 nodes that could deploy to a real robot.

**Step 1: Setting Up the Environment**

First, ensure you have both Isaac Sim and ROS 2 installed. For this example, we'll assume ROS 2 Humble (a popular long-term support version). Verify installation:

```bash
# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --version

# Launch Isaac Sim (method depends on your installation)
# Typically: ~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

**Step 2: Creating the Robot in Isaac Sim**

Open Isaac Sim and create a new scene. Build a warehouse environment with:
- Floor (20m x 20m)
- Walls around perimeter
- Several shelving units
- Various colored boxes scattered around

Now add the robot. Isaac Sim includes several pre-configured robots. For this example, use a mobile manipulator like the Carter robot with an attached UR10 arm:

1. In the content browser, navigate to Isaac → Robots → Carter
2. Drag Carter into the scene
3. Add a UR10 arm from Isaac → Robots → UR10
4. Position the UR10 on top of Carter to create a mobile manipulator

**Step 3: Configuring Sensors**

Add sensors to the robot:

**Navigation Camera**: Mount an RGB-D camera on the front for obstacle detection
- Select Carter's chassis
- Add → Camera → RGB-D Camera
- Position: (0.3, 0.2, 0) facing forward
- Configure resolution: 640x480
- Enable depth output

**Manipulation Camera**: Mount a camera on the arm's end effector to see objects
- Select the UR10's wrist link
- Add → Camera → RGB Camera  
- Position: (0, 0, 0.1) facing down
- Resolution: 1280x720

**LIDAR**: Add 2D LIDAR for navigation
- Select Carter's chassis
- Add → LIDAR → Rotating LIDAR
- Position: (0, 0.3, 0) on top
- Configure: 360° scan, 10m range, 1° resolution

**Step 4: Enabling ROS 2 Bridge**

This is where the magic happens. Enable the ROS 2 bridge for your robot:

In Isaac Sim's menu:
- Go to Isaac Utils → ROS2 Bridge
- Enable the bridge
- Configure domain ID (default 0 is usually fine)

For each sensor, enable ROS 2 publishing:
- Select the navigation camera
- In properties, find ROS 2 settings
- Enable "Publish Image" on topic `/carter/camera/image`
- Enable "Publish Depth" on topic `/carter/camera/depth`
- Enable "Publish Camera Info" on topic `/carter/camera/camera_info`

Similarly configure:
- LIDAR to publish on `/carter/scan`
- Manipulation camera on `/arm/camera/image`
- Robot joint states on `/joint_states`
- TF transforms (automatically published)

For control:
- Configure Carter to subscribe to `/cmd_vel` for velocity commands
- Configure UR10 to subscribe to `/arm/joint_trajectory` for arm control

**Step 5: Testing the Bridge**

Start the simulation (Play button). The ROS 2 bridge activates automatically.

In a terminal, verify ROS 2 communication:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# List active topics
ros2 topic list
# You should see:
#   /carter/camera/image
#   /carter/camera/depth
#   /carter/scan
#   /cmd_vel
#   /joint_states
#   /tf
#   ... and more

# Echo a topic to see data
ros2 topic echo /carter/scan --once

# Send a test command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

When you publish to `/cmd_vel`, the simulated robot should move forward in Isaac Sim! This confirms the bridge works.

**Step 6: Creating a ROS 2 Navigation Node**

Create a ROS 2 workspace and navigation package:

```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
ros2 pkg create warehouse_navigator --build-type ament_python --dependencies rclpy geometry_msgs sensor_msgs nav_msgs
```

Create a navigation node (`warehouse_navigator/navigator.py`):

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class WarehouseNavigator(Node):
    def __init__(self):
        super().__init__('warehouse_navigator')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/carter/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.scan_data = None
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigator initialized')
    
    def scan_callback(self, msg):
        """Receive LIDAR data"""
        self.scan_data = msg
    
    def odom_callback(self, msg):
        """Receive odometry (position) data"""
        self.current_pose = msg.pose.pose
    
    def set_goal(self, x, y):
        """Set navigation goal"""
        self.goal_pose = (x, y)
        self.get_logger().info(f'Goal set to ({x}, {y})')
    
    def control_loop(self):
        """Main control loop"""
        if not self.goal_pose or not self.current_pose or not self.scan_data:
            return
        
        # Calculate error to goal
        dx = self.goal_pose[0] - self.current_pose.position.x
        dy = self.goal_pose[1] - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if reached goal
        if distance < 0.2:  # 20cm tolerance
            self.get_logger().info('Goal reached!')
            self.stop()
            self.goal_pose = None
            return
        
        # Calculate desired heading
        desired_angle = math.atan2(dy, dx)
        
        # Get current heading from quaternion
        current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Calculate angular error
        angle_error = desired_angle - current_angle
        # Normalize to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Check for obstacles
        min_distance = self.check_obstacles()
        
        # Generate control commands
        cmd = Twist()
        
        if min_distance < 0.5:  # Obstacle within 50cm
            # Emergency stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if angle_error > 0 else -0.5
        elif abs(angle_error) > 0.3:  # Need to turn
            # Turn in place
            cmd.linear.x = 0.1
            cmd.angular.z = 1.0 * angle_error
        else:  # Mostly aligned
            # Move forward
            cmd.linear.x = min(0.5, distance)  # Slow down as approaching
            cmd.angular.z = 0.5 * angle_error
        
        self.cmd_pub.publish(cmd)
    
    def check_obstacles(self):
        """Check LIDAR for obstacles ahead"""
        if not self.scan_data:
            return float('inf')
        
        # Check front 60 degrees
        ranges = self.scan_data.ranges
        front_ranges = ranges[-30:] + ranges[:30]
        return min(front_ranges)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def stop(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    navigator = WarehouseNavigator()
    
    # Set a test goal
    navigator.set_goal(5.0, 3.0)
    
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Build and run:

```bash
cd ~/isaac_ros_ws
colcon build
source install/setup.bash
ros2 run warehouse_navigator navigator
```

With Isaac Sim running and the bridge active, your navigation node receives LIDAR and odometry data, computes control commands, and sends them back to Isaac Sim. Watch the robot navigate to the goal!

**Step 7: Adding Vision-Based Object Detection**

Create a perception node that uses the manipulation camera:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/arm/camera/image', self.image_callback, 10)
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            DetectionArray, '/detections', 10)
        
        self.get_logger().info('Object detector initialized')
    
    def image_callback(self, msg):
        """Process camera images"""
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Simple color-based detection (red objects)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Red color range
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest = max(contours, key=cv2.contourArea)
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(largest)
            
            # Calculate center
            center_x = x + w/2
            center_y = y + h/2
            
            self.get_logger().info(
                f'Red object detected at ({center_x}, {center_y})')
            
            # Publish detection (implementation depends on your message type)
            # This would trigger the manipulation behavior

def main():
    rclpy.init()
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
```

**Step 8: Adding Manipulation Control**

Create an arm controller node:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Publisher for arm commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm/joint_trajectory', 10)
        
        # Subscriber for current joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.current_joints = None
        
        self.get_logger().info('Arm controller initialized')
    
    def joint_callback(self, msg):
        """Receive current joint positions"""
        self.current_joints = msg.position
    
    def move_to_pose(self, joint_positions, duration=2.0):
        """Move arm to specified joint positions"""
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        traj.points.append(point)
        
        self.trajectory_pub.publish(traj)
        self.get_logger().info('Moving arm to target pose')
    
    def pick_object(self, x, y, z):
        """
        Execute pick sequence for object at position (x, y, z)
        In reality, this would use inverse kinematics
        """
        # Pre-grasp position (above object)
        self.move_to_home()
        self.get_logger().info('Moving to pre-grasp')
        # Wait...
        
        # Approach
        self.get_logger().info('Approaching object')
        # Calculate IK and move...
        
        # Grasp
        self.close_gripper()
        
        # Lift
        self.get_logger().info('Lifting object')
        
        # Return to home
        self.move_to_home()
    
    def move_to_home(self):
        """Move to home position"""
        home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.move_to_pose(home_joints)
    
    def close_gripper(self):
        """Close gripper (implementation specific)"""
        # Send gripper command
        pass

def main():
    rclpy.init()
    controller = ArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

**Step 9: Coordinating with a State Machine**

Create a high-level coordinator that manages the task:

```python
import rclpy
from rclpy.node import Node
from enum import Enum

class TaskState(Enum):
    NAVIGATING = 1
    SEARCHING = 2
    APPROACHING = 3
    GRASPING = 4
    RETURNING = 5
    DONE = 6

class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        
        self.state = TaskState.NAVIGATING
        self.target_location = (5.0, 3.0)
        
        # Create client connections to other nodes
        # (In practice, use action clients for better control)
        
        self.timer = self.create_timer(1.0, self.state_machine)
        
        self.get_logger().info('Task coordinator initialized')
    
    def state_machine(self):
        """Execute state machine"""
        if self.state == TaskState.NAVIGATING:
            self.get_logger().info('State: Navigating to target')
            # Command navigator to go to location
            # When navigator reports arrival, transition to SEARCHING
            
        elif self.state == TaskState.SEARCHING:
            self.get_logger().info('State: Searching for object')
            # Monitor object detector
            # When object detected, transition to APPROACHING
            
        elif self.state == TaskState.APPROACHING:
            self.get_logger().info('State: Approaching object')
            # Fine position adjustment
            # Transition to GRASPING when positioned
            
        elif self.state == TaskState.GRASPING:
            self.get_logger().info('State: Grasping object')
            # Command arm controller to pick
            # Transition to RETURNING when grasped
            
        elif self.state == TaskState.RETURNING:
            self.get_logger().info('State: Returning to start')
            # Navigate back
            # Transition to DONE when returned
            
        elif self.state == TaskState.DONE:
            self.get_logger().info('Task complete!')
            self.timer.cancel()

def main():
    rclpy.init()
    coordinator = TaskCoordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()
```

**Step 10: Using RViz for Visualization**

Launch RViz to visualize the system:

```bash
rviz2
```

In RViz, add displays:
- **RobotModel**: Shows the robot structure
- **TF**: Shows coordinate frames
- **LaserScan**: Visualizes LIDAR data (topic: `/carter/scan`)
- **Image**: Shows camera feed (topic: `/arm/camera/image`)
- **Map**: If you add mapping, shows the map
- **Path**: If you add path planning, shows planned paths

Now you have dual visualization: Isaac Sim shows photorealistic rendering, RViz shows the data and processing structure that ROS 2 sees.

**Step 11: Creating a Launch File**

Tie everything together with a launch file (`warehouse_mission.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Note: Isaac Sim typically started separately
        
        # Navigation node
        Node(
            package='warehouse_navigator',
            executable='navigator',
            name='navigator',
            output='screen'
        ),
        
        # Object detection node
        Node(
            package='warehouse_navigator',
            executable='detector',
            name='object_detector',
            output='screen'
        ),
        
        # Arm controller node
        Node(
            package='warehouse_navigator',
            executable='arm_controller',
            name='arm_controller',
            output='screen'
        ),
        
        # Task coordinator
        Node(
            package='warehouse_navigator',
            executable='coordinator',
            name='task_coordinator',
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/warehouse.rviz']
        ),
    ])
```

Launch everything:

```bash
ros2 launch warehouse_navigator warehouse_mission.launch.py
```

**Step 12: Testing and Iteration**

With the complete system running:

1. **Test individual components**: Verify each node works independently
2. **Test integration**: Run the complete pipeline
3. **Test edge cases**: Object in different positions, obstacles in path, sensor noise
4. **Collect metrics**: Success rate, time to completion, path efficiency
5. **Iterate**: Improve algorithms based on simulation results

**Step 13: Deploying to Real Hardware**

When ready to deploy:

1. **Prepare physical robot**: Ensure it has the same sensors (camera, LIDAR)
2. **Configure ROS 2**: Same topic