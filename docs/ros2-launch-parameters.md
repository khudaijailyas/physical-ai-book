# Chapter 5: Launch Files & Parameter Management

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what launch files are and why they're essential
- Create Python launch files to start multiple nodes
- Pass arguments to launch files for flexibility
- Declare and use parameters in ROS 2 nodes
- Load parameters from YAML configuration files
- Override parameters at runtime
- Use parameters to configure node behavior without changing code
- Organize launch files for complex robot systems

## Concept Explanation

### What are Launch Files?

Imagine you're building a robot with 10 different nodes: camera, object detector, navigation, motor controller, safety monitor, battery tracker, and more. Without launch files, starting your robot looks like this:

**Terminal 1**: `ros2 run camera_pkg camera_node`  
**Terminal 2**: `ros2 run vision_pkg detector_node`  
**Terminal 3**: `ros2 run nav_pkg navigation_node`  
**Terminal 4**: `ros2 run control_pkg motor_controller`  
...and 6 more terminals!

This is tedious, error-prone, and impossible to manage as your robot grows.

**Launch files solve this problem**. They let you start all your nodes with ONE command:

```bash
ros2 launch my_robot_pkg robot.launch.py
```

Everything starts automatically, in the right order, with the right configuration.

### What Launch Files Do

Launch files are Python scripts (in ROS 2) that:

1. **Start multiple nodes** with one command
2. **Configure nodes** with parameters
3. **Set up namespaces** to organize nodes
4. **Remap topics** to change topic names
5. **Include other launch files** for modularity
6. **Add conditional logic** (start nodes only if conditions are met)
7. **Set environment variables**
8. **Group related nodes** together

**Think of launch files as a recipe**: They tell ROS 2 exactly what to start, how to configure it, and in what order.

### Basic Launch File Structure

A simple launch file looks like this:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name'
        ),
    ])
```

**Let's break this down**:

**Imports**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
```
- `LaunchDescription`: Container for everything you want to launch
- `Node`: Represents a ROS 2 node to start

**Main function**:
```python
def generate_launch_description():
```
This function must be named exactly `generate_launch_description()`. ROS 2 looks for this function when you run the launch file.

**Return LaunchDescription**:
```python
return LaunchDescription([...])
```
You return a `LaunchDescription` containing a list of things to launch (nodes, parameters, etc.).

**Node definition**:
```python
Node(
    package='my_package',      # Package containing the node
    executable='my_node',       # Entry point name from setup.py
    name='my_node_name'        # Name for this node instance
)
```

### Launching Multiple Nodes

The power of launch files: starting many nodes at once.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='camera_node',
            name='front_camera'
        ),
        Node(
            package='vision_pkg',
            executable='detector_node',
            name='object_detector'
        ),
        Node(
            package='control_pkg',
            executable='controller_node',
            name='robot_controller'
        ),
    ])
```

This starts three nodes with one command. Simple!

### What are Parameters?

**Parameters** are configuration values that you can change without modifying your code. Think of them like settings in an app—you can adjust brightness, volume, or language without reinstalling the app.

**Why parameters matter**:

Imagine you have a camera node. Without parameters:
```python
# Hard-coded values in the code
image_width = 1920
image_height = 1080
frame_rate = 30
```

If you want different settings, you must:
1. Edit the source code
2. Rebuild the package
3. Re-source the workspace
4. Restart the node

**With parameters**:
```python
# Configurable values
image_width = self.get_parameter('image_width').value
image_height = self.get_parameter('image_height').value
frame_rate = self.get_parameter('frame_rate').value
```

Now you can change settings without touching the code!

### Declaring Parameters in Nodes

To use parameters in a node, you must **declare** them first.

**Basic parameter declaration**:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('debug_mode', False)
        
        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        debug_mode = self.get_parameter('debug_mode').value
        
        self.get_logger().info(f'Robot: {robot_name}')
        self.get_logger().info(f'Max speed: {max_speed} m/s')
        self.get_logger().info(f'Debug mode: {debug_mode}')
```

**Two-step process**:
1. **Declare**: Tell ROS 2 this parameter exists and its default value
2. **Get**: Retrieve the parameter value to use in your code

### Parameter Types

ROS 2 supports several parameter types:

```python
# String parameter
self.declare_parameter('robot_name', 'default_robot')

# Integer parameter
self.declare_parameter('sensor_count', 5)

# Float parameter
self.declare_parameter('max_speed', 2.5)

# Boolean parameter
self.declare_parameter('enable_safety', True)

# List parameter
self.declare_parameter('sensor_ids', [1, 2, 3, 4, 5])

# String array parameter
self.declare_parameter('camera_names', ['front', 'back', 'left', 'right'])
```

### Setting Parameters at Launch

You can set parameters when launching nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{
                'robot_name': 'explorer_1',
                'max_speed': 3.0,
                'debug_mode': True
            }]
        ),
    ])
```

Now the node starts with these specific parameter values instead of defaults.

### Loading Parameters from YAML Files

For many parameters, it's cleaner to use a YAML configuration file.

**Create a YAML file**: `config/robot_params.yaml`

```yaml
/**:
  ros__parameters:
    robot_name: "explorer_1"
    max_speed: 2.5
    min_speed: 0.1
    debug_mode: false
    sensor_ids: [1, 2, 3, 4]
    camera_names: ["front", "back", "left", "right"]
```

**Load in launch file**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to config file
    config = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'robot_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[config]
        ),
    ])
```

All parameters from the YAML file are automatically loaded!

### Launch Arguments

**Launch arguments** let you pass values when you run the launch file, making it flexible.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )
    
    # Use the argument value
    return LaunchDescription([
        robot_name_arg,
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }]
        ),
    ])
```

**Run with custom value**:
```bash
ros2 launch my_package my_launch.py robot_name:=explorer
```

**Run with default value**:
```bash
ros2 launch my_package my_launch.py
```

### Remapping Topics

Sometimes you need to change topic names without modifying code.

```python
Node(
    package='camera_pkg',
    executable='camera_node',
    name='front_camera',
    remappings=[
        ('/image', '/front/camera/image'),
        ('/info', '/front/camera/info')
    ]
)
```

The node publishes to `/image`, but launch file remaps it to `/front/camera/image`.

### Namespaces

**Namespaces** group related nodes and topics together.

```python
Node(
    package='camera_pkg',
    executable='camera_node',
    name='camera',
    namespace='robot1'
)
```

This creates:
- Node name: `/robot1/camera`
- Topics: `/robot1/image`, `/robot1/info`, etc.

**Why use namespaces?** Great for multi-robot systems:
```python
# Robot 1
Node(..., namespace='robot1')

# Robot 2
Node(..., namespace='robot2')
```

Now each robot has its own set of topics:
- Robot 1: `/robot1/camera/image`
- Robot 2: `/robot2/camera/image`

### Including Other Launch Files

For large systems, split launch files into smaller pieces:

**main_robot.launch.py**:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sensor_pkg'), 'launch'),
            '/sensors.launch.py'
        ])
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav_pkg'), 'launch'),
            '/navigation.launch.py'
        ])
    )
    
    return LaunchDescription([
        sensors_launch,
        navigation_launch,
    ])
```

This includes two other launch files, keeping things organized.

### Conditional Launch

Start nodes only when certain conditions are met:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false'
    )
    
    return LaunchDescription([
        use_sim_arg,
        
        # Only start simulator if use_sim is true
        Node(
            package='simulator_pkg',
            executable='simulator_node',
            name='simulator',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
        
        # Always start this node
        Node(
            package='control_pkg',
            executable='controller',
            name='controller'
        ),
    ])
```

**Usage**:
```bash
# Without simulator
ros2 launch my_package robot.launch.py

# With simulator
ros2 launch my_package robot.launch.py use_sim:=true
```

### Organizing Launch Files in Packages

Launch files go in the `launch/` folder of your package:

```
my_package/
├── launch/
│   ├── robot.launch.py
│   ├── sensors.launch.py
│   └── navigation.launch.py
├── config/
│   ├── robot_params.yaml
│   └── sensor_params.yaml
├── my_package/
│   └── (Python code)
└── setup.py
```

**Update setup.py** to install launch files:

```python
from setuptools import setup
import os
from glob import glob

setup(
    name='my_package',
    # ... other fields ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    # ... rest of setup ...
)
```

After rebuilding, you can run:
```bash
ros2 launch my_package robot.launch.py
```

### Parameter Callbacks

Sometimes you want to react when parameters change at runtime:

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameter with descriptor
        self.declare_parameter(
            'max_speed',
            2.0,
            ParameterDescriptor(description='Maximum speed in m/s')
        )
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.max_speed = self.get_parameter('max_speed').value
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                self.max_speed = param.value
                self.get_logger().info(f'Max speed updated to: {self.max_speed}')
        return SetParametersResult(successful=True)
```

Now if someone changes the parameter at runtime, your node reacts:

```bash
ros2 param set /my_node max_speed 3.0
```

## Why This Matters

### Real-World Impact

Launch files and parameters are essential for professional robotics:

**1. System Complexity Management**

Real robots have dozens or hundreds of nodes. Launch files make them manageable:
- **Small robot**: 10-20 nodes
- **Warehouse robot**: 50-100 nodes  
- **Autonomous vehicle**: 100-300 nodes
- **Humanoid robot**: 200-500 nodes

Without launch files, these systems would be impossible to start and configure.

**2. Configuration Without Recompilation**

Parameters let you adjust behavior instantly:
- **Testing**: Try different sensor frequencies without rebuilding
- **Tuning**: Adjust control parameters to optimize performance
- **Deployment**: Use different settings for different environments

**Real example**: A delivery robot might use:
- `max_speed: 2.0` in crowded areas
- `max_speed: 5.0` in open spaces
- `debug_mode: true` during development
- `debug_mode: false` in production

Change parameters, not code!

**3. Multi-Environment Support**

Same code, different configurations:
```bash
# Development with simulator
ros2 launch robot.launch.py use_sim:=true debug:=true

# Testing with real hardware
ros2 launch robot.launch.py use_sim:=false debug:=true

# Production deployment
ros2 launch robot.launch.py use_sim:=false debug:=false
```

**4. Team Collaboration**

Launch files enable team workflows:
- **Perception team**: Works on `sensors.launch.py`
- **Planning team**: Works on `navigation.launch.py`
- **Control team**: Works on `control.launch.py`
- **Integration**: Main `robot.launch.py` includes all of them

Everyone works independently, integrates easily.

**5. Documentation and Reproducibility**

Launch files document your system:
- What nodes run together?
- What parameters are set?
- What's the system configuration?

Six months later, you (or a teammate) can understand the system by reading launch files.

### Industry Standards

Professional robotics companies use launch files extensively:

**Manufacturing**: Robot cells with multiple coordinated robots, each with its own namespace and parameters

**Autonomous vehicles**: Complex sensor fusion systems with dozens of nodes launched together

**Service robots**: Different configurations for different deployment sites (hospital vs. office vs. warehouse)

**Research labs**: Quickly switch between experimental configurations

### Cost Savings

Parameters save development time and money:

**Without parameters**:
- Adjust code → Rebuild → Test → Repeat
- Each iteration: 2-5 minutes
- 20 iterations per day: 40-100 minutes wasted

**With parameters**:
- Adjust parameter → Test immediately
- Each iteration: 5 seconds
- Save 30+ minutes per day per developer

For a team of 5 developers over a year: **Hundreds of hours saved**!

## Example

Let's build a complete example: **A configurable mobile robot system** with sensors, navigation, and telemetry.

### Goal

Create a launch system for a robot with:
1. Multiple camera nodes (front, back)
2. LIDAR node
3. Navigation node
4. Telemetry node (sends status updates)
5. All nodes configurable via parameters
6. Different launch configurations for simulation vs. real robot

### Step 1: Create the Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_bringup \
    --dependencies rclpy std_msgs sensor_msgs
```

Create folders:
```bash
cd robot_bringup
mkdir -p launch config
```

Structure:
```
robot_bringup/
├── launch/
│   ├── robot.launch.py
│   ├── sensors.launch.py
│   └── navigation.launch.py
├── config/
│   ├── robot_params.yaml
│   └── sim_params.yaml
├── robot_bringup/
│   ├── __init__.py
│   ├── camera_node.py
│   ├── lidar_node.py
│   ├── navigation_node.py
│   └── telemetry_node.py
└── setup.py
```

### Step 2: Create a Parameterized Camera Node

**File**: `robot_bringup/robot_bringup/camera_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('camera_name', 'default_camera')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('resolution', '1920x1080')
        self.declare_parameter('enable_processing', True)
        
        # Get parameter values
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.resolution = self.get_parameter('resolution').value
        self.enable_processing = self.get_parameter('enable_processing').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            f'{self.camera_name}/status',
            10
        )
        
        # Create timer based on frame rate
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        
        self.get_logger().info(f'Camera node started: {self.camera_name}')
        self.get_logger().info(f'Resolution: {self.resolution}')
        self.get_logger().info(f'Frame rate: {self.frame_rate} Hz')
        self.get_logger().info(f'Processing: {self.enable_processing}')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'{self.camera_name}: Frame {self.frame_count}'
        self.publisher.publish(msg)
        self.frame_count += 1
        
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'{self.camera_name}: {self.frame_count} frames captured')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create LIDAR Node with Parameters

**File**: `robot_bringup/robot_bringup/lidar_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Declare parameters
        self.declare_parameter('scan_frequency', 10)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('angle_range', 360)
        
        # Get parameters
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_range = self.get_parameter('angle_range').value
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'lidar/scan', 10)
        
        # Create timer
        timer_period = 1.0 / self.scan_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.scan_count = 0
        
        self.get_logger().info('LIDAR node started')
        self.get_logger().info(f'Scan frequency: {self.scan_frequency} Hz')
        self.get_logger().info(f'Range: {self.range_min}m to {self.range_max}m')
        self.get_logger().info(f'Angle range: {self.angle_range}°')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'LIDAR scan #{self.scan_count}'
        self.publisher.publish(msg)
        self.scan_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Create Navigation Node

**File**: `robot_bringup/robot_bringup/navigation_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Declare parameters
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('update_rate', 20)
        self.declare_parameter('safety_distance', 0.5)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        self.get_logger().info('Navigation node started')
        self.get_logger().info(f'Speed range: {self.min_speed} - {self.max_speed} m/s')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Safety distance: {self.safety_distance} m')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Create Telemetry Node

**File**: `robot_bringup/robot_bringup/telemetry_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('report_rate', 1)
        self.declare_parameter('send_diagnostics', True)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.report_rate = self.get_parameter('report_rate').value
        self.send_diagnostics = self.get_parameter('send_diagnostics').value
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'telemetry/status', 10)
        
        # Create timer
        timer_period = 1.0 / self.report_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.report_count = 0
        
        self.get_logger().info(f'Telemetry node started for {self.robot_id}')
        self.get_logger().info(f'Report rate: {self.report_rate} Hz')
        self.get_logger().info(f'Diagnostics: {"enabled" if self.send_diagnostics else "disabled"}')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'{self.robot_id}: Status report #{self.report_count}'
        self.publisher.publish(msg)
        self.report_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Create Configuration Files

**File**: `robot_bringup/config/robot_params.yaml`

```yaml
/**:
  ros__parameters:
    # Front camera parameters
    front_camera:
      camera_name: "front_camera"
      frame_rate: 30
      resolution: "1920x1080"
      enable_processing: true
    
    # Back camera parameters
    back_camera:
      camera_name: "back_camera"
      frame_rate: 15
      resolution: "1280x720"
      enable_processing: false
    
    # LIDAR parameters
    lidar:
      scan_frequency: 10
      range_min: 0.1
      range_max: 30.0
      angle_range: 360
    
    # Navigation parameters
    navigation:
      max_speed: 2.0
      min_speed: 0.1
      update_rate: 20
      safety_distance: 0.5
    
    # Telemetry parameters
    telemetry:
      robot_id: "explorer_001"
      report_rate: 1
      send_diagnostics: true
```

**File**: `robot_bringup/config/sim_params.yaml`

```yaml
/**:
  ros__parameters:
    # Simulation-specific parameters (lower quality for performance)
    front_camera:
      camera_name: "front_camera"
      frame_rate: 15
      resolution: "640x480"
      enable_processing: false
    
    back_camera:
      camera_name: "back_camera"
      frame_rate: 10
      resolution: "640x480"
      enable_processing: false
    
    lidar:
      scan_frequency: 5
      range_min: 0.1
      range_max: 20.0
      angle_range: 270
    
    navigation:
      max_speed: 1.0
      min_speed: 0.1
      update_rate: 10
      safety_distance: 0.3
    
    telemetry:
      robot_id: "sim_robot_001"
      report_rate: 0.5
      send_diagnostics: false
```

### Step 7: Create Sensors Launch File

**File**: `robot_bringup/launch/sensors.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robot_bringup')
    
    # Declare launch argument for config file
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'robot_params.yaml'),
        description='Path to config file'
    )
    
    config_file = LaunchConfiguration('config_file')
    
    # Navigation node
    navigation = Node(
        package='robot_bringup',
        executable='navigation_node',
        name='navigation',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        config_arg,
        navigation,
    ])
```

### Step 9: Create Main Launch File

**File**: `robot_bringup/launch/robot.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    
    # Declare arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation parameters'
    )
    
    enable_telemetry_arg = DeclareLaunchArgument(
        'enable_telemetry',
        default_value='true',
        description='Enable telemetry node'
    )
    
    # Determine which config file to use
    use_sim = LaunchConfiguration('use_sim')
    enable_telemetry = LaunchConfiguration('enable_telemetry')
    
    # Choose config file based on use_sim
    config_file_real = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    config_file_sim = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    
    # Include sensors launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'config_file': config_file_real  # Can be made conditional
        }.items()
    )
    
    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'config_file': config_file_real
        }.items()
    )
    
    # Telemetry node (conditional)
    telemetry = Node(
        package='robot_bringup',
        executable='telemetry_node',
        name='telemetry',
        parameters=[config_file_real],
        condition=IfCondition(enable_telemetry)
    )
    
    return LaunchDescription([
        use_sim_arg,
        enable_telemetry_arg,
        sensors_launch,
        navigation_launch,
        telemetry,
    ])
```

### Step 10: Update setup.py

**File**: `robot_bringup/setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Robot bringup package with launch files and parameters',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = robot_bringup.camera_node:main',
            'lidar_node = robot_bringup.lidar_node:main',
            'navigation_node = robot_bringup.navigation_node:main',
            'telemetry_node = robot_bringup.telemetry_node:main',
        ],
    },
)
```

### Step 11: Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select robot_bringup
source install/setup.bash
```

### Step 12: Run Different Configurations

**Full robot system** (real hardware parameters):
```bash
ros2 launch robot_bringup robot.launch.py
```

**Output**:
```
[front_camera]: Camera node started: front_camera
[front_camera]: Resolution: 1920x1080
[front_camera]: Frame rate: 30 Hz
[back_camera]: Camera node started: back_camera
[back_camera]: Resolution: 1280x720
[back_camera]: Frame rate: 15 Hz
[lidar]: LIDAR node started
[lidar]: Scan frequency: 10 Hz
[navigation]: Navigation node started
[navigation]: Speed range: 0.1 - 2.0 m/s
[telemetry]: Telemetry node started for explorer_001
```

**Without telemetry**:
```bash
ros2 launch robot_bringup robot.launch.py enable_telemetry:=false
```

**Just sensors**:
```bash
ros2 launch robot_bringup sensors.launch.py
```

**With simulation parameters**:
```bash
ros2 launch robot_bringup sensors.launch.py \
    config_file:=install/robot_bringup/share/robot_bringup/config/sim_params.yaml
```

### Step 13: Change Parameters at Runtime

**Check current parameters**:
```bash
ros2 param list /sensors/front_camera
```

Output:
```
camera_name
enable_processing
frame_rate
resolution
use_sim_time
```

**Get specific parameter**:
```bash
ros2 param get /sensors/front_camera frame_rate
```

Output:
```
Integer value is: 30
```

**Set parameter**:
```bash
ros2 param set /sensors/front_camera frame_rate 60
```

Output:
```
Set parameter successful
```

**Dump all parameters to file**:
```bash
ros2 param dump /sensors/front_camera
```

Creates a YAML file with current parameters—great for saving configurations!

### What We Built

A complete, professional launch system with:

✅ **Multiple nodes** started with one command  
✅ **Organized structure** with separate launch files  
✅ **Configurable parameters** via YAML files  
✅ **Different configurations** for different scenarios  
✅ **Conditional launching** (enable/disable nodes)  
✅ **Namespaces** for organization  
✅ **Runtime parameter changes** without restart  

This is how professional robot systems are structured!

## Practical Notes

### Development Best Practices

**1. Start Simple, Build Complexity**
- Begin with one node and basic parameters
- Add more nodes incrementally
- Test at each step
- Build complex launch files from simple ones

**2. Organize Launch Files Logically**
```
launch/
├── robot.launch.py           # Main entry point
├── sensors.launch.py         # All sensors
├── navigation.launch.py      # Navigation stack
├── manipulation.launch.py    # Arm control
└── simulation.launch.py      # Simulator-specific
```

**3. Use Meaningful Parameter Names**
```yaml
# Good
max_speed: 2.0
camera_frame_rate: 30
enable_safety_monitoring: true

# Bad
ms: 2.0
cfr: 30
esm: true
```

**4. Document Your Parameters**

In YAML files, add comments:
```yaml
/**:
  ros__parameters:
    # Maximum speed in meters per second
    # Safe range: 0.5 - 3.0
    max_speed: 2.0
    
    # Camera frame rate in Hz
    # Higher rates increase CPU usage
    frame_rate: 30
```

**5. Group Related Parameters**
```yaml
camera_settings:
  resolution: "1920x1080"
  frame_rate: 30
  auto_exposure: true

navigation_settings:
  max_speed: 2.0
  min_speed: 0.1
  safety_distance: 0.5
```

### Common Issues and Solutions

**Issue 1: Launch file not found**
```bash
ros2 launch my_package my_launch.py
# Error: Package 'my_package' not found
```

**Solution**: Did you install launch files in setup.py and rebuild?
```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
```

**Issue 2: Config file not found**
```
FileNotFoundError: config/params.yaml
```

**Solution**: Use `get_package_share_directory()` for absolute paths:
```python
config = os.path.join(
    get_package_share_directory('my_package'),
    'config',
    'params.yaml'
)
```

**Issue 3: Parameters not loading**

**Solution**: Check YAML syntax:
```yaml
/**:                    # Must start with this
  ros__parameters:      # Exactly this, two underscores
    param_name: value   # Proper indentation
```

**Issue 4: Launch argument not working**
```bash
ros2 launch pkg robot.launch.py my_arg:=value
# Argument not recognized
```

**Solution**: Verify argument is declared:
```python
DeclareLaunchArgument('my_arg', default_value='default')
```

**Issue 5: Node gets wrong parameters**

**Solution**: Check parameter namespace in YAML matches node namespace:
```yaml
/my_namespace/my_node:  # Must match actual node path
  ros__parameters:
    my_param: value
```

### Debugging Launch Files

**See what's being launched**:
```bash
ros2 launch my_package robot.launch.py --show-args
```

Shows all available arguments.

**Check launch file syntax**:
```bash
python3 launch/robot.launch.py
```

Runs the Python file directly—catches syntax errors before launching.

**Verbose output**:
```bash
ros2 launch my_package robot.launch.py --debug
```

Shows detailed information about what's being launched.

**Monitor nodes**:
```bash
# After launching, check what's running
ros2 node list

# Check specific node's parameters
ros2 param list /node_name

# Watch topic activity
ros2 topic list
ros2 topic hz /topic_name
```

### Performance Tips

**Parameter Access**:
```python
# Good: Get once in __init__
self.max_speed = self.get_parameter('max_speed').value

# Bad: Get repeatedly in callback (slow!)
def callback(self):
    speed = self.get_parameter('max_speed').value  # Don't do this!
```

**Launch File Optimization**:
- Use `IncludeLaunchDescription` for modularity
- Don't duplicate parameter definitions
- Group related nodes in same launch file

**Config File Organization**:
```yaml
# One file per robot/configuration
robot1_params.yaml
robot2_params.yaml
sim_params.yaml
test_params.yaml
```

### Testing Launch Files

**Unit test individual nodes** before integrating into launch files.

**Test launch files incrementally**:
1. Launch one node → verify it works
2. Add second node → verify both work
3. Add parameters → verify they load
4. Add conditionals → verify logic

**Create test launch configurations**:
```
launch/
├── robot.launch.py          # Production
├── robot_test.launch.py     # Testing
└── robot_minimal.launch.py  # Minimal for debugging
```

### Advanced Launch Techniques

**Environment variables**:
```python
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable('MY_VAR', 'value')
```

**Event handlers**:
```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# Do something when a node starts
```

**Group actions**:
```python
from launch.actions import GroupAction

GroupAction(
    actions=[node1, node2, node3],
    condition=SomeCondition()
)
```

**Launch arguments from CLI**:
```bash
ros2 launch pkg robot.launch.py robot_name:=robot1 debug:=true
```

### Documentation Tips

**Document your launch files**:
```python
def generate_launch_description():
    """
    Launch the complete robot system.
    
    Arguments:
        use_sim (bool): Use simulation parameters (default: false)
        enable_telemetry (bool): Enable telemetry node (default: true)
        robot_id (string): Robot identifier (default: robot_001)
    
    Nodes launched:
        - Camera nodes (front and back)
        - LIDAR node
        - Navigation node
        - Telemetry node (optional)
    """
    # ... launch code ...
```

**Create README for launch files**:
```markdown
# Robot Launch Files

## robot.launch.py
Launches complete robot system.

Usage:
```bash
ros2 launch robot_bringup robot.launch.py [args]
```

Arguments:
- `use_sim`: Use simulation config (default: false)
- `enable_telemetry`: Enable telemetry (default: true)
```

## Summary

Let's recap the essential concepts of launch files and parameters:

**Launch files** start multiple nodes with one command, configure them with parameters, and organize complex robot systems. They're Python scripts that return a `LaunchDescription`.

**Basic launch file structure**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pkg', executable='node', name='name'),
    ])
```

**Parameters** are configuration values that can be changed without modifying code. They enable flexible, reusable nodes.

**Declaring parameters** in nodes:
```python
self.declare_parameter('param_name', default_value)
value = self.get_parameter('param_name').value
```

**Setting parameters** in launch files:
```python
Node(
    package='pkg',
    executable='node',
    parameters=[{'param1': value1, 'param2': value2}]
)
```

**Loading from YAML**:
```python
parameters=[config_file_path]
```

**YAML structure**:
```yaml
/**:
  ros__parameters:
    param1: value1
    param2: value2
```

**Launch arguments** make launch files flexible:
```python
DeclareLaunchArgument('arg_name', default_value='default')
LaunchConfiguration('arg_name')  # Use the argument
```

**Including other launch files** enables modular organization:
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource('path/to/launch.py')
)
```

**Conditional launching** starts nodes only when needed:
```python
Node(..., condition=IfCondition(LaunchConfiguration('flag')))
```

**Namespaces** organize multi-robot systems:
```python
Node(..., namespace='robot1')
```

**Topic remapping** changes topic names:
```python
Node(..., remappings=[('/old', '/new')])
```

**Runtime parameter changes**:
```bash
ros2 param set /node_name param_name value
ros2 param get /node_name param_name
ros2 param dump /node_name
```

**Best practices**:
- Organize launch files by functionality
- Use descriptive parameter names
- Document parameters with comments
- Test incrementally
- Use YAML for many parameters
- Keep launch files modular with includes

**Common issues**:
- Forgetting to install launch files in setup.py
- Incorrect YAML syntax (`ros__parameters` with two underscores)
- Wrong file paths (use `get_package_share_directory()`)
- Parameter namespace mismatches

**Why they matter**:
- Manage complex systems with dozens of nodes
- Configure without recompilation
- Support multiple deployment environments
- Enable team collaboration
- Document system architecture
- Save development time

Launch files and parameters are fundamental to professional ROS 2 development. They transform scattered nodes into organized, configurable, production-ready robot systems.

## Looking Ahead

Now that you master launch files and parameters, you're ready to build sophisticated multi-node systems efficiently.

In the next chapter, we'll explore Gazebo simulation—how to test your robots in virtual environments before deploying to real hardware. You'll learn to create simulated worlds, spawn robot models, and connect Gazebo to your ROS 2 nodes.

You'll also discover how to use RViz2 for visualization, helping you see sensor data, robot state, and debug your systems visually.

---

## Review Questions

Test your understanding:

1. **What problem do launch files solve?** Why can't we just start nodes manually?

2. **Write a launch file** that starts two nodes: a camera node and a detector node, both with a namespace of "robot1".

3. **What's the difference between a launch argument and a parameter?**

4. **In a YAML config file, what does `/**:` mean?** Why is it needed?

5. **How do you pass an argument to a launch file** from the command line?

6. **Write the code to declare a parameter** called `max_speed` with a default value of 2.0.

7. **What's the advantage of loading parameters from a YAML file** instead of setting them directly in the launch file?

8. **How would you create a launch file** that includes two other launch files?

## Hands-On Exercise

**Multi-Node Launch Challenge**:

Create a `weather_station` package with:

**Nodes**:
1. `temperature_sensor.py` - Publishes temperature (param: `update_rate`, `sensor_id`)
2. `humidity_sensor.py` - Publishes humidity (param: `update_rate`, `sensor_id`)
3. `data_logger.py` - Subscribes to both sensors (param: `log_file_path`, `enable_console_output`)

**Configuration**:
- Create `weather_params.yaml` with all parameters
- Create `sensors.launch.py` that starts both sensors
- Create `station.launch.py` that includes sensors launch and starts logger
- Add launch argument: `station_id` (default: "station_001")

**Test**:
```bash
ros2 launch weather_station station.launch.py station_id:=station_42
```

Verify all nodes start with correct parameters.

## Key Takeaways

Remember these essential principles:

✅ **Launch files are essential** - They make complex systems manageable  
✅ **Parameters enable flexibility** - Configure without recompiling  
✅ **YAML for many parameters** - Cleaner than inline definitions  
✅ **Modular launch files** - Use includes for organization  
✅ **Launch arguments for flexibility** - Pass values at runtime  
✅ **Namespaces prevent conflicts** - Essential for multi-robot systems  
✅ **Document everything** - Comments in YAML, docstrings in launch files  
✅ **Test incrementally** - Start simple, add complexity gradually  

Master launch files and parameters, and you'll build professional-quality robot systems that are configurable, maintainable, and ready for production deployment!

Next, we'll bring your robots to life in simulation with Gazebo!config_file',
        default_value=os.path.join(pkg_dir, 'config', 'robot_params.yaml'),
        description='Path to config file'
    )
    
    config_file = LaunchConfiguration('config_file')
    
    # Front camera node
    front_camera = Node(
        package='robot_bringup',
        executable='camera_node',
        name='front_camera',
        parameters=[config_file],
        namespace='sensors'
    )
    
    # Back camera node
    back_camera = Node(
        package='robot_bringup',
        executable='camera_node',
        name='back_camera',
        parameters=[config_file],
        namespace='sensors'
    )
    
    # LIDAR node
    lidar = Node(
        package='robot_bringup',
        executable='lidar_node',
        name='lidar',
        parameters=[config_file],
        namespace='sensors'
    )
    
    return LaunchDescription([
        config_arg,
        front_camera,
        back_camera,
        lidar,
    ])
```

### Step 8: Create Navigation Launch File

**File**: `robot_bringup/launch/navigation.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    
    config_arg = DeclareLaunchArgument(
        '