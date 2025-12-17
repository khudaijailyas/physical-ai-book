# Practical Examples & Labs

## Learning Objectives

By the end of this chapter, you will be able to:

- Set up and run three complete simulation projects from start to finish
- Apply the concepts from previous chapters to real, working examples
- Troubleshoot common problems that arise when building simulations
- Modify existing simulation projects to create your own variations
- Understand the complete workflow from concept to working simulation

## Concept Explanation

This chapter is different from the others. Instead of introducing new concepts, we're going to put everything you've learned into practice through three complete lab exercises. Each lab builds a real simulation project that you can run, modify, and learn from.

**What are Practical Labs?**: A practical lab is a hands-on exercise where you build something concrete. Rather than just reading about how simulations work, you'll actually create them. Think of it like the difference between reading a recipe and actually cooking the meal. Both are valuable, but cooking teaches you things reading never could.

**The Three Labs**: We've designed three labs that increase in complexity and demonstrate different aspects of robot simulation:

**Lab 1: Basic Mobile Robot in Gazebo** - You'll create a simple wheeled robot that can move around and avoid obstacles. This lab introduces the fundamental workflow of creating a robot model, setting up a world, and writing basic control code.

**Lab 2: Robotic Arm Manipulation in Webots** - You'll build a simulation with a robotic arm that picks up and moves objects. This lab focuses on manipulation, precise control, and working with more complex robot structures.

**Lab 3: Warehouse Robot with ROS 2** - You'll create a complete warehouse navigation system where a robot receives goals, plans paths, and navigates autonomously. This lab integrates multiple concepts and shows how professional robotics systems are built.

**The Learning Approach**: Each lab follows the same structure. First, we explain what you're building and why. Second, we provide step-by-step instructions to build the basic version. Third, we offer challenges and extensions you can tackle to deepen your understanding. You don't need to complete every extension—they're there for when you're ready to push further.

**Prerequisites**: Before starting these labs, you should have simulation software installed on your computer. You'll need Gazebo for Lab 1, Webots for Lab 2, and ROS 2 with Gazebo for Lab 3. Each lab begins with a quick checklist to ensure you have what you need.

**Iteration and Experimentation**: The labs are designed for experimentation. If something doesn't work the first time, that's not failure—that's learning. Professional roboticists spend much of their time debugging and refining. When you encounter errors or unexpected behavior, you're getting authentic experience in robotics development. We've included a troubleshooting section for each lab with solutions to common problems.

**Building Your Portfolio**: As you complete these labs, you're building a portfolio of working projects. You can modify these projects, extend them in creative directions, and use them as starting points for your own ideas. Many students have taken these basic labs and turned them into impressive projects for classes, competitions, or job applications.

## Why This Matters

Practical labs matter because they transform theoretical knowledge into actual skills. Here's why hands-on practice is essential for learning robot simulation.

**Theory vs. Practice Gap**: You can understand every concept in this textbook intellectually, but until you actually build something, you won't develop the intuition that makes you effective. When you read about physics engines, you understand the concept. When your simulated robot unexpectedly tips over because you didn't account for its center of mass, you truly understand physics engines. This kind of learning sticks with you.

**Problem-Solving Skills**: Labs force you to encounter and solve real problems. Your code has a bug. Your robot model won't load. Your sensors aren't detecting what you expected. These problems are frustrating in the moment but invaluable for learning. Professional robotics work is largely problem-solving, and labs give you practice in a low-stakes environment.

**Confidence Building**: There's a psychological benefit to completing a working project. When you see your simulated robot successfully navigating a maze or a robotic arm picking up objects, it builds confidence. This confidence encourages you to tackle more challenging projects. Many people who felt intimidated by robotics discovered through labs that they could actually do it.

**Understanding Trade-offs**: Labs reveal the trade-offs involved in robotics design. When you try to make your robot move faster, you might discover it becomes less stable. When you add more sensors, your code becomes more complex. When you increase simulation fidelity, your computer slows down. These trade-offs aren't obvious from reading—you need to experience them.

**Integration of Concepts**: Earlier chapters taught individual concepts: what a digital twin is, how Gazebo works, how to use ROS 2. Labs show you how these concepts work together. You'll see how the simulator provides sensor data that your ROS 2 code processes to make control decisions that move the robot. This integrated understanding is crucial for working on real robotics projects.

**Preparation for Real Robots**: These simulation labs prepare you for working with physical robots. The workflow is similar: you set up the hardware (or load a model), write control code, test and debug, and iteratively improve. Many people find that after mastering simulation, working with real robots feels less daunting because the development process is familiar.

**Job Readiness**: In robotics job interviews, being able to discuss specific projects you've built demonstrates practical experience. Employers value candidates who can show working code and explain the decisions they made. These labs give you concrete projects to discuss and build upon.

**Foundation for Innovation**: Once you can confidently build these standard lab projects, you can start innovating. You might think "What if I combined the navigation from Lab 3 with the manipulation from Lab 2?" or "Could I modify Lab 1 to work with multiple robots?" Innovation builds on solid fundamentals, and labs provide those fundamentals.

## Example

Let's walk through complete examples of each of the three labs, showing exactly what you'll build and how you'll build it.

### Lab 1: Basic Mobile Robot in Gazebo

**What You're Building**: A two-wheeled differential drive robot that wanders around a simple environment, avoiding obstacles using distance sensors. When the robot detects a wall or obstacle ahead, it turns away and continues exploring.

**Step 1: Installing and Verifying Gazebo**

First, ensure Gazebo is installed. Open a terminal and type:

```bash
gazebo --version
```

You should see a version number. If not, you need to install Gazebo following the instructions for your operating system.

**Step 2: Creating Your Workspace**

Create a folder for your project:

```bash
mkdir -p ~/robot_labs/lab1
cd ~/robot_labs/lab1
```

This gives you an organized place to keep all your lab files.

**Step 3: Creating the Robot Model**

Create a file called `simple_robot.sdf` with your robot description. This file defines what your robot looks like and how it behaves. The model includes:

- A main body (a box shape)
- Two wheels (cylinder shapes)
- Sensors (simulated distance sensors)
- Motors (to make the wheels turn)

You can start with a basic model that just describes the physical structure. The model uses SDF (Simulation Description Format), which is XML-based. Here's a simplified version:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="simple_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.4 0.2 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.4 0.2 0.1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

This creates just the body. You'll add wheels and sensors following similar patterns.

**Step 4: Creating a Simple World**

Create a file called `simple_world.world` that defines your environment. This world has:

- A flat ground plane
- Four walls forming a room
- Some box obstacles scattered around
- Lighting so you can see everything

Gazebo worlds also use SDF format. You can include existing models like ground planes from Gazebo's model database.

**Step 5: Testing the Basics**

Launch Gazebo with your world:

```bash
gazebo simple_world.world
```

Gazebo opens showing your environment. Now manually add your robot using the insert tab. You should see your robot sitting in the world. It won't move yet—that comes next.

**Step 6: Adding Control with a Plugin**

To make your robot move, you'll use a Gazebo plugin. Plugins are pieces of code that add behavior to your simulation. You modify your robot model to include a differential drive plugin that lets you control the robot's wheels.

The plugin configuration specifies which wheels to control, how fast they can turn, and what topics to use for commands. You add this to your SDF file within the model tags.

**Step 7: Writing the Obstacle Avoidance Logic**

Now you write a simple Python script that implements obstacle avoidance:

```python
import random

def obstacle_avoidance():
    while True:
        # Read sensor data
        distances = read_distance_sensors()
        
        # If something is close ahead
        if min(distances) < 0.5:  # Less than 0.5 meters
            # Turn away
            turn_direction = random.choice(['left', 'right'])
            send_turn_command(turn_direction)
        else:
            # Move forward
            send_forward_command()
```

This pseudocode shows the logic. The actual implementation uses Gazebo's API to read sensors and send commands.

**Step 8: Running the Complete System**

With everything in place, you launch Gazebo with your world and robot, then run your control script. Your robot starts moving! It wanders around the environment, and when it approaches a wall or obstacle, it turns away and continues. You've built your first autonomous robot simulation!

**Extensions to Try**:
- Modify the robot to always turn left instead of randomly choosing
- Add more sensors to detect obstacles from the sides
- Create a more complex environment with narrow corridors
- Make the robot move faster and observe what happens
- Add a second robot to the environment

### Lab 2: Robotic Arm Manipulation in Webots

**What You're Building**: A robotic arm mounted on a table that can pick up colored blocks and stack them. The arm uses a camera to detect block positions and a gripper to grasp them.

**Step 1: Setting Up Webots**

Launch Webots and verify it opens correctly. Create a new world file. Webots provides a wizard that helps you set up the basic scene.

**Step 2: Adding the Robotic Arm**

Webots includes several pre-built robot models. For this lab, you'll use a simulated UR5 robotic arm (a common industrial arm) or a similar model. You place the arm in your world using Webots' robot library:

1. Click "Add a new object"
2. Navigate to the robots section
3. Select a manipulator arm
4. Place it at position (0, 0, 0)

The arm appears with all its joints, links, and end-effector already configured.

**Step 3: Creating the Environment**

Add a table beneath the arm and several colored blocks on the table surface. Webots makes this easy:

- Add a rectangular solid for the table
- Set its appearance to wood texture
- Add several small box objects
- Give each box a different color (red, blue, green)
- Position them randomly on the table

**Step 4: Adding Sensors**

Your arm needs to see the blocks. Add a camera to the arm:

1. Select the arm's end-effector node
2. Add a camera device as a child
3. Configure the camera's position and angle to look down at the table
4. Set the image resolution (640x480 works well)

**Step 5: Writing the Controller**

Create a Python controller that will control the arm. The controller goes through these steps:

```python
# Pseudocode for arm controller
def main():
    # Initialize robot and devices
    robot = Robot()
    camera = robot.getDevice('camera')
    gripper = robot.getDevice('gripper')
    
    while robot.step(timestep) != -1:
        # Capture image from camera
        image = camera.getImage()
        
        # Find colored blocks
        blocks = detect_blocks(image)
        
        # If blocks found
        if blocks:
            # Pick the closest block
            target = blocks[0]
            
            # Move arm to block
            move_to_position(target.x, target.y, target.z)
            
            # Close gripper
            gripper.close()
            
            # Lift block
            move_to_position(target.x, target.y, target.z + 0.2)
            
            # Move to stacking location
            move_to_position(stack_x, stack_y, stack_z)
            
            # Release block
            gripper.open()
```

The actual code is more detailed, handling inverse kinematics (calculating joint angles to reach a position) and error checking.

**Step 6: Implementing Vision Processing**

The `detect_blocks` function processes the camera image to find colored blocks:

```python
def detect_blocks(image):
    blocks = []
    
    # Convert image to format for processing
    processed = process_image(image)
    
    # Find regions of each color
    for color in ['red', 'blue', 'green']:
        regions = find_color_regions(processed, color)
        
        # Convert image coordinates to 3D positions
        for region in regions:
            position_3d = image_to_world(region.center)
            blocks.append(Block(color, position_3d))
    
    return blocks
```

This involves basic computer vision. Webots provides helper functions for image processing.

**Step 7: Testing and Debugging**

Run your simulation. Watch the arm's behavior:

- Does it correctly identify blocks?
- Does it move to the right positions?
- Does the gripper successfully grasp blocks?

Use Webots' visualization tools to display what the camera sees. This helps debug vision problems.

**Step 8: Improving the System**

Once basic functionality works, improve it:

- Handle cases where the arm drops a block
- Stack blocks in order by color
- Optimize the arm's path for faster movement
- Add collision detection to prevent the arm from hitting the table

**Extensions to Try**:
- Stack blocks in a specific pattern (pyramid, tower)
- Add more blocks and see how the system scales
- Implement a different grasping strategy
- Add a second arm and coordinate their movements
- Create blocks of different shapes (cylinders, spheres)

### Lab 3: Warehouse Robot with ROS 2

**What You're Building**: An autonomous mobile robot that receives navigation goals, plans collision-free paths through a warehouse environment, and executes those paths while avoiding dynamic obstacles.

**Step 1: Setting Up ROS 2 and Gazebo**

Verify your ROS 2 installation:

```bash
ros2 --version
```

Create a ROS 2 workspace:

```bash
mkdir -p ~/robot_labs/lab3_ws/src
cd ~/robot_labs/lab3_ws/src
```

**Step 2: Creating the Robot Description Package**

Create a ROS 2 package for your robot:

```bash
ros2 pkg create warehouse_robot --build-type ament_python
```

Inside this package, create a URDF (Unified Robot Description Format) file describing your robot. The robot has:

- A rectangular base
- Two drive wheels and one caster wheel
- A LIDAR sensor for detecting obstacles
- A camera for visual navigation

**Step 3: Building the Warehouse World**

Create a Gazebo world file representing a warehouse:

```xml
<!-- Simplified warehouse structure -->
<world name="warehouse">
  <include>
    <uri>model://ground_plane</uri>
  </include>
  
  <!-- Warehouse walls -->
  <!-- Shelving units -->
  <!-- Loading dock area -->
  <!-- Obstacles -->
</world>
```

Use building blocks from Gazebo's model database to construct shelves, walls, and storage areas.

**Step 4: Creating the ROS 2 Launch File**

Write a launch file that starts everything:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with the warehouse world
        # Spawn the robot
        # Start the ROS 2 - Gazebo bridge
        # Start navigation nodes
        # Start visualization tools
    ])
```

**Step 5: Implementing Navigation**

Create a navigation node that:

```python
class WarehouseNavigator:
    def __init__(self):
        # Subscribe to LIDAR data
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback)
        
        # Publish velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist, '/cmd_vel')
        
        # Store current goal
        self.goal = None
    
    def set_goal(self, x, y):
        """Set a navigation goal"""
        self.goal = (x, y)
        self.plan_path()
    
    def plan_path(self):
        """Plan a path from current position to goal"""
        # Use A* or other path planning algorithm
        # Create waypoints
        pass
    
    def scan_callback(self, msg):
        """Process LIDAR data for obstacle avoidance"""
        # Check for obstacles
        # Adjust path if needed
        pass
    
    def execute_path(self):
        """Follow the planned path"""
        # Send velocity commands
        # Check progress
        # Handle obstacles
        pass
```

**Step 6: Adding Obstacle Avoidance**

Implement dynamic obstacle avoidance that works while following the path:

```python
def check_for_obstacles(scan_data):
    """Analyze LIDAR data for obstacles in path"""
    min_distance = min(scan_data.ranges)
    
    if min_distance < SAFETY_DISTANCE:
        return True, calculate_avoidance_direction(scan_data)
    
    return False, None

def calculate_avoidance_direction(scan_data):
    """Find the best direction to avoid obstacle"""
    # Analyze which direction has most free space
    # Return turn direction
    pass
```

**Step 7: Creating a Goal Interface**

Add a service that lets you send goals to the robot:

```python
class GoalService(Node):
    def __init__(self):
        self.service = self.create_service(
            SetGoal, 'set_goal', self.handle_goal)
    
    def handle_goal(self, request, response):
        """Handle incoming goal requests"""
        x, y = request.goal_x, request.goal_y
        self.navigator.set_goal(x, y)
        response.success = True
        return response
```

**Step 8: Running and Testing**

Launch the complete system:

```bash
ros2 launch warehouse_robot warehouse_nav.launch.py
```

Multiple windows open:
- Gazebo showing the warehouse
- RViz showing sensor data and planned paths
- Terminal windows for each node

Send a goal to the robot:

```bash
ros2 service call /set_goal warehouse_robot/SetGoal "{goal_x: 5.0, goal_y: 3.0}"
```

Watch as the robot plans a path, begins navigating, avoids obstacles, and reaches its goal.

**Step 9: Adding Multiple Goals**

Extend the system to handle a sequence of goals:

```python
class MissionPlanner:
    def __init__(self):
        self.goals = []
        self.current_goal_index = 0
    
    def add_mission(self, goal_list):
        """Add multiple goals to complete in order"""
        self.goals = goal_list
    
    def execute_mission(self):
        """Execute all goals in sequence"""
        for goal in self.goals:
            self.send_goal(goal)
            self.wait_for_completion()
```

**Step 10: Performance Analysis**

Add data collection to analyze performance:

```python
class PerformanceMonitor:
    def __init__(self):
        self.start_time = None
        self.distance_traveled = 0
        self.goals_reached = 0
    
    def log_metrics(self):
        """Record and display performance metrics"""
        print(f"Goals reached: {self.goals_reached}")
        print(f"Total distance: {self.distance_traveled}m")
        print(f"Average speed: {self.calculate_avg_speed()}")
```

**Extensions to Try**:
- Add multiple robots and coordinate their movements
- Implement elevator use for multi-floor navigation
- Add package pickup and delivery logic
- Create a scheduling system for multiple delivery requests
- Implement battery monitoring and charging behavior
- Add a user interface for sending goals visually

## Practical Notes

These practical tips will help you successfully complete the labs and learn from the experience.

**Time Expectations**: Lab 1 typically takes 3-5 hours for a beginner to complete the basic version. Lab 2 takes 4-6 hours. Lab 3 takes 6-8 hours. These estimates include time for troubleshooting and experimentation. Don't try to rush—the learning happens during the process, not just in the final result. If you have limited time, complete the labs over several sessions rather than trying to finish in one sitting.

**Save Your Work Frequently**: Simulation projects involve multiple files that all need to work together. Use version control (Git) if you know how. If not, at minimum keep dated backup copies of your working code. Name them clearly like `robot_controller_working_v1.py` so you can return to a working version if you break something while experimenting.

**Start Simple, Add Complexity**: Each lab provides a basic version and extensions. Complete the basic version first. Get it fully working. Only then add extensions. Many beginners try to implement everything at once and end up overwhelmed. Incremental development is how professionals work—it should be how you work too.

**Read Error Messages Carefully**: When something goes wrong, read the error message completely. Error messages often tell you exactly what's wrong and which file and line number has the problem. Don't just see "error" and give up—read the details. Many errors have simple fixes once you understand what the message is telling you.

**Use Print Statements for Debugging**: When your code isn't behaving as expected, add print statements to display variable values and program flow. For example:

```python
print(f"Current position: {x}, {y}")
print(f"Sensor reading: {distance}")
print("Entering obstacle avoidance mode")
```

These simple prints help you understand what's happening inside your program. Remove them once things work, but don't be afraid to use them liberally during development.

**Visualize Everything**: Use the visualization tools available in each simulator. In Gazebo, you can visualize sensor rays and coordinate frames. In Webots, you can see what cameras detect. In ROS 2, use RViz to visualize everything. Visualization reveals problems that are invisible when just watching the robot move.

**Check Units and Coordinate Systems**: A common source of bugs is unit mismatches. Is that distance in meters or centimeters? Is that angle in degrees or radians? Double-check units. Similarly, understand the coordinate system—which axis is up, which is forward? Many frustrating bugs come from assuming X is forward when it's actually Y.

**Hardware Limitations Are Real**: If your simulation runs slowly, you might need to simplify. Reduce the number of objects, lower the graphics quality, or decrease the physics update rate. It's better to have a simpler simulation that runs well than a complex one that's unusably slow. You can always add detail later when working on a faster computer.

**Learn from Failures**: When something doesn't work, that's a learning opportunity. Your robot crashes into a wall? Understand why—was the sensor data wrong, the logic flawed, the control commands incorrect? Investigating failures teaches you more than successes.

**Customize and Experiment**: Once you complete a lab's basic version, make it your own. Change the environment, modify the robot, implement a different algorithm. The labs are starting points, not prescriptions. Some of the best learning comes from trying your own ideas, even if they don't work perfectly.

**Document What You Learn**: Keep notes about problems you encountered and how you solved them. Write comments in your code explaining tricky parts. This documentation helps you remember what you learned and helps others if you share your code. Your future self will thank you when you return to a project months later.

**Ask for Help Appropriately**: When stuck, first try to solve the problem yourself for at least 20-30 minutes. Check documentation, reread the lab instructions, review error messages. If still stuck, then ask for help, but provide specific information: what you're trying to do, what you expected to happen, what actually happened, error messages, and what you've already tried. Specific questions get better answers than "it doesn't work."

**Compare with Reference Solutions**: After completing a lab, compare your solution with others if available. There are many ways to solve each problem. Seeing alternative approaches expands your understanding. Don't worry if your solution looks different—if it works, it's valid.

**Build on These Labs**: Use these labs as foundations for more ambitious projects. Lab 1's obstacle avoidance could become a maze-solving robot. Lab 2's manipulation could become an assembly system. Lab 3's navigation could become a delivery robot system. Think of these labs as building blocks for larger creations.

## Summary

This chapter provided three complete hands-on labs that apply the concepts from earlier chapters to real simulation projects. Lab 1 built a basic mobile robot with obstacle avoidance in Gazebo. Lab 2 created a robotic arm manipulation system in Webots. Lab 3 developed an autonomous warehouse navigation robot using ROS 2 and Gazebo.

Practical labs matter because they transform theoretical knowledge into concrete skills. They force you to encounter and solve real problems, build confidence through completed projects, reveal design trade-offs, integrate multiple concepts, and prepare you for working with physical robots. The experience of building working systems is irreplaceable.

Each lab followed a structured approach: verify prerequisites, build components incrementally, test at each stage, debug problems, and extend the basic system with additional features. The labs increased in complexity, with Lab 1 introducing fundamental concepts, Lab 2 adding manipulation and vision processing, and Lab 3 integrating multiple systems for autonomous navigation.

Working through these labs effectively requires saving work frequently, starting simple before adding complexity, reading error messages carefully, using visualization tools, checking units and coordinate systems, and learning from both successes and failures. The labs are starting points for your own creative extensions and more ambitious projects.

By completing these labs, you've moved from understanding robot simulation concepts to actually building working simulated robot systems. You have tangible projects that demonstrate your skills, experience with the development workflow, and confidence to tackle more complex robotics challenges. These practical skills form the foundation for continued learning and innovation in robotics.