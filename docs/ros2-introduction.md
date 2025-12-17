# Chapter 2: Introduction to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what ROS 2 is and why it exists
- Explain the basic architecture of a ROS 2 system
- Identify the key components: nodes, topics, services, and actions
- Recognize when and why to use ROS 2 for robotics projects
- Understand the differences between ROS 1 and ROS 2

## Concept Explanation

### What is ROS 2?

**ROS 2** stands for **Robot Operating System 2**. Despite its name, ROS 2 is not actually an operating system like Windows or Linux. Instead, it's a framework—a collection of tools, libraries, and conventions that help you build robot software.

Think of ROS 2 as a **language translator and messenger system** for robots. Just like how people in a company need to communicate and share information to work together, different parts of a robot need to talk to each other. ROS 2 makes this communication easy and organized.

### A Simple Analogy: The Restaurant Kitchen

Imagine a busy restaurant kitchen:

- The **head chef** needs to coordinate everything
- **Line cooks** prepare different dishes
- **Servers** take orders and deliver food
- The **dishwasher** cleans plates
- The **expeditor** makes sure dishes go to the right tables

Everyone has a specific job, but they must communicate constantly:
- "Order up! Table 5 needs pasta!"
- "We're out of tomatoes!"
- "Is the soup ready?"

ROS 2 works the same way for robots. Different parts of the robot (like the camera, motors, navigation system) are like workers in the kitchen. ROS 2 is the communication system that lets them work together smoothly.

### The Core Components of ROS 2

ROS 2 has four main building blocks that you'll use constantly:

#### 1. Nodes

**What they are**: Nodes are individual programs that each do one specific job.

**The analogy**: Each node is like one worker in the restaurant. One node might control the robot's camera, another handles navigation, and another manages the motors.

**Why separate nodes**: Instead of one giant program doing everything, we break tasks into smaller pieces. This makes the code easier to write, test, and fix. If the camera node crashes, the rest of the robot can keep working.

**Example**: A simple robot might have:
- A `camera_node` that captures images
- A `detector_node` that finds objects in images
- A `movement_node` that controls the wheels
- A `brain_node` that makes decisions

#### 2. Topics

**What they are**: Topics are communication channels where nodes publish and subscribe to messages.

**The analogy**: Topics are like radio stations. One node "broadcasts" information on a topic, and other nodes "tune in" to listen. Multiple nodes can listen to the same broadcast, and a node can broadcast on multiple channels.

**How they work**:
- **Publisher**: A node that sends messages (like a radio station)
- **Subscriber**: A node that receives messages (like a radio receiver)
- **Message**: The actual data being sent (like a song on the radio)

**Example**: 
```
camera_node → publishes images → /camera/image topic → detector_node subscribes
```

The camera publishes images to the `/camera/image` topic, and the detector subscribes to that topic to receive the images.

**Key feature**: Topics use a "publish-subscribe" pattern. The publisher doesn't know or care who's listening. The subscriber doesn't know or care where the data comes from. They just use the topic name to connect.

#### 3. Services

**What they are**: Services allow one node to request something from another node and wait for a response.

**The analogy**: Services are like asking a coworker a question and waiting for an answer. "Hey, what's the status of order #42?" and they respond "It's ready!"

**How they work**:
- **Service Server**: A node that provides a service (answers requests)
- **Service Client**: A node that requests the service (asks questions)
- **Request-Response**: Unlike topics (continuous streaming), services are one request, one response

**Example**:
```
Navigation node → requests → "Is path clear?" → Obstacle detector → responds → "Yes, clear!"
```

**When to use services**: Use services when you need:
- A question answered (not continuous data)
- Confirmation that something happened
- A task that completes and returns a result

#### 4. Actions

**What they are**: Actions are like services, but for tasks that take a long time and need progress updates.

**The analogy**: Actions are like ordering a pizza. You place the order (request), the restaurant confirms it (accepts), they give you status updates ("pizza in oven," "out for delivery"), and finally you get the pizza (result). You can also cancel the order midway if you change your mind.

**How they work**:
- **Action Server**: Does the long-running task
- **Action Client**: Requests the task and monitors progress
- **Goal**: What you want accomplished
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when complete

**Example**:
```
Action: "Navigate to kitchen"
- Goal: Kitchen coordinates
- Feedback: "20% complete... 50% complete... 80% complete"
- Result: "Arrived successfully" or "Failed: obstacle blocked path"
```

**When to use actions**: Use actions when you need:
- A task that takes several seconds or longer
- Progress updates during the task
- Ability to cancel the task midway

### How These Components Work Together

Let's see how nodes, topics, services, and actions work together in a real scenario.

**Scenario**: A delivery robot bringing medicine to a hospital room.

**The system**:

1. **Camera Node** (publishes to topic)
   - Continuously publishes images to `/camera/image` topic
   - Other nodes subscribe to see what the camera sees

2. **Object Detection Node** (subscribes to topic, publishes to another topic)
   - Subscribes to `/camera/image` topic
   - Processes images to detect obstacles
   - Publishes results to `/detected_objects` topic

3. **Navigation Node** (uses action)
   - Receives goal: "Go to Room 305"
   - Subscribes to `/detected_objects` to avoid obstacles
   - Provides feedback: "25% there... 50% there..."
   - Returns result: "Arrived at Room 305"

4. **Battery Monitor Node** (provides service)
   - Other nodes can request: "What's the battery level?"
   - Responds immediately: "75% charged"

5. **Main Control Node** (coordinates everything)
   - Sends action goal to Navigation Node
   - Monitors feedback from navigation
   - Checks battery via service call if needed
   - Makes high-level decisions

**Communication flow**:
```
Camera → (topic) → Object Detector → (topic) → Navigation → (action feedback) → Main Control
                                                      ↑
                                                      |
                                          Battery ← (service) ← Main Control
```

### ROS 2 Graph: The Big Picture

All these nodes and connections form what's called the **ROS 2 Graph**. You can visualize it like a network diagram showing:
- Circles for nodes
- Arrows for topics
- Lines for services and actions

This graph shows you exactly how information flows through your robot system.

### Message Types

When nodes communicate through topics, services, or actions, they use **message types**—defined data structures.

**Common message types**:

- `std_msgs/String`: Simple text message
- `sensor_msgs/Image`: Camera image data
- `geometry_msgs/Twist`: Movement commands (forward speed, turning speed)
- `nav_msgs/Odometry`: Robot's position and velocity
- `sensor_msgs/LaserScan`: LIDAR distance measurements

**Example message**:
```
Message Type: geometry_msgs/Twist
Contents:
  linear:
    x: 0.5  # Move forward at 0.5 m/s
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.1  # Turn at 0.1 rad/s
```

You can also create **custom message types** for your specific robot's needs.

### ROS 2 Workspaces and Packages

To stay organized, ROS 2 uses a specific structure:

**Workspace**: A folder containing all your ROS 2 projects.

**Package**: A collection of related code, like a mini-project. Each package might contain:
- Python or C++ code for nodes
- Configuration files
- Launch files (to start multiple nodes at once)
- Message definitions

**Structure**:
```
my_robot_workspace/
├── src/
│   ├── camera_package/
│   │   ├── camera_node.py
│   │   └── package.xml
│   ├── navigation_package/
│   │   ├── navigation_node.py
│   │   └── package.xml
│   └── sensors_package/
│       ├── lidar_node.py
│       └── package.xml
├── install/
├── build/
└── log/
```

### Launch Files

Instead of manually starting each node one by one, **launch files** let you start your entire robot system with one command.

**Example**: A launch file might start:
- Camera node
- Object detection node
- Navigation node
- Visualization tools

All at once, with proper configuration.

### ROS 2 Tools

ROS 2 comes with powerful command-line tools:

**`ros2 node list`**: Show all running nodes

**`ros2 topic list`**: Show all active topics

**`ros2 topic echo /topic_name`**: See messages being published

**`ros2 service list`**: Show all available services

**`ros2 run package_name node_name`**: Start a specific node

**`ros2 launch package_name launch_file.py`**: Start multiple nodes

These tools help you understand what's happening in your robot system and debug problems.

### Why ROS 2 Instead of ROS 1?

ROS 1 was incredibly successful, but it had limitations. ROS 2 was built from the ground up to fix these issues:

**Key improvements in ROS 2**:

1. **Better for real robots**: ROS 1 wasn't designed for safety-critical systems. ROS 2 has real-time support and better reliability.

2. **Security**: ROS 2 includes built-in security features (encryption, authentication) that ROS 1 lacked.

3. **Multi-robot systems**: ROS 2 makes it easier to run multiple robots that communicate with each other.

4. **Cross-platform**: ROS 2 works on Linux, Windows, and macOS. ROS 1 was mainly Linux-only.

5. **Modern communication**: ROS 2 uses DDS (Data Distribution Service), an industry-standard protocol that's more robust than ROS 1's custom system.

6. **Quality of Service (QoS)**: ROS 2 lets you tune reliability vs. speed based on your needs.

**Should you learn ROS 1 or ROS 2?**: Learn ROS 2. While some older robots still use ROS 1, ROS 2 is the future and receives all new development.

## Why This Matters

### Real-World Impact

ROS 2 is used in:

**Industrial robots**: Factory automation, warehouse robots (like those in Amazon warehouses)

**Self-driving cars**: Many autonomous vehicle companies build on ROS 2

**Drones**: For navigation, obstacle avoidance, and mission planning

**Service robots**: Delivery robots, cleaning robots, restaurant robots

**Research**: Universities and research labs worldwide use ROS 2

**Space exploration**: NASA uses ROS for planetary rovers

### Why Learn ROS 2?

**Industry standard**: If you want a job in robotics, most companies expect ROS knowledge.

**Avoid reinventing the wheel**: ROS 2 provides solutions to common robotics problems. Don't spend months building communication systems when ROS 2 already solved it.

**Large ecosystem**: Thousands of pre-built packages for cameras, sensors, navigation, manipulation, and more. You can build on others' work.

**Community support**: Active community means when you have questions, help is available.

**Hardware agnostic**: Your ROS 2 code can work with different robot brands and sensors without major rewrites.

### The Power of Modularity

The biggest benefit of ROS 2 is **modularity**—breaking complex systems into simple, reusable pieces.

**Benefits**:

- **Easier development**: Work on one small piece at a time
- **Better testing**: Test each node independently
- **Team collaboration**: Different people can work on different nodes
- **Reusability**: Use the same camera node on different robot projects
- **Debugging**: When something breaks, you know which node to fix

### Simulation First

ROS 2 integrates beautifully with simulators like Gazebo. This means:

**Safe testing**: Test dangerous behaviors in simulation first

**Faster iteration**: No need to charge batteries or repair hardware

**Impossible scenarios**: Test your robot in environments you don't have access to physically

**Cost savings**: One simulator license is cheaper than multiple physical robots

## Example

Let's walk through a complete, concrete example: **A simple robot that follows colored objects**.

### The Goal

Build a robot that:
1. Uses a camera to see colored objects
2. Identifies red objects
3. Moves toward the red object

### The Node Architecture

We'll create three nodes:

**Node 1: Camera Node**
- **Job**: Capture images from the camera
- **Publishes to**: `/camera/image` topic
- **Message type**: `sensor_msgs/Image`

**Node 2: Color Detector Node**
- **Job**: Find red objects in images
- **Subscribes to**: `/camera/image` topic
- **Publishes to**: `/red_object_position` topic
- **Message type**: `geometry_msgs/Point` (x, y coordinates)

**Node 3: Movement Controller Node**
- **Job**: Move the robot toward the red object
- **Subscribes to**: `/red_object_position` topic
- **Publishes to**: `/cmd_vel` topic (velocity commands)
- **Message type**: `geometry_msgs/Twist`

### The Data Flow

```
Camera Node → publishes image → /camera/image topic
                                      ↓
                          Color Detector Node subscribes
                                      ↓
                          processes image, finds red
                                      ↓
        publishes position → /red_object_position topic
                                      ↓
                      Movement Controller Node subscribes
                                      ↓
                    calculates movement commands
                                      ↓
            publishes velocity → /cmd_vel topic → Robot moves!
```

### Simplified Code Structure

**Camera Node (pseudocode)**:
```python
class CameraNode:
    def __init__(self):
        # Create publisher for images
        self.image_publisher = create_publisher('/camera/image')
        
    def capture_and_publish(self):
        # Get image from camera
        image = camera.capture()
        
        # Publish it
        self.image_publisher.publish(image)
```

**Color Detector Node (pseudocode)**:
```python
class ColorDetectorNode:
    def __init__(self):
        # Subscribe to camera images
        create_subscriber('/camera/image', self.image_callback)
        
        # Create publisher for detected position
        self.position_publisher = create_publisher('/red_object_position')
        
    def image_callback(self, image):
        # Find red pixels in image
        red_pixels = find_red_color(image)
        
        # Calculate center position
        position = calculate_center(red_pixels)
        
        # Publish position
        self.position_publisher.publish(position)
```

**Movement Controller Node (pseudocode)**:
```python
class MovementControllerNode:
    def __init__(self):
        # Subscribe to object position
        create_subscriber('/red_object_position', self.position_callback)
        
        # Create publisher for movement commands
        self.cmd_publisher = create_publisher('/cmd_vel')
        
    def position_callback(self, position):
        # Calculate how to move toward object
        if position.x < center:
            turn_left()
        elif position.x > center:
            turn_right()
        else:
            move_forward()
```

### Running the System

**Step 1**: Start each node (or use a launch file):
```bash
ros2 run camera_package camera_node
ros2 run detector_package color_detector_node
ros2 run controller_package movement_controller_node
```

**Step 2**: The nodes automatically connect through topics

**Step 3**: Watch the robot follow red objects!

### Debugging the System

**Check what's running**:
```bash
ros2 node list
# Output:
# /camera_node
# /color_detector_node
# /movement_controller_node
```

**Check communication**:
```bash
ros2 topic list
# Output:
# /camera/image
# /red_object_position
# /cmd_vel
```

**Watch the data**:
```bash
ros2 topic echo /red_object_position
# Output shows real-time position updates:
# x: 320
# y: 240
# ---
# x: 325
# y: 238
```

### Extending the System

The beauty of ROS 2's modular design: you can easily improve the system:

**Add obstacle avoidance**: Create a new node that subscribes to LIDAR data and publishes to `/cmd_vel` with higher priority.

**Add voice control**: Create a speech recognition node that publishes color choices to a `/target_color` topic that the detector subscribes to.

**Add logging**: Create a node that subscribes to all topics and saves data for later analysis.

**Add multiple robots**: Run the same nodes on different robots that coordinate through shared topics.

## Practical Notes

### Getting Started with ROS 2

**Installation**: ROS 2 is available for Ubuntu Linux (recommended), Windows, and macOS. Installation takes 30-60 minutes following the official guides.

**Recommended version**: Use the latest LTS (Long Term Support) release. As of late 2024, that's ROS 2 Jazzy Jalisco or Humble Hawksbill.

**Learning path**: 
1. Start with tutorials on the official ROS 2 website
2. Build simple publisher-subscriber examples
3. Progress to services and actions
4. Learn launch files and parameters
5. Practice with simulation before real hardware

### Common Beginner Mistakes

**Mistake 1**: Trying to build everything in one node
- **Solution**: Break your system into small, focused nodes

**Mistake 2**: Not understanding topic names
- **Solution**: Use descriptive names like `/camera/front/image` instead of `/img`

**Mistake 3**: Forgetting to source the workspace
- **Solution**: Always run `source install/setup.bash` before running nodes

**Mistake 4**: Not checking the ROS 2 graph
- **Solution**: Use `ros2 node list` and `ros2 topic list` frequently to verify connections

**Mistake 5**: Ignoring message types
- **Solution**: Always check what message type a topic uses before publishing or subscribing

### Performance Tips

**Topic frequency matters**: Don't publish camera images at 1000Hz if you only process at 30Hz. Match publish rates to subscriber needs.

**QoS settings are powerful**: Choose "reliable" for critical data (control commands) and "best effort" for sensor data that's constantly updated.

**Use nodelets for speed**: For high-performance needs, nodes can share memory instead of copying messages.

**Monitor your system**: Use `ros2 topic hz /topic_name` to check message rates and find bottlenecks.

### Development Environment

**Recommended setup**:
- **OS**: Ubuntu Linux (easiest ROS 2 support)
- **IDE**: VS Code with ROS extensions
- **Terminal**: Multiple terminal windows (one per node while developing)
- **Visualization**: RViz2 for seeing sensor data and robot state
- **Simulation**: Gazebo for testing

### Documentation and Resources

**Official documentation**: [docs.ros.org](https://docs.ros.org)

**Community forum**: [answers.ros.org](https://answers.ros.org)

**Package index**: [index.ros.org](https://index.ros.org) - find existing packages before building your own

**Tutorials**: Start with the official ROS 2 tutorials, which are excellent and well-maintained

### Hardware Considerations

**Single board computers**: Raspberry Pi 4 or NVIDIA Jetson boards work well for ROS 2

**Communication**: ROS 2 works over WiFi, Ethernet, or even across the internet

**Sensors**: Most modern robotics sensors have ROS 2 drivers already available

**Robot platforms**: Many commercial robot platforms (TurtleBot, Clearpath robots) come with ROS 2 support

### When NOT to Use ROS 2

ROS 2 isn't always the answer:

**Very simple projects**: If you're just blinking an LED, ROS 2 is overkill

**Hard real-time constraints**: ROS 2 has real-time capabilities, but if you need microsecond precision, consider dedicated real-time systems

**Extremely resource-constrained**: If you have only a tiny microcontroller, ROS 2 might be too heavy (though micro-ROS exists for this)

**Commercial products without modifications**: Some companies prefer complete control over their stack

## Summary

Let's review the key concepts from this chapter:

**ROS 2 (Robot Operating System 2)** is a framework for building robot software. It's not an operating system, but rather a set of tools and libraries for robot communication and coordination.

**The four core components** of ROS 2 are:
1. **Nodes**: Individual programs that each do one job
2. **Topics**: Communication channels for continuous data streaming (publish-subscribe)
3. **Services**: Request-response communication for immediate queries
4. **Actions**: Long-running tasks with progress feedback and cancellation

**The key principle** is modularity—breaking complex robot systems into small, focused, reusable pieces that communicate through well-defined interfaces.

**Topics use publish-subscribe**: Publishers send data without knowing who receives it. Subscribers receive data without knowing who sent it. They connect through topic names.

**Services are synchronous**: The client sends a request and waits for a response. Used for quick queries.

**Actions are asynchronous**: The client sends a goal and receives periodic feedback. Used for long tasks that can be cancelled.

**ROS 2 improves on ROS 1** with better real-time support, security, cross-platform compatibility, and multi-robot capabilities.

**The ROS 2 ecosystem** includes thousands of existing packages, strong community support, excellent simulation integration, and industry-wide adoption.

**Best practices** include starting simple, using descriptive names, testing each node independently, and leveraging existing packages before building your own.

**Common tools** like `ros2 node list`, `ros2 topic echo`, and `ros2 launch` help you understand and debug your robot systems.

## Looking Ahead

Now that you understand what ROS 2 is and how its components work together, you're ready to start building ROS 2 systems.

In the next chapter, we'll set up a ROS 2 development environment and write your first nodes. You'll create a simple publisher-subscriber system, learn to use launch files, and visualize your robot system using ROS 2's built-in tools.

You'll also learn how to integrate ROS 2 with Gazebo simulation, allowing you to test robot behaviors safely before deploying to real hardware.

---

## Review Questions

Test your understanding:

1. What are the four core components of ROS 2? Describe each in one sentence.

2. When would you use a topic versus a service versus an action? Give an example of each.

3. What is a node, and why do we break robot systems into multiple nodes instead of one large program?

4. Explain the publish-subscribe pattern. Why don't publishers and subscribers need to know about each other?

5. What are three key improvements in ROS 2 compared to ROS 1?

6. Draw a simple ROS 2 graph for a robot that uses a camera to detect faces and moves toward people.

## Hands-On Challenge

**Observation Exercise**: Find a video of a ROS 2 robot demonstration (search "ROS 2 robot demo" on YouTube). While watching, try to identify:

- What nodes might be running? (camera node, navigation node, etc.)
- What topics might they be using? (/camera/image, /cmd_vel, etc.)
- Where might services or actions be used?
- Draw a simple graph showing how you think the nodes communicate

This exercise will help you start thinking in terms of ROS 2 architecture when looking at any robot system.

## Key Takeaways

Remember these essential points:

✅ ROS 2 is a framework, not an operating system
✅ Modularity is the core principle—small, focused nodes working together
✅ Topics for streaming data, services for queries, actions for long tasks
✅ The ROS 2 graph visualizes your entire system's communication
✅ Existing packages can save you months of development time
✅ Simulation first, hardware second for safe, efficient development

In the next chapter, we'll move from theory to practice and get your hands on actual ROS 2 code!