# Chapter 3: ROS 2 Architecture & Core Concepts

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the layered architecture of ROS 2
- Explain how nodes communicate through topics, services, and actions
- Describe the role of DDS middleware in ROS 2
- Identify when to use each communication pattern
- Understand Quality of Service (QoS) settings and their impact
- Recognize how the computational graph represents your robot system

## Concept Explanation

### What is ROS 2 Architecture?

Think of **architecture** as the blueprint for how ROS 2 is built. Just like a house has a foundation, walls, and a roof working together, ROS 2 has different layers that work together to make robots function.

ROS 2's architecture is designed around one simple but powerful idea: **break big problems into small pieces that talk to each other**.

Instead of writing one massive program that controls your entire robot, you write many small programs (called **nodes**) that each do one thing well. These nodes communicate with each other to accomplish complex tasks.

### The Three Layers of ROS 2

ROS 2 is built in three layers, like a three-layer cake:

#### Layer 1: Application Layer (Top)

This is where YOUR code lives:
- The nodes you write in Python or C++
- Pre-built packages you download from the community
- The actual robot behaviors and logic

**Analogy**: This is like the apps on your smartphone—the programs you interact with directly.

#### Layer 2: ROS 2 Client Libraries (Middle)

This layer provides tools to build nodes:
- **rclpy**: Python library for creating ROS 2 nodes
- **rclcpp**: C++ library for creating ROS 2 nodes
- Handles communication between nodes
- Provides standard interfaces for topics, services, and actions

**Analogy**: This is like the operating system on your phone—it provides the tools that apps use to function.

#### Layer 3: DDS Middleware (Bottom)

This is the "engine" that makes communication work:
- Actually sends messages between nodes
- Handles network communication
- Manages node discovery
- Provides reliability and security

**Analogy**: This is like the cellular network—the invisible infrastructure that actually transmits data.

### Why This Layered Design?

**Separation of concerns**: You focus on robot logic (Layer 1), without worrying about how messages travel across networks (Layer 3).

**Flexibility**: You can swap out different DDS implementations without changing your code.

**Portability**: The same code runs on different robots, different computers, even different operating systems.

### Core Concept 1: Nodes

**What is a Node?**

A **node** is a single running program that performs one specific task. Think of nodes as workers in a factory—each worker has one job to do.

**Examples of nodes**:
- **Camera node**: Captures images from a camera
- **Object detector node**: Finds objects in images
- **Motor controller node**: Makes wheels turn
- **Navigation node**: Plans paths to destinations

**Why multiple nodes?**

Imagine trying to cook a meal where one person must shop, prep, cook, serve, and clean. It's overwhelming! Better to have different people handling different tasks.

Similarly, breaking your robot into nodes means:
- ✅ Easier to write (one simple task per node)
- ✅ Easier to test (test each node separately)
- ✅ Easier to debug (find which node has the problem)
- ✅ Easier to reuse (same camera node works on different robots)
- ✅ Parallel execution (multiple nodes run simultaneously)

**Node characteristics**:
- Each node has a unique name (like `/camera_front` or `/navigation`)
- Nodes run as separate processes
- Nodes can be on the same computer or distributed across multiple computers
- Nodes automatically discover each other when started

### Core Concept 2: Topics (Publish-Subscribe)

**What is a Topic?**

A **topic** is a named channel where nodes can send and receive messages. It's like a TV channel—some nodes broadcast (publish) on the channel, while other nodes tune in (subscribe) to watch.

**The Radio Station Analogy**:
- **Topic name**: The radio frequency (101.5 FM)
- **Publisher**: The radio station broadcasting
- **Subscriber**: Your radio receiving the signal
- **Message**: The music or news being transmitted

**Key features**:

**Anonymous**: Publishers don't know who's listening. Subscribers don't know who's broadcasting. They just share a topic name.

**Many-to-Many**: 
- One publisher can send to many subscribers ✓
- Many publishers can send to one subscriber ✓
- Many publishers to many subscribers ✓

**Continuous**: Data flows continuously while the publisher is active.

**Example scenario**:
```
Camera Node → publishes images → Topic: /camera/image → Vision Node subscribes
                                                      ↘ Logger Node subscribes
```

The camera publishes one stream of images to `/camera/image`. Both the vision node and logger node receive every image. The camera doesn't know or care who's listening.

**When to use topics**:
- Continuous data streams (sensor readings, camera images)
- One source, multiple consumers
- Data that updates frequently
- Real-time information flow

### Core Concept 3: Services (Request-Response)

**What is a Service?**

A **service** is like asking someone a question and waiting for an answer. One node sends a request, another node responds, and the requesting node waits for that response.

**The Restaurant Analogy**:
- You (client): "Do you have any soup?"
- Waiter (server): Checks the kitchen
- Waiter responds: "Yes, we have tomato soup"
- You wait during this entire exchange

**Key features**:

**Synchronous**: The requester waits (blocks) until it gets a response.

**One-to-One**: One request gets exactly one response.

**Transient**: No ongoing connection—just request → response → done.

**Example scenario**:
```
Navigation Node → Request: "Is path clear?" → Service: /check_path
                                                       ↓
Obstacle Detector ← Response: "Yes" or "No" ← Service: /check_path
```

The navigation node asks "is the path clear?" and WAITS for the obstacle detector to check sensors and respond.

**When to use services**:
- You need an immediate answer
- The operation completes quickly (typically < 1 second)
- One-time queries or commands
- Setting parameters or configurations

**When NOT to use services**:
- Continuous data streams (use topics)
- Long operations (use actions)
- Multiple simultaneous requesters competing for one server

### Core Concept 4: Actions (Goal-Feedback-Result)

**What is an Action?**

An **action** handles tasks that take time and provides progress updates along the way. You can also cancel actions midway if needed.

**The Pizza Delivery Analogy**:
- You order a pizza (send goal)
- Restaurant accepts your order
- You get updates: "Preparing... Baking... Out for delivery..." (feedback)
- Pizza arrives (result) OR you cancel the order partway

**Key features**:

**Asynchronous**: You send the goal and continue doing other things while it executes.

**Progress feedback**: Regular updates on how it's going.

**Cancellable**: You can stop the action before it completes.

**Three-phase communication**:
1. **Goal**: What you want accomplished
2. **Feedback**: Periodic progress updates
3. **Result**: Final outcome (success or failure)

**Example scenario**:
```
Controller → Goal: "Navigate to kitchen (10, 5)"
                ↓
Navigation Action Server → Accepts goal
                ↓
Feedback: "Distance remaining: 8m"
Feedback: "Distance remaining: 5m"
Feedback: "Distance remaining: 2m"
                ↓
Result: "Goal reached successfully!"
```

Throughout this process, the controller can:
- Monitor progress through feedback
- Cancel navigation if something changes
- Continue doing other tasks while navigation happens

**When to use actions**:
- Tasks that take more than a second or two
- You need progress updates
- Tasks that might need to be cancelled
- Long-running operations (navigation, manipulation, charging)

### Comparison: Topics vs Services vs Actions

Let's clarify when to use each:

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Communication** | One-way streaming | Request-Response | Goal-Feedback-Result |
| **Timing** | Continuous | Immediate | Long-running |
| **Blocking** | No (asynchronous) | Yes (synchronous) | No (asynchronous) |
| **Progress Updates** | N/A | No | Yes |
| **Cancellable** | N/A | No | Yes |
| **Best for** | Sensor data, status | Quick queries | Complex tasks |

**Quick decision guide**:

**Use Topics when**:
- "I need to constantly monitor this" (camera feed, battery status)
- "Multiple nodes need this data" (sensor readings)
- "This updates frequently" (robot position)

**Use Services when**:
- "I need to ask a quick question" (Is door open?)
- "I need to trigger something now" (Save map)
- "I need an immediate confirmation" (Did you receive this?)

**Use Actions when**:
- "This will take a while" (Drive to location)
- "I want to track progress" (Picking up object)
- "I might need to stop it" (Cancel navigation)

### Core Concept 5: The Computational Graph

The **computational graph** is a visual representation of your robot system showing:
- All running nodes (as circles or boxes)
- All topics connecting them (as arrows)
- Services and actions (as connections)

**Why it matters**: The graph lets you see your entire system at a glance. It's like a map showing how information flows through your robot.

**Example graph**:
```
[Camera] → /image → [Detector] → /objects → [Planner] → /cmd_vel → [Motors]
                                                ↑
                                            [Sensors] → /scan
```

This shows:
- Camera publishes images
- Detector finds objects
- Sensors provide LIDAR data
- Planner decides movements
- Motors execute commands

**Visualizing your graph**: ROS 2 provides tools to see your graph in real-time:
```bash
ros2 run rqt_graph rqt_graph
```

This opens a window showing all active nodes and their connections.

### Core Concept 6: Messages and Message Types

**What are Messages?**

**Messages** are the actual data being sent through topics, services, and actions. They have a defined structure, like a form with specific fields.

**Think of it like a job application**: Every job application has the same fields (name, address, phone), but different values for each person.

**Common message types**:

**Standard Messages** (`std_msgs`):
- `String`: Text data
- `Int32`: Integer numbers
- `Float64`: Decimal numbers
- `Bool`: True or false

**Geometry Messages** (`geometry_msgs`):
- `Point`: 3D coordinate (x, y, z)
- `Pose`: Position + orientation
- `Twist`: Velocity commands (linear and angular speed)

**Sensor Messages** (`sensor_msgs`):
- `Image`: Camera image data
- `LaserScan`: LIDAR measurements
- `Imu`: Inertial measurement data

**Example message structure**:
```
Message Type: geometry_msgs/Twist

Contents:
  linear:
    x: 0.5    # Move forward at 0.5 m/s
    y: 0.0    
    z: 0.0    
  angular:
    x: 0.0    
    y: 0.0    
    z: 0.3    # Turn at 0.3 rad/s
```

This tells a robot: "Move forward at 0.5 m/s while turning at 0.3 rad/s."

**Creating custom messages**:

Sometimes standard messages aren't enough. You can create custom ones:

```
# PersonDetection.msg
string name
float64 confidence
geometry_msgs/Point location
int32 estimated_age
```

This custom message includes a person's name, detection confidence, location, and estimated age.

### Core Concept 7: Quality of Service (QoS)

**What is QoS?**

**Quality of Service** settings control HOW messages are delivered. It's like choosing between regular mail, priority mail, or certified mail for sending a letter.

Different data has different requirements:
- Control commands must arrive reliably
- Sensor data can tolerate some loss
- Video streams need speed over perfection

**Key QoS Settings**:

**1. Reliability**:
- **Reliable**: Guarantees delivery (like certified mail)
- **Best Effort**: Send and hope it arrives (like regular mail)

When to use:
- Reliable → Control commands, important state changes
- Best Effort → High-frequency sensors (camera, LIDAR)

**2. Durability**:
- **Transient Local**: Save recent messages for late-joining subscribers
- **Volatile**: Only send to current subscribers

When to use:
- Transient → Configuration, system parameters
- Volatile → Real-time sensor data

**3. History**:
- **Keep Last N**: Store only N most recent messages
- **Keep All**: Store everything until delivered

When to use:
- Keep Last 1 → Latest sensor reading
- Keep Last 10 → Recent command history
- Keep All → Critical commands

**Example QoS choices**:

**Camera topic** (30 fps):
```python
QoS:
  Reliability: Best Effort
  History: Keep Last 1
  
Reason: New frames arrive constantly. 
        If one is lost, the next one is coming anyway.
        Only the latest frame matters.
```

**Motor commands**:
```python
QoS:
  Reliability: Reliable
  History: Keep Last 10
  
Reason: Commands must arrive.
        Keep recent history in case of delays.
```

**QoS Compatibility**:

Publishers and subscribers must have compatible QoS:
- Reliable publisher → Reliable subscriber ✓
- Best Effort publisher → Best Effort subscriber ✓
- Reliable publisher → Best Effort subscriber ✓
- Best Effort publisher → Reliable subscriber ✗ (Won't connect!)

### Core Concept 8: DDS Middleware

**What is DDS?**

**DDS** (Data Distribution Service) is the technology underneath ROS 2 that actually transmits messages between nodes. It's the "postal service" that delivers your data.

**Why ROS 2 uses DDS**:

ROS 1 had a custom communication system. It worked but had problems:
- Not designed for safety-critical systems
- No built-in security
- Difficult to use across networks
- Limited real-time capabilities

ROS 2 switched to DDS because:
- ✅ Industry-proven (used in aerospace, military, medical devices)
- ✅ Built-in security (encryption, authentication)
- ✅ Real-time capable
- ✅ Scales to thousands of nodes
- ✅ Automatic node discovery
- ✅ Works across multiple computers easily

**The good news**: You rarely interact with DDS directly. ROS 2's client libraries handle it for you. It's like driving a car—you don't need to understand the engine to drive.

**DDS Implementations**:

Multiple DDS vendors exist:
- **Fast DDS** (default)
- **CycloneDDS**
- **Connext DDS**

You can switch between them without changing your code. ROS 2 abstracts the differences.

### Core Concept 9: Workspaces and Packages

**What is a Workspace?**

A **workspace** is a folder containing all your ROS 2 projects. It's your development environment.

**Structure**:
```
my_robot_ws/              # Workspace root
├── src/                  # Your source code
│   ├── camera_pkg/
│   ├── navigation_pkg/
│   └── sensors_pkg/
├── build/                # Temporary build files
├── install/              # Compiled code (what runs)
└── log/                  # Build logs
```

**What is a Package?**

A **package** is a collection of related code—like a mini-project. Each package contains:
- Python or C++ source files
- Launch files (to start nodes)
- Configuration files
- Message/service/action definitions

**Package structure**:
```
my_robot_pkg/
├── package.xml           # Package metadata
├── setup.py              # Python configuration
├── my_robot_pkg/         # Python code
│   ├── __init__.py
│   ├── camera_node.py
│   └── controller_node.py
├── launch/               # Launch files
│   └── robot.launch.py
└── config/               # Configuration files
    └── params.yaml
```

**Why packages?**

- ✅ Keep related code together
- ✅ Declare dependencies clearly
- ✅ Easy to share with others
- ✅ Reusable across projects

### Core Concept 10: Launch Files

**What are Launch Files?**

**Launch files** start multiple nodes at once with one command. Instead of manually starting 10 nodes, one launch file starts them all.

**The problem they solve**:

Without launch files:
```bash
ros2 run pkg1 node1
ros2 run pkg2 node2
ros2 run pkg3 node3
ros2 run pkg4 node4
# ... 6 more times
```

With launch files:
```bash
ros2 launch my_pkg robot.launch.py
```

Everything starts automatically!

**What launch files do**:
- Start multiple nodes
- Configure parameters for each node
- Set up remappings (change topic names)
- Include other launch files
- Add conditional logic (start node X only if Y is true)

**Simple example**:
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
    ])
```

This launches both the camera and detector nodes together.

## Why This Matters

### Real-World Impact

Understanding ROS 2 architecture is essential because:

**Industry Adoption**: Companies worldwide use ROS 2:
- Manufacturing robots (ABB, KUKA)
- Autonomous vehicles (Tier IV, Apex.AI)
- Warehouse robots (Amazon, Otto Motors)
- Service robots (Fetch Robotics, PAL Robotics)
- Agricultural robots (FarmWise, Blue River Technology)

**Career Readiness**: Robotics jobs expect you to understand:
- Distributed system design
- Inter-process communication
- Message-based architectures
- Real-time systems

ROS 2 teaches all of these.

**Avoiding Reinvention**: ROS 2 solves hard problems:
- How do multiple processes communicate reliably?
- How do nodes discover each other?
- How do we handle network delays?
- How do we ensure security?

Without ROS 2, you'd spend months solving these problems yourself.

### Why This Architecture Works

**Modularity**: Small, focused nodes are easier to:
- Write (one task at a time)
- Test (isolate and verify)
- Debug (identify problem nodes)
- Reuse (same node, different robots)

**Scalability**: Start with a few nodes, grow to hundreds:
- Add new features without breaking existing ones
- Distribute across multiple computers
- Run multiple robots that coordinate

**Flexibility**: Change parts without rebuilding everything:
- Swap camera types (same interface)
- Replace navigation algorithms
- Add new sensors incrementally

**Team Collaboration**: Different people work on different nodes:
- Camera team builds camera nodes
- Vision team builds detection nodes
- Control team builds navigation nodes
- Everything integrates through standard interfaces

### Learning Curve Payoff

Yes, ROS 2 has complexity. But this complexity pays off:

**Week 1**: "This is confusing, so many concepts!"
**Month 1**: "I'm starting to see how it fits together"
**Month 3**: "This makes building robots so much easier!"
**Year 1**: "I can't imagine building robots without ROS 2"

The architecture might seem complex at first, but once you understand it, you'll build robots faster than you ever thought possible.

## Example

Let's build something concrete: **A simple home security robot** that patrols rooms and detects intruders.

### System Requirements

Our security robot must:
1. Move through rooms autonomously
2. Use camera to detect people
3. Sound alarm when intruder detected
4. Send alerts to homeowner's phone
5. Return to charging station when battery low

### Designing the Architecture

Let's design this using ROS 2 concepts.

#### Step 1: Identify Nodes

Break the system into focused nodes:

**1. Camera Node**
- Responsibility: Capture images
- Publishes: `/camera/image` topic (sensor_msgs/Image)

**2. Person Detector Node**
- Responsibility: Find people in images
- Subscribes: `/camera/image`
- Publishes: `/detected_persons` topic (custom PersonArray message)

**3. Navigation Node**
- Responsibility: Move robot through patrol route
- Action Server: `/patrol` action
- Subscribes: `/map` topic
- Publishes: `/cmd_vel` topic (velocity commands)

**4. Alarm Controller Node**
- Responsibility: Sound alarm when intruder found
- Subscribes: `/detected_persons`
- Publishes: `/alarm` topic (std_msgs/Bool)

**5. Network Alert Node**
- Responsibility: Send alerts to phone
- Subscribes: `/detected_persons`
- Service Client: Calls external notification API

**6. Battery Monitor Node**
- Responsibility: Track battery status
- Publishes: `/battery_status` topic
- Service Server: `/get_battery_level`

**7. Charging Dock Node**
- Responsibility: Return to dock when battery low
- Subscribes: `/battery_status`
- Action Client: Calls `/navigate_to_dock` action

#### Step 2: Design Communication

**Topics** (continuous data):
- `/camera/image`: Image stream (30 Hz)
- `/detected_persons`: Person detections (5 Hz)
- `/cmd_vel`: Velocity commands (10 Hz)
- `/battery_status`: Battery percentage (1 Hz)
- `/alarm`: Alarm on/off (when changed)

**Services** (quick queries):
- `/get_battery_level`: Ask current battery percentage
- `/set_patrol_route`: Configure patrol path

**Actions** (long tasks):
- `/patrol`: Execute patrol route (can take minutes)
- `/navigate_to_dock`: Return to charging station

#### Step 3: Data Flow

Let's trace a complete scenario: **Robot detects an intruder**

```
1. Camera Node → publishes image → /camera/image (30 fps)
                                          ↓
2. Person Detector subscribes, processes image
                                          ↓
3. Detects person at location (5, 3)
                                          ↓
4. Person Detector → publishes → /detected_persons
                                          ↓
                                    ↙           ↘
5. Alarm Controller subscribes          Network Alert subscribes
         ↓                                      ↓
6. Publishes: /alarm = TRUE          Sends notification to phone
         ↓
7. Physical alarm sounds!
```

**Simultaneously**, battery monitoring happens:

```
Battery Monitor → publishes → /battery_status (15%)
                                      ↓
                      Charging Dock Node subscribes
                                      ↓
                           Detects low battery
                                      ↓
              Sends action goal: /navigate_to_dock
                                      ↓
                        Navigation Node executes
                                      ↓
                  Feedback: "Distance to dock: 10m"
                  Feedback: "Distance to dock: 5m"
                  Feedback: "Distance to dock: 1m"
                                      ↓
                        Result: "Docked successfully"
```

#### Step 4: The Computational Graph

Visual representation:

```
[Camera] → /camera/image → [Person Detector] → /detected_persons → [Alarm Controller]
                                                        ↓
                                               [Network Alert]

[Battery Monitor] → /battery_status → [Charging Dock] ⟲ /navigate_to_dock action
                                                                ↓
                                                        [Navigation Node]
                                                                ↓
                                                            /cmd_vel
                                                                ↓
                                                            [Motors]
```

#### Step 5: Message Types

**Custom message for person detection**:

```
# PersonDetection.msg
geometry_msgs/Point location
float64 confidence
string timestamp
```

**Standard messages used**:
- `sensor_msgs/Image`: Camera images
- `geometry_msgs/Twist`: Movement commands
- `std_msgs/Bool`: Alarm state
- `sensor_msgs/BatteryState`: Battery info

#### Step 6: QoS Configuration

Choose QoS based on data characteristics:

**Camera images** (high frequency, best effort OK):
```python
qos_camera = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Only latest frame matters
)
```

**Person detections** (critical, must not lose):
```python
qos_detections = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Keep recent detections
)
```

**Velocity commands** (critical, need reliability):
```python
qos_control = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

#### Step 7: Launch File

Start the entire system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensors
        Node(
            package='camera_pkg',
            executable='camera_node',
            name='security_camera'
        ),
        
        # Processing
        Node(
            package='vision_pkg',
            executable='person_detector',
            name='intruder_detector'
        ),
        
        # Control
        Node(
            package='navigation_pkg',
            executable='patrol_nav',
            name='patrol_navigator'
        ),
        
        # Alerts
        Node(
            package='security_pkg',
            executable='alarm_controller',
            name='alarm'
        ),
        Node(
            package='security_pkg',
            executable='network_alert',
            name='phone_notifier'
        ),
        
        # Utilities
        Node(
            package='battery_pkg',
            executable='battery_monitor',
            name='power_monitor'
        ),
        Node(
            package='charging_pkg',
            executable='dock_controller',
            name='dock'
        ),
    ])
```

Run with: `ros2 launch security_robot full_system.launch.py`

### Why This Design Works

**Modularity**: Each node has one clear responsibility
- Camera just captures images
- Detector just finds people
- Alarm just makes noise

**Reusability**: The camera node works for any robot needing a camera

**Testability**: Test person detector with recorded images before running on robot

**Extensibility**: Easy to add features:
- Add face recognition node (subscribes to `/camera/image`)
- Add motion detection node (another image subscriber)
- Add logging node (subscribes to all topics)

**Fault Tolerance**: If person detector crashes, camera and navigation keep working

**Parallel Execution**: Detection, navigation, and battery monitoring run simultaneously

## Practical Notes

### Getting Started Tips

**Start with the Basics**:
1. First, understand nodes and topics
2. Then add services for queries
3. Finally, learn actions for complex tasks
4. Build complexity gradually

**Use Visualization Tools**:
```bash
# See all running nodes
ros2 node list

# See all topics
ros2 topic list

# Visualize the graph
ros2 run rqt_graph rqt_graph

# Monitor a topic
ros2 topic echo /topic_name
```

These tools help you understand what's happening in your system.

### Common Beginner Mistakes

**Mistake 1: Creating "God Nodes"**
❌ One node that does everything
✓ Break into focused nodes (one responsibility each)

**Mistake 2: Wrong Communication Pattern**
❌ Using topics when you need a service
❌ Using services for continuous data
✓ Match pattern to use case (see comparison table above)

**Mistake 3: Ignoring QoS**
❌ Using default QoS everywhere
✓ Choose QoS based on data requirements

**Mistake 4: Not Naming Things Clearly**
❌ Topic names like `/data` or `/topic1`
✓ Descriptive names like `/camera/front/image`

**Mistake 5: Not Testing Incrementally**
❌ Build entire system, then test
✓ Test each node individually first

### Debugging Strategies

**When things don't work**:

**Step 1**: Verify all nodes are running
```bash
ros2 node list
```

**Step 2**: Check if topics are being published
```bash
ros2 topic list
ros2 topic hz /topic_name  # Check publish rate
```

**Step 3**: Inspect messages
```bash
ros2 topic echo /topic_name
```

**Step 4**: Check node connections
```bash
ros2 node info /node_name
```

**Step 5**: Verify QoS compatibility
Publishers and subscribers must have compatible QoS settings.

### Performance Considerations

**Topic Frequency**:
- Don't publish faster than subscribers can process
- Camera at 30 Hz is usually sufficient (not 1000 Hz)

**Message Size**:
- Large images consume bandwidth
- Consider compressing or reducing resolution
- Use efficient message types

**Node Distribution**:
- Keep high-bandwidth topics on same machine
- Distribute computation across multiple computers when needed
- ROS 2 handles multi-machine communication automatically

### Best Practices

**Design Before Coding**:
1. Draw your computational graph on paper
2. List all nodes and their responsibilities
3. Identify topics, services, and actions
4. Choose message types
5. Plan QoS settings

**Use Standard Messages When Possible**:
- Standard messages are well-tested
- Work with existing tools
- Make your code compatible with other packages

**Name Things Descriptively**:
- Nodes: `/robot/subsystem/function`
- Topics: `/robot/sensor/data_type`
- Services: `/system/action_verb`

**Document Your Architecture**:
- Draw and save your computational graph
- List all topics with their message types
- Explain what each node does
- Include example commands to run the system

**Test Incrementally**:
- Get one node working
- Add one more node
- Verify communication
- Repeat

### Learning Resources

**Official ROS 2 Documentation**:
- docs.ros.org
- Comprehensive tutorials
- API references

**Community Support**:
- answers.ros.org (Q&A forum)
- discourse.ros.org (discussion forum)
- ROS Discord server

**Practice Projects**:
- Start with official tutorials
- Build simple 2-3 node systems
- Gradually increase complexity
- Study existing ROS 2 packages

## Summary

Let's recap the essential concepts of ROS 2 architecture:

**ROS 2 has three layers**:
1. Application layer (your code)
2. Client libraries (rclpy/rclcpp)
3. DDS middleware (communication infrastructure)

**Nodes** are independent programs that each perform one specific task. Breaking systems into nodes provides modularity, reusability, testability, and parallel execution.

**Three communication patterns**:
1. **Topics** (publish-subscribe): Continuous data streams, anonymous, many-to-many
2. **Services** (request-response): Synchronous queries, one-to-one, blocking
3. **Actions** (goal-feedback-result): Long-running tasks, progress updates, cancellable

**The computational graph** visualizes your system—showing all nodes and how they communicate through topics, services, and actions.

**Messages** define data structure. Use standard message types when possible; create custom messages when needed.

**Quality of Service (QoS)** controls message delivery:
- Reliability (reliable vs best effort)
- Durability (transient vs volatile)
- History (keep last N vs keep all)

**DDS middleware** provides the underlying communication infrastructure with reliability, security, and scalability