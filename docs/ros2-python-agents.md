# Chapter 4: Building ROS 2 Packages with Python

## Learning Objectives

By the end of this chapter, you will be able to:

- Create a ROS 2 workspace from scratch
- Build a Python package with proper structure
- Write your first ROS 2 node in Python
- Create publishers that send messages on topics
- Create subscribers that receive messages from topics
- Build and run your package
- Debug common package setup issues
- Understand package dependencies and configuration files

## Concept Explanation

### What is a ROS 2 Package?

A **ROS 2 package** is a folder containing organized code that does something specific. Think of it like an app on your phone—each app (package) has a specific purpose and all its files are bundled together.

**Package = Code + Configuration + Resources**

Every package contains:
- **Source code**: Your Python scripts (the actual nodes)
- **Configuration files**: Tell ROS 2 about your package
- **Resources**: Launch files, parameters, custom messages

**Why packages?**

Instead of having scattered Python files everywhere, packages keep everything organized:
- ✅ All related code stays together
- ✅ Easy to share with others
- ✅ Clear dependencies (what other packages you need)
- ✅ Reusable across different robots

### What is a ROS 2 Workspace?

A **workspace** is the home for all your packages. It's like a folder on your computer where you keep all your ROS 2 projects.

**Workspace structure**:
```
my_robot_ws/              # Workspace root
├── src/                  # All packages go here
│   ├── package1/
│   ├── package2/
│   └── package3/
├── build/                # Temporary build files (ignore this)
├── install/              # Compiled code (what actually runs)
└── log/                  # Build logs (for debugging)
```

**The `src` folder** is where YOU work—this is where you create and edit packages.

**The `install` folder** is what ROS 2 actually runs—after you build your workspace, runnable code goes here.

### Python Package Structure

A Python ROS 2 package has a specific structure. Here's what it looks like:

```
my_robot_package/                    # Package root folder
├── package.xml                      # Package metadata (REQUIRED)
├── setup.py                         # Python package info (REQUIRED)
├── setup.cfg                        # Python configuration (REQUIRED)
├── resource/                        # Resource marker folder
│   └── my_robot_package            # Empty marker file
├── my_robot_package/                # Python module (same name as package)
│   ├── __init__.py                 # Makes this a Python module
│   ├── my_first_node.py            # Your node code
│   └── my_second_node.py           # Another node
├── launch/                          # Launch files (optional)
│   └── my_launch.py
├── config/                          # Configuration files (optional)
│   └── params.yaml
└── test/                            # Unit tests (optional)
    └── test_my_node.py
```

Let's understand each important file:

#### 1. package.xml

This file describes your package—like a label on a box telling you what's inside.

**What it contains**:
- Package name
- Version number
- Description
- Author/maintainer info
- **Dependencies**: What other packages you need

**Simple example**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Key parts**:
- `<name>`: Your package name (must match folder name)
- `<depend>`: Packages you need (like rclpy for Python ROS 2)
- `<build_type>`: Tells ROS 2 this is a Python package

#### 2. setup.py

This file tells Python how to install your package and where your nodes are.

**Simple example**:
```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_first_node:main',
        ],
    },
)
```

**Most important part** is `entry_points`:
```python
'my_node = my_robot_package.my_first_node:main'
```

This says: "Create a command called `my_node` that runs the `main()` function in `my_first_node.py`"

#### 3. setup.cfg

This is a simple configuration file. Usually looks like this:

```ini
[develop]
script_dir=$base/lib/my_robot_package
[install]
install_scripts=$base/lib/my_robot_package
```

You rarely need to change this—just copy it as-is.

### Creating Your First Node

A **node** in Python is just a Python class that inherits from `Node`. Let's break down what a simple node looks like.

**Basic node template**:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node has started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Let's understand each part**:

**Import statements**:
```python
import rclpy                    # ROS 2 Python library
from rclpy.node import Node     # Base Node class
```

**Node class**:
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')  # Give your node a name
        self.get_logger().info('Node has started!')  # Log message
```

- `super().__init__('my_node_name')`: Creates a ROS 2 node with this name
- `self.get_logger().info()`: Prints messages (like `print()` but better)

**Main function**:
```python
def main(args=None):
    rclpy.init(args=args)        # Initialize ROS 2
    node = MyNode()              # Create your node
    rclpy.spin(node)             # Keep node running
    rclpy.shutdown()             # Clean up when done
```

- `rclpy.init()`: Starts up ROS 2
- `rclpy.spin(node)`: Keeps your node alive and processing callbacks
- `rclpy.shutdown()`: Closes everything cleanly

### Creating a Publisher

A **publisher** sends messages on a topic. Here's how to create one:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        
        # Create timer to publish regularly
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key parts**:

**Creating the publisher**:
```python
self.publisher = self.create_publisher(String, 'my_topic', 10)
```
- `String`: Message type (simple text)
- `'my_topic'`: Topic name
- `10`: Queue size (how many messages to buffer)

**Creating a timer**:
```python
self.timer = self.create_timer(1.0, self.timer_callback)
```
- `1.0`: Timer period in seconds (publish every 1 second)
- `self.timer_callback`: Function to call each time

**Publishing a message**:
```python
msg = String()
msg.data = 'Hello World'
self.publisher.publish(msg)
```

### Creating a Subscriber

A **subscriber** receives messages from a topic. Here's how:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key parts**:

**Creating the subscriber**:
```python
self.subscription = self.create_subscription(
    String,              # Message type
    'my_topic',          # Topic name
    self.listener_callback,  # Function to call when message arrives
    10                   # Queue size
)
```

**Callback function**:
```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: {msg.data}')
```

This function runs automatically every time a message arrives on the topic.

### Building and Running Your Package

Once you've created your package and nodes, you need to build and run them.

**Step 1: Build the workspace**

From your workspace root:
```bash
cd ~/my_robot_ws
colcon build
```

This compiles your package and puts it in the `install/` folder.

**Step 2: Source the workspace**

Tell your terminal where to find your package:
```bash
source install/setup.bash
```

You need to do this in every new terminal window.

**Step 3: Run your node**

```bash
ros2 run my_robot_package my_node
```

- `my_robot_package`: Your package name
- `my_node`: The entry point name from setup.py

### Common Build Options

**Build specific package**:
```bash
colcon build --packages-select my_robot_package
```

**Build with verbose output** (see what's happening):
```bash
colcon build --event-handlers console_direct+
```

**Clean build** (start fresh):
```bash
rm -rf build install log
colcon build
```

## Why This Matters

### Real-World Development

Understanding packages is essential because:

**Professional Development**: Every ROS 2 project uses packages. Whether you're working at:
- Manufacturing companies (ABB, KUKA)
- Self-driving car companies (Cruise, Waymo)
- Service robot companies (Fetch Robotics)
- Research labs (NASA, universities)

Everyone organizes code into packages.

**Code Organization**: As projects grow, organization becomes critical:
- Small project: 5 nodes, manageable without packages
- Medium project: 50 nodes, needs multiple packages
- Large project: 500+ nodes, requires dozens of organized packages

**Team Collaboration**: Packages enable multiple people to work together:
- Camera team works on `camera_package`
- Navigation team works on `navigation_package`
- Control team works on `control_package`
- Teams integrate through package interfaces

**Code Reusability**: Well-built packages can be reused:
- Use the same camera package on 5 different robots
- Share your navigation package with the community
- Download others' packages to save development time

### Why Python for ROS 2?

Python is popular for ROS 2 because:

**Beginner-Friendly**: 
- Easier syntax than C++
- Faster to write and test
- Great for prototyping

**Rich Ecosystem**:
- Machine learning libraries (TensorFlow, PyTorch)
- Computer vision (OpenCV, scikit-image)
- Data processing (NumPy, Pandas)

**Good Performance**: 
- Fast enough for most robotics tasks
- Critical loops can be written in C++ if needed

**Industry Use**:
- Many companies use Python for robot control
- Research labs prefer Python for rapid development
- Educational institutions teach Python first

### Build System Benefits

The ROS 2 build system (colcon) provides:

**Dependency Management**: Automatically handles what packages depend on what

**Parallel Building**: Builds multiple packages simultaneously (faster!)

**Workspace Isolation**: Each workspace is independent

**Cross-Platform**: Same build commands on Linux, Windows, macOS

## Example

Let's build a complete example: **A simple chat system** with two nodes that send messages to each other.

### Goal

Create a package with:
1. A "talker" node that publishes messages
2. A "listener" node that subscribes to those messages
3. Messages sent every second
4. Counter to track how many messages sent

### Step-by-Step Implementation

#### Step 1: Create Workspace

```bash
# Create workspace folder
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### Step 2: Create Package

```bash
ros2 pkg create --build-type ament_python chat_package --dependencies rclpy std_msgs
```

This command:
- Creates a package called `chat_package`
- Sets it up as a Python package
- Adds dependencies on `rclpy` and `std_msgs`

**What gets created**:
```
chat_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── chat_package
├── chat_package/
│   └── __init__.py
└── test/
```

#### Step 3: Create Talker Node

Create file: `chat_package/chat_package/talker.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        # Create timer (publish every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Message counter
        self.counter = 0
        
        self.get_logger().info('Talker node started!')
        
    def timer_callback(self):
        # Create message
        msg = String()
        msg.data = f'Hello! Message #{self.counter}'
        
        # Publish message
        self.publisher.publish(msg)
        
        # Log what we sent
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment counter
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does**:
- Creates a publisher on topic `chatter`
- Every 1 second, sends a message
- Counter tracks total messages sent
- Logs each message

#### Step 4: Create Listener Node

Create file: `chat_package/chat_package/listener.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        
        # Track received messages
        self.message_count = 0
        
        self.get_logger().info('Listener node started!')
        
    def listener_callback(self, msg):
        # Increment counter
        self.message_count += 1
        
        # Log received message
        self.get_logger().info(f'Received: "{msg.data}" (Total: {self.message_count})')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does**:
- Creates a subscriber on topic `chatter`
- When messages arrive, calls `listener_callback()`
- Counts total messages received
- Logs each received message

#### Step 5: Update setup.py

Edit `chat_package/setup.py` to add entry points:

```python
from setuptools import setup

package_name = 'chat_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Simple chat package with talker and listener',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = chat_package.talker:main',
            'listener = chat_package.listener:main',
        ],
    },
)
```

**Key addition**:
```python
'talker = chat_package.talker:main',
'listener = chat_package.listener:main',
```

This creates two commands: `talker` and `listener`.

#### Step 6: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select chat_package
```

**You should see**:
```
Starting >>> chat_package
Finished <<< chat_package [1.23s]

Summary: 1 package finished [1.45s]
```

#### Step 7: Source the Workspace

```bash
source install/setup.bash
```

#### Step 8: Run the Nodes

**Terminal 1** - Start the listener:
```bash
ros2 run chat_package listener
```

**Output**:
```
[INFO] [listener]: Listener node started!
```

**Terminal 2** - Start the talker:
```bash
ros2 run chat_package talker
```

**Output**:
```
[INFO] [talker]: Talker node started!
[INFO] [talker]: Publishing: "Hello! Message #0"
[INFO] [talker]: Publishing: "Hello! Message #1"
[INFO] [talker]: Publishing: "Hello! Message #2"
...
```

**Terminal 1 now shows**:
```
[INFO] [listener]: Listener node started!
[INFO] [listener]: Received: "Hello! Message #0" (Total: 1)
[INFO] [listener]: Received: "Hello! Message #1" (Total: 2)
[INFO] [listener]: Received: "Hello! Message #2" (Total: 3)
...
```

**Success!** The talker is publishing messages, and the listener is receiving them.

#### Step 9: Verify with ROS 2 Tools

**Check running nodes**:
```bash
ros2 node list
```

Output:
```
/listener
/talker
```

**Check active topics**:
```bash
ros2 topic list
```

Output:
```
/chatter
/parameter_events
/rosout
```

**Monitor the topic**:
```bash
ros2 topic echo /chatter
```

You'll see messages appearing in real-time!

**Check message rate**:
```bash
ros2 topic hz /chatter
```

Output:
```
average rate: 1.000
```

Perfect! One message per second as expected.

### Understanding What Happened

Let's trace the complete flow:

**1. Build Process**:
```
setup.py → tells Python where nodes are
colcon build → compiles package
install/ → creates runnable commands
```

**2. Runtime Process**:
```
ros2 run chat_package talker → starts talker node
Talker creates publisher on /chatter
Timer triggers every 1 second
Message published to /chatter topic
```

**3. Communication**:
```
Listener subscribes to /chatter
DDS middleware detects publisher and subscriber
Messages flow from talker to listener
Listener callback executes for each message
```

**4. The Beauty of Decoupling**:
- Talker doesn't know listener exists
- Listener doesn't know talker exists
- They only share the topic name `/chatter`
- You could add 10 more listeners—talker wouldn't care
- You could have 5 talkers—listener would receive from all

## Practical Notes

### Development Workflow

**The standard workflow**:

1. **Write code** in `src/your_package/your_package/`
2. **Build** with `colcon build`
3. **Source** with `source install/setup.bash`
4. **Test** with `ros2 run your_package your_node`
5. **Debug** and repeat

**Pro tip**: Create an alias to speed up sourcing:
```bash
# Add to ~/.bashrc
alias src_ros='source ~/ros2_ws/install/setup.bash'
```

Then just type `src_ros` instead of the full command.

### Common Issues and Solutions

**Issue 1: "Package not found"**

```bash
ros2 run my_package my_node
# Error: Package 'my_package' not found
```

**Solution**: Did you source the workspace?
```bash
source install/setup.bash
```

**Issue 2: "No executable found"**

```bash
ros2 run my_package my_node
# Error: No executable found
```

**Solution**: Check your setup.py entry_points. The name must match.

**Issue 3: Build fails with "package.xml not found"**

**Solution**: Are you in the workspace root when building?
```bash
cd ~/ros2_ws  # Make sure you're here
colcon build
```

**Issue 4: Changes not taking effect**

**Solution**: Rebuild and re-source:
```bash
colcon build --packages-select my_package
source install/setup.bash
```

**Issue 5: Import errors**

```python
ModuleNotFoundError: No module named 'my_package'
```

**Solution**: Check three things:
1. Package folder name matches package name in setup.py
2. `__init__.py` exists in package folder
3. You rebuilt after creating new files

### Best Practices

**Package Naming**:
- Use lowercase with underscores: `robot_control_pkg`
- Be descriptive: `camera_driver` not `pkg1`
- Follow convention: `_pkg` or `_driver` suffix

**File Organization**:
```
my_package/
├── my_package/
│   ├── __init__.py
│   ├── nodes/              # All node files
│   │   ├── node1.py
│   │   └── node2.py
│   ├── utils/              # Helper functions
│   │   └── helpers.py
│   └── config/             # Constants
│       └── constants.py
```

**Code Style**:
- Use clear variable names: `camera_publisher` not `cp`
- Add docstrings to functions
- Keep nodes under 200 lines (split if larger)
- One node per file

**Logging Best Practices**:

```python
# Use appropriate log levels
self.get_logger().debug('Detailed information')  # Only in debug mode
self.get_logger().info('Normal information')     # Regular updates
self.get_logger().warn('Warning, but not critical')  # Concerning
self.get_logger().error('Error occurred!')       # Something broke
self.get_logger().fatal('Critical failure!')     # System failure
```

**Testing Your Nodes**:

Always test:
1. Does the node start without errors?
2. Are topics being published/subscribed correctly?
3. Are messages formatted correctly?
4. Does cleanup happen properly on shutdown?

**Use ROS 2 tools for verification**:
```bash
ros2 node list          # Check node is running
ros2 topic list         # Check topics exist
ros2 topic echo /topic  # See actual messages
ros2 topic hz /topic    # Check publish rate
ros2 node info /node    # See node details
```

### Development Tips

**Rapid Testing**:

Instead of building every time, you can test Python changes directly:
```bash
# Run Python file directly (for quick testing only)
python3 src/my_package/my_package/my_node.py
```

But remember: always do a proper build before committing code!

**Debugging with Print Statements**:

```python
# Instead of print(), use logger
self.get_logger().info(f'Variable value: {my_var}')
```

Benefits:
- Shows timestamps
- Shows node name
- Can be filtered by log level
- Appears in ROS 2 logging system

**Multiple Workspaces**:

You can have multiple workspaces:
```bash
~/ros2_ws         # Main workspace
~/test_ws         # Testing workspace
~/project_ws      # Project workspace
```

Just source the one you want to use.

**Overlay Workspaces**:

You can "overlay" workspaces—one on top of another:
```bash
source /opt/ros/humble/setup.bash    # ROS 2 installation
source ~/ros2_ws/install/setup.bash  # Your workspace

# Your workspace takes priority
```

### Package Dependencies

**Understanding dependencies** in package.xml:

```xml
<depend>rclpy</depend>              <!-- Build and run dependency -->
<build_depend>some_package</build_depend>  <!-- Only needed to build -->
<exec_depend>another_package</exec_depend>  <!-- Only needed to run -->
```

**Common dependencies**:
- `rclpy`: Required for all Python ROS 2 nodes
- `std_msgs`: Standard message types
- `sensor_msgs`: Sensor message types
- `geometry_msgs`: Geometry message types

**Adding new dependencies**:

1. Add to `package.xml`:
```xml
<depend>geometry_msgs</depend>
```

2. Add to `setup.py` if needed:
```python
install_requires=['setuptools', 'numpy'],
```

3. Rebuild:
```bash
colcon build --packages-select my_package
```

### Performance Considerations

**Queue Size**:
```python
self.publisher = self.create_publisher(String, 'topic', 10)
#                                                       ^^
#                                                  Queue size
```

- Too small: Messages get dropped
- Too large: Uses more memory
- Typical: 10 for most use cases
- High-frequency sensors: 1 (only latest matters)

**Timer Frequency**:
```python
self.timer = self.create_timer(0.1, self.callback)  # 10 Hz
self.timer = self.create_timer(0.01, self.callback) # 100 Hz
```

- Match frequency to actual need
- Don't publish faster than subscribers can process
- Consider CPU load

**Message Types**:
- Use appropriate message types
- Smaller messages = less bandwidth
- Custom messages when needed, standard when possible

## Summary

Let's recap what we learned about building ROS 2 packages with Python:

**A ROS 2 package** is an organized collection of code, configuration, and resources. Packages keep related code together and make it easy to share and reuse.

**Package structure** includes:
- `package.xml`: Package metadata and dependencies
- `setup.py`: Python package configuration and entry points
- `setup.cfg`: Python installation configuration
- Source code folder: Your Python nodes
- Optional folders: launch, config, test

**Workspaces** contain multiple packages. Structure:
- `src/`: Where you create and edit packages
- `build/`: Temporary build files
- `install/`: Compiled code that actually runs
- `log/`: Build logs

**Creating a publisher** in Python:
```python
self.publisher = self.create_publisher(MessageType, 'topic_name', queue_size)
self.publisher.publish(message)
```

**Creating a subscriber** in Python:
```python
self.subscription = self.create_subscription(
    MessageType, 'topic_name', self.callback, queue_size)
```

**Build and run workflow**:
1. Write code in `src/package_name/`
2. `colcon build` to compile
3. `source install/setup.bash` to activate
4. `ros2 run package_name node_name` to execute

**Common commands**:
- `ros2 pkg create`: Create new package
- `colcon build`: Build workspace
- `ros2 run`: Run a node
- `ros2 node list`: See running nodes
- `ros2 topic list`: See active topics

**Best practices**:
- Use descriptive names for packages and nodes
- Keep nodes focused (one responsibility)
- Use appropriate log levels
- Test with ROS 2 tools
- Organize code into logical folders
- Add proper dependencies to package.xml

**Common issues**:
- Forgetting to source the workspace
- Mismatched names in setup.py entry_points
- Missing `__init__.py` in package folder
- Building from wrong directory
- Not rebuilding after code changes

**Python benefits for ROS 2**:
- Beginner-friendly syntax
- Rapid development and prototyping
- Rich ecosystem of libraries
- Good enough performance for most tasks
- Industry standard for many applications

Understanding packages is fundamental to ROS 2 development. Every robot system you build will be organized into packages. Master this structure, and you'll be able to build, share, and maintain professional-quality robot software.

## Looking Ahead

Now that you can create packages and basic nodes, you're ready to explore more advanced topics:

In the next chapter, we'll dive into different message types, learn to create custom messages for your specific needs, and understand how to work with complex data structures in ROS 2.

You'll also learn about parameters, which allow you to configure your nodes without changing code, and launch files, which let you start complex multi-node systems with a single command.

---

## Review Questions

Test your understanding:

1. **What are the three required files in a Python ROS 2 package?** What does each file do?

2. **What is the difference between the `src/` and `install/` folders** in a workspace?

3. **Write the code to create a publisher** that publishes String messages to a topic called `/robot_status` every 2 seconds.

4. **What does `colcon build --packages-select my_package` do?** When would you use this instead of just `colcon build`?