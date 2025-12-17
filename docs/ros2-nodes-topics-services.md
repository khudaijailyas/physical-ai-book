# ROS 2 Nodes, Topics, Services & Actions

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain what nodes are and why ROS 2 uses them
- Understand the differences between topics, services, and actions
- Identify when to use each communication method in your robot system
- Create simple examples of nodes that communicate using topics, services, and actions
- Visualize how information flows through a ROS 2 system

## Concept Explanation

ROS 2 (Robot Operating System 2) is built around a modular design where different parts of your robot system work independently but communicate with each other. Understanding the four core concepts - nodes, topics, services, and actions - is essential for building any robot application.

### Nodes: The Building Blocks

A **node** is a single program that performs one specific task. Think of nodes as workers in a factory, where each worker has a specialized job. In a humanoid robot system, you might have:

- A node that reads data from the camera
- A node that processes images to detect objects
- A node that plans where the robot should move
- A node that sends commands to the motors

Each node runs independently as its own process. This means if one node crashes, the others can keep running. It also means you can test, debug, and update individual nodes without affecting the whole system.

### Topics: Broadcasting Information

**Topics** are like radio stations that continuously broadcast information. Any node can publish (broadcast) messages to a topic, and any node can subscribe (listen) to that topic. The key characteristics of topics are:

**One-way communication.** Publishers send messages but don't know who's listening or whether anyone is listening at all. Subscribers receive messages but don't send responses back through the same topic.

**Continuous streaming.** Topics are designed for data that flows regularly, like sensor readings. A camera node might publish images 30 times per second, and any node that needs those images can subscribe.

**Multiple publishers and subscribers.** Many nodes can publish to the same topic, and many nodes can subscribe to the same topic. For example, both your obstacle avoidance node and your object recognition node might subscribe to the camera topic.

**Asynchronous.** Subscribers receive messages whenever they arrive. There's no guarantee about timing or whether the subscriber is ready.

### Services: Request-Response Communication

**Services** are like asking someone a question and waiting for an answer. One node (the client) sends a request, and another node (the server) processes that request and sends back a response. The key characteristics of services are:

**Two-way communication.** Unlike topics, services involve a back-and-forth exchange. The client asks, the server answers.

**Synchronous.** When a client calls a service, it typically waits for the response before continuing. This is like calling someone on the phone - you wait for them to answer.

**One-to-one.** Each service request goes to exactly one server, and that server sends exactly one response back.

**Used for occasional tasks.** Services are perfect for things you do once in a while, like "calculate the path to this location" or "turn on the flashlight." They're not good for continuous data streams.

### Actions: Long-Running Tasks with Feedback

**Actions** are like services, but designed for tasks that take a long time and where you want progress updates. When you ask a humanoid robot to walk across the room, you don't want to wait in silence - you want to know it's making progress. The key characteristics of actions are:

**Three-way communication.** Actions involve a goal (what you want), feedback (progress updates), and a result (the final outcome).

**Interruptible.** You can cancel an action while it's running. If you tell your robot to walk somewhere but then realize it's going the wrong way, you can stop it.

**Asynchronous with feedback.** The client sends a goal and can do other things while waiting. Meanwhile, the server sends periodic feedback updates like "I'm 25% done" or "I've walked 2 meters so far."

**Used for tasks that take time.** Actions are perfect for movements, navigation, grasping objects, or any task where you need to know progress and might want to cancel.

### How They Work Together

In a real robot system, you use all three communication methods together. Here's how they complement each other:

Topics carry sensor data and state information that flows continuously. Your camera publishes images, your joint sensors publish positions, your IMU publishes orientation data.

Services handle quick computations or configuration changes. You might call a service to compute inverse kinematics, change a parameter, or query the robot's current mode.

Actions handle complex behaviors that take time. Walking, reaching for an object, or following a person are all actions because they have duration, provide feedback, and might need to be cancelled.

## Why This Matters

Understanding nodes, topics, services, and actions is fundamental to building effective robot systems because:

**Modularity enables reusability.** When you structure your code as separate nodes, you can reuse them across different projects. The camera node you write for one humanoid robot can work on another with minimal changes. You can even share nodes with the robotics community.

**The right communication method improves performance.** Using topics when you need services (or vice versa) leads to inefficient, unreliable systems. Topics are lightweight and fast for streaming data, but terrible for request-response patterns. Services are perfect for questions and answers, but wasteful for continuous data. Actions handle long-running tasks gracefully with cancellation and feedback, which topics and services can't do well.

**Debugging becomes manageable.** When your entire robot program is one giant file, finding bugs is nightmare. With nodes, you can test each component independently. You can record topic data and replay it later for debugging. You can check which nodes are running, what topics exist, and who's publishing or subscribing.

**Parallel processing improves responsiveness.** Since nodes run independently, your robot can do multiple things at once. While one node processes camera images (which is slow), another node can read joint positions and control motors. If you put everything in one program, the slow image processing would block everything else.

**You can build on existing work.** The ROS 2 ecosystem has thousands of pre-built nodes for common tasks. Want SLAM (mapping and localization)? There's a node for that. Need motion planning? There's a node for that. Understanding how nodes communicate lets you integrate these tools into your system.

## Example

Let's build a simple example that shows all three communication methods working together. Imagine a humanoid robot that can wave its hand when you ask it to. We'll create three nodes:

### Example 1: Hand Position Publisher (Topic)

This node continuously publishes the current position of the robot's hand:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HandPositionPublisher(Node):
    def __init__(self):
        super().__init__('hand_position_publisher')
        
        # Create a publisher for the hand position
        self.publisher = self.create_publisher(Point, 'hand_position', 10)
        
        # Publish the position 10 times per second
        self.timer = self.create_timer(0.1, self.publish_position)
        
        self.x = 0.3  # Hand starts 0.3 meters in front
        self.y = 0.2  # Hand starts 0.2 meters to the right
        self.z = 1.0  # Hand starts 1.0 meters high
        
        self.get_logger().info('Hand position publisher started')

    def publish_position(self):
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates a **topic** called `hand_position` and publishes the hand's current position 10 times per second. Any other node that wants to know where the hand is can subscribe to this topic.

### Example 2: Wave Service (Service)

This node provides a **service** that checks whether the robot is ready to wave:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class WaveService(Node):
    def __init__(self):
        super().__init__('wave_service')
        
        # Create a service called 'can_wave'
        self.service = self.create_service(
            Trigger,
            'can_wave',
            self.check_can_wave
        )
        
        self.battery_level = 85  # Percent
        self.is_busy = False
        
        self.get_logger().info('Wave service ready')

    def check_can_wave(self, request, response):
        # Check if conditions are right for waving
        if self.battery_level < 20:
            response.success = False
            response.message = 'Battery too low to wave'
        elif self.is_busy:
            response.success = False
            response.message = 'Robot is busy with another task'
        else:
            response.success = True
            response.message = 'Ready to wave'
        
        self.get_logger().info(f'Wave check: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaveService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates a **service** called `can_wave`. When another node calls this service, it checks the battery level and whether the robot is busy, then responds with whether it's safe to wave.

### Example 3: Wave Action (Action)

This node provides an **action** that actually performs the waving motion:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci  # Using standard action for example
import time

class WaveAction(Node):
    def __init__(self):
        super().__init__('wave_action')
        
        # Create an action server
        self.action_server = ActionServer(
            self,
            Fibonacci,  # In real code, you'd use a custom Wave action
            'perform_wave',
            self.execute_wave
        )
        
        self.get_logger().info('Wave action server ready')

    def execute_wave(self, goal_handle):
        self.get_logger().info('Starting wave action...')
        
        # Simulate the waving motion with 5 steps
        feedback_msg = Fibonacci.Feedback()
        
        for i in range(5):
            # Check if someone cancelled the action
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Wave cancelled')
                return Fibonacci.Result()
            
            # Simulate moving the hand
            # In real code, this would send commands to motors
            progress = (i + 1) * 20  # 20%, 40%, 60%, 80%, 100%
            feedback_msg.sequence = [progress]
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Wave progress: {progress}%')
            time.sleep(0.5)  # Each wave step takes 0.5 seconds
        
        # Wave complete
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [100]
        self.get_logger().info('Wave completed!')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = WaveAction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates an **action** called `perform_wave`. When another node sends a goal (request to wave), it simulates the waving motion over 2.5 seconds (5 steps of 0.5 seconds each), sending feedback updates along the way. The action can be cancelled at any time.

### How These Work Together

Now imagine a fourth node that coordinates everything:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from action_tutorials_interfaces.action import Fibonacci

class WaveCoordinator(Node):
    def __init__(self):
        super().__init__('wave_coordinator')
        
        # Subscribe to hand position topic
        self.subscription = self.create_subscription(
            Point,
            'hand_position',
            self.position_callback,
            10
        )
        
        # Create service client
        self.service_client = self.create_client(Trigger, 'can_wave')
        
        # Create action client
        self.action_client = ActionClient(self, Fibonacci, 'perform_wave')
        
        self.current_position = None
        self.get_logger().info('Wave coordinator ready')

    def position_callback(self, msg):
        # Continuously receive hand position from the topic
        self.current_position = msg

    def request_wave(self):
        # First, check if we can wave (using the service)
        request = Trigger.Request()
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if not response.success:
            self.get_logger().info(f'Cannot wave: {response.message}')
            return
        
        # We can wave! Start the wave action
        self.get_logger().info('Starting wave action')
        goal = Fibonacci.Goal()
        
        # Send the goal
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
    def feedback_callback(self, feedback_msg):
        # Receive progress updates from the action
        progress = feedback_msg.feedback.sequence[0]
        self.get_logger().info(f'Received feedback: {progress}% complete')

def main(args=None):
    rclpy.init(args=args)
    node = WaveCoordinator()
    
    # Wait a moment for connections to establish
    time.sleep(1)
    
    # Request a wave
    node.request_wave()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This coordinator node demonstrates all three communication methods:

- It **subscribes to the topic** `hand_position` to continuously monitor where the hand is
- It **calls the service** `can_wave` to check if conditions are right
- It **sends an action goal** to `perform_wave` to execute the waving motion, receiving feedback updates along the way

## Practical Notes

**Start with topics for most sensor data.** If you're reading from a camera, lidar, IMU, or joint encoders, publish that data on topics. Topics are the most efficient way to stream continuous data, and they allow multiple nodes to use the same sensor information.

**Use services for quick computations and queries.** If you need to compute something that takes less than a second and returns a single answer, use a service. Examples include inverse kinematics calculations, coordinate transformations, or querying configuration parameters. Services are simpler than actions and perfect for quick tasks.

**Choose actions for anything the user might want to cancel.** Walking, reaching, grasping, and navigation should all be actions. If a task takes more than a second or two, or if the user needs progress updates, use an action instead of a service.

**Keep nodes focused on one responsibility.** Don't create a "mega node" that does everything. Each node should have a clear, single purpose. This makes testing easier, reduces bugs, and allows nodes to be reused in different projects.

**Topic names should be descriptive.** Use clear names like `/left_camera/image` or `/right_arm/joint_states` rather than generic names like `/data` or `/output`. Good names make your system easier to understand and debug.

**Quality of Service (QoS) settings matter.** ROS 2 allows you to configure how topics handle message delivery. For sensor data that becomes stale quickly, use a small queue and allow dropping old messages. For critical commands, use reliable delivery. Start with default settings and adjust when you understand your system's needs.

**Use visualization tools during development.** Tools like `rqt_graph` show you which nodes are running and how they're connected. The `ros2 topic` command-line tools let you see what's being published. Use these tools to verify your system is wired correctly before debugging complex behaviors.

**Test nodes independently before integration.** Write each node so it can run on its own. Use command-line tools to manually publish test messages to your node's topics or call its services. This catches bugs early before they interact with other nodes.

**Handle communication failures gracefully.** Networks have delays, nodes crash, and messages get lost. Your subscriber should handle cases where messages stop arriving. Your service client should handle timeouts. Your action client should handle failed goals. Don't assume perfect communication.

**Consider message frequency and bandwidth.** Publishing high-resolution images at 100 Hz might overwhelm your network. Think about how often you really need updates. Can you reduce resolution? Publish less frequently? Use compression? Balance data needs with system resources.

## Summary

ROS 2 organizes robot systems into independent nodes that communicate through three main methods: topics, services, and actions. Nodes are individual programs that each perform one specific task, allowing you to build modular, testable, and reusable systems.

Topics provide one-way, continuous communication ideal for streaming sensor data. Publishers broadcast messages to topics without knowing who's listening, and subscribers receive those messages asynchronously. Use topics when data flows regularly and you don't need responses.

Services provide two-way, synchronous communication for request-response patterns. A client sends a request and waits for the server to send back a response. Use services for quick computations, queries, or commands that complete in under a second.

Actions provide two-way communication for long-running tasks with feedback and cancellation. A client sends a goal, the server provides periodic feedback updates, and eventually sends a result. Use actions for any task that takes time, where users need progress updates, or where cancellation might be needed.

Choosing the right communication method for each part of your system is crucial for building efficient, responsive robots. Topics keep sensor data flowing without blocking other operations. Services handle quick questions and answers efficiently. Actions make long-running behaviors interruptible and observable. By understanding these building blocks, you can design robot systems that are modular, maintainable, and performant.