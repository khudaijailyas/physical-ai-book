# Sensors & Actuators Setup

## Learning Objectives

- Configure and calibrate common sensors used in robotic systems including cameras, LiDAR, IMUs, and encoders
- Set up and tune actuators including DC motors, servo motors, and grippers for reliable operation
- Integrate sensors and actuators with ROS 2 for data streaming and control
- Troubleshoot common sensor and actuator problems in lab environments
- Apply proper wiring, mounting, and safety practices when installing robot hardware

## Concept Explanation

Setting up sensors and actuators transforms individual hardware components into a functional robotic system. This process involves physical installation, electrical connection, software configuration, calibration, and testing. Proper setup ensures reliable data collection and precise control, which are essential for all robot behaviors from simple teleoperation to complex autonomous tasks.

### Sensor Setup Fundamentals

Sensors require careful installation and configuration to provide accurate, reliable data to your robot's control systems.

**Physical Mounting Considerations**

Sensor placement directly affects data quality and robot performance. Cameras need unobstructed views of the environment with stable mounting to prevent image blur from vibration. LiDAR units require 360-degree clearance and level mounting for accurate distance measurements. IMUs must mount rigidly to the robot's main body, preferably near the center of mass, with known orientation relative to the robot's coordinate frame. Encoders attach directly to motor shafts or wheels with secure mechanical coupling. Force sensors install at contact points with proper load alignment. Every sensor mounting should consider vibration damping, thermal expansion, and cable strain relief.

**Electrical Connection and Power**

Different sensors have different power and signal requirements. Most modern sensors use standard interfaces like USB for cameras, serial UART for GPS modules, or I2C for small sensors. Verify voltage requirements carefully, as some sensors need 3.3 volts while others require 5 volts or 12 volts. Use appropriate power supplies or voltage regulators. Always connect ground first when wiring, and double-check polarity before applying power. Shielded cables reduce electromagnetic interference for analog sensors. Keep power cables separated from signal cables when possible to minimize noise.

**Software Driver Installation**

Most sensors require specific software drivers to communicate with your robot's computer. For ROS 2 systems, check if official ROS 2 packages exist for your sensor. Install drivers through the package manager or build from source following documentation. Verify the driver recognizes the sensor by checking device connections and running diagnostic tools. Configure parameters like frame rates, resolution, or sampling frequency in configuration files. Set appropriate coordinate frame names to match your robot's URDF model.

**Sensor Calibration Procedures**

Calibration compensates for manufacturing variations and environmental factors. Camera calibration determines intrinsic parameters like focal length and distortion coefficients using checkerboard patterns at various positions and orientations. IMU calibration corrects for bias and scale errors through stationary periods and known rotations. Magnetometer calibration requires rotating the sensor through all orientations to map magnetic field distortions. Encoder calibration establishes counts-per-revolution and direction. Store calibration parameters in configuration files that load automatically at startup.

**Data Validation and Quality Checks**

After setup, verify sensor output quality. For cameras, check image clarity, proper exposure, and consistent frame rates. Examine depth data for noise levels and range accuracy. Monitor IMU data for drift over time and noise characteristics. Verify encoder counts match actual wheel rotations. Use ROS 2 visualization tools like RViz to inspect sensor data graphically. Record data during test movements and analyze for anomalies or unexpected patterns.

### Actuator Setup Fundamentals

Actuators require proper configuration to deliver safe, precise, and repeatable motion control.

**Motor Driver Selection and Connection**

Motor drivers amplify control signals from your processor to power levels needed by motors. Match driver current rating to motor requirements with safety margin. Common driver types include H-bridge boards for DC motors, servo controller boards, and stepper motor drivers. Connect motor power lines to the driver's output terminals, observing polarity for DC motors. Connect control signals from your microcontroller or computer, typically PWM signals for speed control and direction pins. Provide adequate power supply with current capacity exceeding peak motor draw. Include fuse protection on power lines.

**Motor Parameter Configuration**

Set software parameters that match your motor characteristics. Configure PWM frequency appropriate for your motor type, typically 1 to 20 kilohertz for DC motors. Define maximum speed and acceleration limits to prevent mechanical stress and ensure safety. Set PID control gains for position or velocity control loops. For servo motors, configure pulse width ranges that correspond to minimum and maximum positions. Establish direction conventions so positive commands produce expected movement directions. Create software limits that prevent travel beyond mechanical constraints.

**Mechanical Coupling and Alignment**

Secure mechanical connection between motors and robot parts is essential for efficient power transmission and control accuracy. Shaft couplers must attach firmly without slippage using set screws or clamping mechanisms. Align motor shafts carefully with driven components to minimize bearing stress and power loss. For belt or chain drives, maintain proper tension without over-tightening. Gear systems require precise center distances and backlash management. Check all mechanical connections after initial operation and periodically thereafter as vibration can loosen fasteners.

**Control Loop Implementation**

Most actuators require closed-loop control for precise operation. Read encoder feedback or motor driver feedback at consistent rates. Implement PID controllers that compare desired setpoints with actual positions or velocities. Tune PID gains systematically, starting with proportional gain only, then adding derivative and integral terms as needed. Monitor control signals for saturation indicating insufficient motor power or unrealistic commands. Implement safety checks that detect excessive errors and stop motion when control fails.

**Gripper and End-Effector Setup**

Grippers and specialized end-effectors require additional consideration. Configure open and closed positions with appropriate grip force limits. Add force sensing when available to detect grasping success and prevent crushing objects. Implement timeout protection that releases grip if commanded to close but sensors indicate no object present. For pneumatic grippers, set air pressure regulators correctly and check for leaks. Test with objects of various sizes and materials to verify reliable operation.

### Integration with ROS 2

ROS 2 provides standardized ways to interface sensors and actuators across different hardware.

**Creating Hardware Interface Nodes**

Write or configure ROS 2 nodes that bridge between hardware and the ROS network. For sensors, nodes read data from hardware interfaces and publish to ROS topics with appropriate message types. Camera nodes publish Image and CameraInfo messages. LiDAR nodes publish LaserScan or PointCloud2 messages. IMU nodes publish Imu messages with orientation, angular velocity, and linear acceleration. For actuators, nodes subscribe to command topics and translate ROS messages into hardware control signals. Motor controller nodes subscribe to velocity or position commands and output appropriate PWM or serial commands.

**Topic Configuration and Remapping**

Organize ROS topics logically with clear naming conventions. Use namespaces to group related sensors on multi-robot systems. Configure topic names in launch files for easy reconfiguration. Remap topic names to match existing software when integrating new hardware. Set appropriate quality of service profiles for reliable or best-effort delivery based on application needs.

**Transform Tree Setup**

Define coordinate frame relationships between sensors, actuators, and robot base. Publish static transforms for fixed sensor positions using robot_state_publisher or static_transform_publisher. Ensure transform tree connects all sensor frames to the robot base frame. Verify transforms using RViz visualization showing all frames properly positioned. Accurate transforms are essential for sensor fusion and spatial reasoning.

**Parameter Management**

Store hardware configuration parameters in YAML files separate from code. Include sensor calibration data, control gains, hardware limits, and communication settings. Use ROS 2 parameter system to load values at node startup. Implement dynamic reconfigure for parameters that may need runtime adjustment like camera exposure or PID gains. Document all parameters with units and valid ranges.

**Launch File Organization**

Create launch files that start all necessary nodes for a complete hardware setup. Include hardware driver nodes, coordinate frame publishers, and any processing nodes. Set parameters from configuration files. Configure logging levels appropriately. Use launch file arguments to enable optional components or switch between different hardware configurations. Test launch files thoroughly to ensure reliable startup.

### Testing and Validation Procedures

Systematic testing ensures your sensor and actuator setup functions correctly and safely.

**Incremental Testing Approach**

Test components individually before integrating into complete systems. Verify sensor data quality with simple test programs before connecting to complex algorithms. Test actuators at reduced speeds and ranges before full operation. Confirm ROS message publication rates and formats. Gradually increase complexity, adding one component or behavior at a time while monitoring for issues.

**Safety Testing**

Conduct dedicated safety tests for actuators. Verify emergency stop functionality halts all motion immediately. Test software limit switches prevent motion beyond safe ranges. Confirm collision detection stops motion when unexpected resistance occurs. Check that communication loss triggers safe shutdown procedures. Observe behavior during power loss or low battery conditions.

**Performance Characterization**

Measure actual performance against specifications. Record sensor update rates under various conditions. Measure actuator speed, acceleration, and position accuracy. Characterize latency from command to actual motion. Test repeatability by commanding identical motions multiple times. Document performance data for future troubleshooting and system design.

**Long-Duration Testing**

Run extended tests to identify issues that appear only after prolonged operation. Monitor for sensor drift, thermal issues, mechanical wear, or software memory leaks. Check battery consumption rates. Verify consistent performance as temperatures change. Long-duration tests reveal problems that short tests miss.

## Why This Matters

Proper sensor and actuator setup is the foundation of reliable robotic systems, directly impacting safety, performance, and learning outcomes in educational environments.

**Safety Cannot Be Compromised**

Incorrectly configured actuators pose serious safety risks. Motors without proper limits can cause collisions, crush objects, or injure people. Miscalibrated sensors lead to erroneous perceptions causing dangerous autonomous behaviors. Students working with improperly set up hardware may develop unsafe habits that carry into professional work. Taking time to set up hardware correctly instills safety-conscious thinking and practices that protect people and equipment throughout careers in robotics.

**Data Quality Determines Algorithm Success**

Modern robotics relies heavily on sensor data for perception, mapping, localization, and decision-making. Poor sensor setup produces noisy, inaccurate, or inconsistent data that causes algorithm failures regardless of how well the algorithms are designed. Students may incorrectly blame their code when hardware setup is actually the problem, leading to frustration and lost learning time. Proper setup ensures students can focus on algorithm development knowing their data is reliable.

**Control Precision Enables Advanced Behaviors**

Poorly tuned actuators exhibit jerky motion, position errors, and unreliable responses that make complex behaviors impossible. Path following, manipulation, and interaction tasks require precise, predictable control. Students cannot learn advanced control concepts when basic actuator setup is inadequate. Well-configured actuators let students explore sophisticated behaviors and appreciate the relationship between low-level control and high-level planning.

**Industry Standards and Professional Practice**

Professional robotics requires systematic hardware setup following established procedures and documentation practices. Students who learn proper setup methods, calibration procedures, and testing protocols develop professional competencies. Understanding how to read sensor datasheets, interpret actuator specifications, and configure hardware interfaces prepares students for industry positions where these skills are expected from day one.

**Troubleshooting Skills Development**

Hardware setup inevitably involves problems and debugging. Working through sensor connection issues, motor direction problems, or calibration challenges develops critical troubleshooting skills. Students learn to isolate problems, consult documentation, measure signals, and systematically test hypotheses. These problem-solving abilities transfer to all aspects of robotics and engineering work.

**Foundation for Physical AI Systems**

Vision-language-action models, humanoid control systems, and conversational robots all depend on reliable sensor inputs and precise actuator control. Advanced AI algorithms cannot overcome poor hardware setup. As robotics incorporates more AI and learning-based methods, the quality of physical interfaces becomes even more critical. Students building physical AI systems must master hardware setup to achieve meaningful results.

## Example

Consider setting up a mobile manipulator robot in an educational lab, integrating multiple sensors and actuators into a cohesive ROS 2 system for navigation and object manipulation tasks.

**System Overview**

The robot platform consists of a differential-drive mobile base with a 3-degree-of-freedom arm mounted on top. Students will use this platform to learn navigation, perception, and manipulation by programming tasks like navigating to objects, picking them up, and delivering them to goal locations.

**Mobile Base Sensor Setup**

Begin with the base platform's sensors. Mount an RPLidar A1 on the top center of the base, ensuring 360-degree clearance with at least 5 centimeters of space around the rotating scanner. Connect the LiDAR's USB cable to the onboard computer using a secure strain-relief clip to prevent disconnection from vibration. Install the ROS 2 rplidar driver package and create a configuration file setting the serial port, frame rate to 10 hertz, and coordinate frame name to "laser". Launch the node and verify scan data appears in RViz as expected points showing the lab environment.

Mount a RealSense D435 camera on the front of the robot at approximately 40 centimeters height, tilted downward 15 degrees to view the floor ahead while also seeing objects at table height. Secure the camera with vibration-damping material. Connect via USB 3.0 for adequate bandwidth. Install the realsense-ros package and configure it to publish both color images at 640x480 resolution and depth images at 30 frames per second. Calibrate the camera using the built-in calibration tools, saving intrinsic parameters. Set the camera frame name to "camera_link" and publish the transform from base_link to camera_link with the measured offset and rotation.

Install a 9-axis IMU on the top plate of the base, oriented with clearly marked x-axis pointing forward. Connect via I2C interface to an Arduino that bridges to the main computer via USB. Create a ROS 2 node that reads IMU data and publishes Imu messages at 100 hertz. Perform IMU calibration by placing the robot stationary for bias estimation, then following a figure-eight pattern for magnetometer calibration. Store calibration coefficients in a YAML file that loads automatically.

**Mobile Base Actuator Setup**

The base uses two DC motors with hall-effect encoders for the left and right wheels. Connect motor power wires to an H-bridge motor driver board, verifying that the motor voltage matches the driver's rating of 12 volts. Connect encoder signal wires to an Arduino's interrupt pins for counting. Wire direction control pins from Arduino to motor driver inputs. Connect PWM pins from Arduino for speed control.

Create an Arduino sketch that receives velocity commands over serial and outputs PWM signals proportional to commanded velocities. Implement a simple low-pass filter to smooth velocity commands and prevent jerky motion. Read encoder counts and publish odometry data back to the main computer. Set a PWM frequency of 10 kilohertz suitable for the DC motors.

Develop a ROS 2 node on the main computer that subscribes to cmd_vel Twist messages and sends corresponding velocity commands to the Arduino. The node also receives encoder data and publishes odometry on the odom topic. Configure the differential drive kinematics with measured wheel radius of 6.5 centimeters and wheelbase of 30 centimeters.

Test motors individually at low speed to verify direction matches expectations. Command forward velocity and confirm both wheels rotate forward. Command rotation and verify opposite wheel directions. Gradually increase speed while monitoring current draw to ensure motors operate within rated limits. Tune acceleration limits to 0.5 meters per second squared for smooth motion without wheel slip.

**Manipulator Arm Sensor and Actuator Setup**

The 3-degree-of-freedom arm uses three servo motors for shoulder, elbow, and gripper actuation. Connect servos to a PWM servo driver board that interfaces via I2C to the main computer. Each servo requires configuration of minimum and maximum pulse widths corresponding to joint limits. For the shoulder joint, set minimum pulse width to 1000 microseconds for -90 degrees and maximum to 2000 microseconds for +90 degrees.

Measure actual joint angles at several commanded positions using a protractor or angle gauge. Create a calibration mapping between commanded pulse widths and actual angles to compensate for servo non-linearity. Store this mapping in a lookup table or polynomial fit coefficients.

Install force-sensitive resistors on the gripper fingers to detect contact with objects. Connect these sensors to analog input pins on an Arduino. Calibrate sensors by measuring resistance values when gripping objects of known weights. Implement thresholding logic that publishes a grasp detection signal when both finger sensors exceed contact thresholds.

Create a ROS 2 control node that provides a JointTrajectoryController interface for the arm. This node subscribes to trajectory commands and interpolates between waypoints, sending position commands to servos at 50 hertz. Implement velocity and acceleration limits for each joint to ensure smooth motion. Add collision checking that stops motion if finger sensors detect unexpected contact during movement.

**System Integration**

Create a master launch file that starts all hardware nodes with proper parameters. Include the LiDAR node, camera node, IMU node, base controller node, and arm controller node. Set up all static transforms defining sensor positions relative to the base frame. Configure logging to record all sensor data and commands for later analysis.

Define coordinate frames following ROS conventions: base_link as the robot's primary frame, odom for odometry-based localization, laser for the LiDAR, camera_link for the camera, and joint frames for the arm. Publish these relationships through robot_state_publisher using a URDF model of the robot.

Test the integrated system with a simple autonomous task. Command the robot to drive forward one meter using odometry feedback while monitoring LiDAR for obstacles. If an obstacle appears closer than 50 centimeters, stop motion. Once at the goal position, command the arm to reach forward and close the gripper. This test validates that all sensors provide data, actuators respond to commands, and coordination works as expected.

**Student Workflow**

Students receive documentation describing the hardware setup and ROS 2 interface. They start by examining active topics using ros2 topic list and visualizing all data streams in RViz. Initial exercises have them record sensor data while manually driving the robot to understand data characteristics. They then write simple control programs commanding specific motions and observing results. As skills develop, students implement navigation algorithms using the LiDAR, vision-based object detection using the camera, and manipulation sequences using the arm. Throughout, they gain appreciation for how hardware setup enables or constrains what algorithms can achieve.

## Practical Notes

**Wiring and Connection Best Practices**

Maintain organized wiring from the start to prevent frustration and failures. Use consistent color coding for power, ground, and signal wires. Red for positive power, black for ground, and other colors for signals helps quickly identify connections. Label both ends of every cable with clear tags indicating source and destination. Use cable ties or cable management channels to route wires neatly and prevent snagging. Leave some slack in cables to accommodate robot motion without strain. Create wiring diagrams documenting every connection before starting physical work. Take photos of successful wiring configurations for reference when building additional robots.

**Common Sensor Issues and Solutions**

USB cameras not detected often result from insufficient power or bad cables. Try different USB ports or use powered hubs. LiDAR scanning incorrectly usually means the device is not level or has obstructed view. Check mechanical mounting and ensure nothing blocks the laser path. IMU drift happens with poor calibration or temperature changes. Recalibrate regularly and consider temperature compensation for precision applications. Encoders reporting erratic counts typically indicate loose mechanical connections or electrical noise. Tighten couplings and add filtering capacitors or twisted-pair wiring.

**Motor and Actuator Troubleshooting**

Motors not responding often means incorrect wiring, insufficient power supply, or damaged motor drivers. Use a multimeter to verify voltage at each connection point. Check that ground connections are solid. Motors moving in wrong directions require swapping motor wire polarity or inverting direction signals in software. Jerky or oscillating motion indicates poor PID tuning, too-high gain values, or mechanical binding. Reduce gains and check for mechanical resistance. Overheating motors suggest excessive load, insufficient cooling, or continuous operation beyond rated duty cycle. Reduce loads, add cooling, or implement rest periods.

**Calibration Maintenance Schedule**

Sensor calibration drifts over time requiring periodic recalibration. Establish a maintenance schedule based on usage intensity and accuracy requirements. IMUs may need weekly recalibration in active labs. Camera calibration remains stable for months unless the lens is adjusted. Encoder calibration should be verified whenever mechanical components are disassembled and reassembled. Keep calibration records noting date, procedure, and resulting parameters to track trends and identify deteriorating sensors.

**Cost-Effective Hardware Choices**

Budget-conscious labs can make strategic component choices without sacrificing learning objectives. USB webcams replace expensive depth cameras for basic vision learning at one-tenth the cost. Low-cost servo motors from hobby suppliers work well for light-duty manipulators. Arduino boards cost less than specialized motor controllers while teaching embedded programming. 2D LiDAR units provide navigation capabilities more affordably than 3D sensors. Prioritize spending on reliable motor drivers and power systems as these affect safety and all subsequent work.

**Scaling to Multiple Robots**

When building multiple robot platforms, standardization dramatically reduces workload. Design a reference hardware configuration with complete documentation including part numbers, wiring diagrams, and software configuration. Create detailed build guides with photos at each step. Develop automated software installation scripts that configure a fresh computer to complete working state. Test the entire build process by having someone unfamiliar with the system follow your documentation. Maintain common spare parts inventory so any failed component can be quickly replaced across all robots.

**Documentation Practices**

Create comprehensive documentation for every hardware setup. Include component specifications with links to datasheets. Provide wiring diagrams using tools like Fritzing for clarity. Document all configuration files with comments explaining each parameter. Write setup procedures as step-by-step checklists. Record calibration procedures and expected results. Maintain troubleshooting guides covering common problems and solutions. Store documentation in version control alongside software code. Good documentation makes hardware setup repeatable and teachable, dramatically reducing setup time for new students or teaching assistants.

**Safety Equipment and Procedures**

Equip your lab with appropriate safety tools for hardware work. Provide safety glasses for all mechanical work. Keep fire extinguishers rated for electrical fires near battery charging areas. Use fused power connections to protect against shorts. Install emergency stop buttons that cut power to all actuators and are clearly visible and easily reachable. Establish clear zones for robot operation separated from workbenches. Create safety checklists that must be completed before powering on robots. Train all students on emergency procedures before allowing hands-on work. Review and update safety protocols as new equipment is added.

**Dealing with Obsolete or Discontinued Hardware**

Sensor and actuator models frequently change or become discontinued. Plan for this by abstracting hardware interfaces in software. Use ROS 2 standard message types so replacing a sensor only requires updating the driver node, not all dependent code. Document the specifications and capabilities you need rather than specific model numbers. When purchasing multiple units, buy spares for critical components. Join ROS community discussions to learn which hardware has strong ongoing support. Consider open-source hardware with published designs that can be manufactured even if commercial products disappear.

## Summary

Setting up sensors and actuators transforms hardware components into functional robotic systems. The process involves careful physical mounting considering vibration, field of view, and mechanical constraints, electrical connections verifying voltage compatibility and using proper wiring practices, software driver installation and configuration for ROS 2 integration, systematic calibration to compensate for manufacturing variations and environmental factors, and thorough testing to validate data quality and control performance. Common sensors like cameras, LiDAR, IMUs, and encoders each require specific setup procedures and calibration methods. Actuators including DC motors, servos, and grippers need proper driver selection, parameter configuration, mechanical coupling, and control loop tuning. Integration with ROS 2 involves creating hardware interface nodes, configuring topics and transforms, managing parameters, and organizing launch files. Proper setup ensures safety by implementing limits and emergency stops, enables reliable algorithm development by providing quality data and precise control, develops professional troubleshooting skills, and forms the foundation for advanced applications in physical AI and robotics. Practical considerations include organized wiring practices, regular calibration maintenance, strategic budget allocation, documentation standards, and safety protocols that protect people and equipment throughout the development process.