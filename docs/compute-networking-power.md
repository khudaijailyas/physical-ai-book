# Compute, Networking & Power

## Learning Objectives

- Design and configure computing architectures for robotic systems including single-board computers and microcontrollers
- Set up reliable networking infrastructure for robot communication, remote operation, and multi-robot coordination
- Implement safe and efficient power systems including battery selection, power distribution, and charging protocols
- Troubleshoot common compute, networking, and power issues in educational robotics labs
- Apply best practices for system integration ensuring all subsystems work together reliably

## Concept Explanation

Compute, networking, and power form the essential infrastructure that supports all robot operations. These systems must work together seamlessly to enable sensors, actuators, and algorithms to function reliably. Understanding how to design, configure, and maintain these foundational systems is critical for building robust robotic platforms.

### Computing Architecture for Robotics

Robotic systems require computing hardware that balances processing power, energy efficiency, real-time performance, and cost. Most educational and research robots use a layered computing architecture combining different types of processors.

**Single-Board Computers**

Single-board computers like Raspberry Pi, NVIDIA Jetson, and Intel NUC boards serve as the primary computing platform for modern robots. These boards run full operating systems like Ubuntu Linux and support ROS 2, making them ideal for high-level processing tasks. They handle sensor data processing, run navigation algorithms, execute AI models, manage communication, and coordinate overall robot behavior.

The Raspberry Pi 4 or 5 offers good general-purpose computing at low cost, suitable for basic robotics applications with moderate sensor loads. NVIDIA Jetson boards including the Nano, Orin Nano, and Orin provide GPU acceleration essential for computer vision, deep learning inference, and point cloud processing. These boards excel at running neural networks for object detection, pose estimation, and vision-language models. Intel NUC computers and similar x86 platforms provide maximum computational power for demanding applications but consume more power and cost more than ARM-based alternatives.

**Microcontrollers**

Microcontrollers handle real-time control tasks that require deterministic timing. Arduino boards, ESP32 modules, Teensy boards, and STM32 development boards are common choices. Microcontrollers excel at reading encoder values at high frequency, generating precise PWM signals for motor control, interfacing with analog sensors, handling emergency stop interrupts, and managing low-level hardware timing.

The division between single-board computers and microcontrollers creates a two-tier architecture. The single-board computer handles high-level planning and perception while the microcontroller ensures reliable low-level control. This separation improves system robustness since real-time motor control continues even if high-level processes are busy with computationally intensive tasks.

**Communication Between Computing Layers**

The main computer and microcontroller must communicate efficiently. Serial communication over USB provides a simple, reliable connection. The microcontroller runs firmware that receives commands and sends sensor data using defined protocols. Common approaches include ASCII-based command strings for simplicity and human readability, or binary protocols for efficiency with larger data volumes. The ROS 2 node on the main computer handles protocol translation, converting ROS messages to microcontroller commands and incoming serial data to ROS messages.

**Computational Resource Management**

Computing resources must be allocated carefully to prevent overload. Monitor CPU usage to ensure critical control loops have sufficient processing time. Track memory consumption to prevent system slowdowns from excessive swap usage. Manage disk space, especially when logging sensor data or recording bags. Configure process priorities so time-critical tasks preempt less important ones. Use tools like htop, iostat, and ROS 2 performance monitoring to identify bottlenecks and optimize resource usage.

**Operating System Configuration**

Proper operating system configuration improves reliability and performance. Use real-time kernel patches if deterministic timing is critical. Disable unnecessary services that consume resources without benefit. Configure automatic startup of robot software using systemd services. Set up logging rotation to prevent disk overflow. Implement watchdog timers that restart failed processes automatically. Configure network interfaces with static IP addresses for predictable connectivity.

### Networking Infrastructure

Networking enables robot communication with operators, other robots, remote computers, and cloud services. Reliable networking is essential for development, operation, and multi-robot systems.

**Network Architecture Options**

Several network architectures suit different robotics applications. Direct WiFi connection between robot and operator laptop provides simplicity for single-robot development. The robot creates a WiFi access point or connects to an existing network, and the operator's computer joins the same network. This works well for lab environments where robots operate within WiFi range.

Dedicated robot network using a local router creates isolated infrastructure for robot systems. All robots and operator stations connect to this network, avoiding interference from other devices. This approach offers better reliability, security, and performance than shared networks. Configure the router with static DHCP assignments so each robot maintains the same IP address across reboots.

Ethernet connections provide maximum bandwidth and reliability for stationary robots or charging stations. Use power-over-ethernet when available to reduce cabling. For mobile robots, Ethernet docking stations enable fast data transfer when robots return to base.

Multiple network interfaces allow robots to maintain simultaneous connections. One WiFi interface connects to the operator network while another creates a robot-to-robot mesh network. Wired Ethernet handles high-bandwidth data transfer while WiFi provides mobility. Configure routing tables to direct traffic appropriately across interfaces.

**ROS 2 Network Configuration**

ROS 2 uses DDS (Data Distribution Service) for communication, which requires proper network configuration. DDS automatically discovers nodes across the network but needs correctly configured multicast routing. Ensure the ROS_DOMAIN_ID environment variable is set consistently across all computers in your system. Different robot systems should use different domain IDs to prevent interference.

For multi-robot systems, decide whether to use a single ROS domain with namespaces for each robot or separate domains per robot. Single domain simplifies some coordination tasks but increases network traffic. Separate domains isolate traffic but require bridging for cross-robot communication.

Configure DDS quality-of-service settings appropriately for different data types. Sensor data often uses best-effort delivery for low latency, while control commands may require reliable delivery. Adjust history depth and reliability settings based on your application's needs.

**Remote Access and Development**

Remote access capabilities streamline development and operation. SSH provides secure command-line access to robot computers for configuration, software deployment, and troubleshooting. Configure SSH key-based authentication to avoid password entry. Set up port forwarding to access ROS 2 topics and services from remote computers.

VNC or remote desktop software enables graphical interface access when needed for debugging or demonstration. NoMachine, TightVNC, or X11 forwarding over SSH provide options with different performance characteristics.

Web-based interfaces using tools like Webviz or Foxglove Studio allow robot monitoring and control through web browsers. These tools work well for demonstrations and provide cross-platform access without special software installation.

**Network Security Considerations**

Security protects robots from unauthorized access and network attacks. Change default passwords on all devices including routers, robot computers, and any embedded systems. Use WPA3 or at minimum WPA2 encryption for WiFi networks. Disable unused network services to reduce attack surface. Configure firewalls to allow only necessary ports. For production deployments, implement VPN access rather than exposing robots directly to the internet. Regular software updates patch security vulnerabilities.

**Bandwidth Management**

Network bandwidth limitations affect system design. Cameras generating compressed images at 30 frames per second consume approximately 5 to 15 megabits per second depending on resolution and compression. Uncompressed depth data from 3D cameras requires significantly more bandwidth. LiDAR point clouds can generate 10 to 50 megabits per second depending on scan density and rate.

Monitor network usage to identify bandwidth bottlenecks. Reduce image resolution or frame rate when full quality is unnecessary. Use efficient compression for camera images. Implement data decimation for point clouds, transmitting only every nth point when appropriate. Consider edge processing that computes features on the robot and transmits compact representations rather than raw sensor data.

### Power Systems

Power systems supply energy to all robot components. Reliable power delivery with appropriate safety measures is critical for system operation and longevity.

**Battery Technology Selection**

Different battery technologies offer distinct trade-offs. Lithium-ion and lithium-polymer batteries provide high energy density, low self-discharge, and good cycle life, making them the standard choice for mobile robots. They deliver stable voltage under load and support high discharge rates needed for motors. However, they require careful charging and protection circuitry to prevent fire hazards.

Lead-acid batteries cost less and handle abuse better but offer lower energy density and heavier weight. They suit stationary robots or teaching environments where low cost matters more than weight. Nickel-metal hydride batteries provide moderate energy density with better safety characteristics than lithium but have mostly been superseded by lithium technologies in robotics.

Select battery capacity based on power consumption and desired operating time. Calculate total power consumption by summing all component power draws. Add 20 to 30 percent margin for losses and aging. Divide total energy needed by battery voltage to determine required amp-hour capacity. For example, a robot consuming 20 watts average power for 2 hours needs 40 watt-hours or approximately 3.3 amp-hours at 12 volts.

**Battery Management Systems**

Battery management systems (BMS) protect batteries and extend their lifespan. A good BMS monitors individual cell voltages to prevent overcharge and over-discharge. It balances cells by redistributing charge among cells to maintain equal voltages. Temperature monitoring shuts down charging if batteries become too hot. Current limiting prevents damage from excessive discharge rates. Short circuit protection immediately disconnects loads if a fault occurs.

Many lithium battery packs include integrated BMS, but verify functionality and specifications. For custom battery packs, select appropriate BMS boards rated for your battery chemistry, voltage, and current. Configure undervoltage and overvoltage thresholds according to your battery specifications. Test protection features in safe conditions before relying on them.

**Power Distribution Architecture**

Efficient power distribution delivers appropriate voltages to different components. Most robots require multiple voltage levels such as 12 volts for motors, 5 volts for logic circuits and sensors, and 3.3 volts for some microcontrollers and sensors.

Use DC-DC converters to efficiently step battery voltage to required levels. Buck converters step voltage down with high efficiency, typically 85 to 95 percent. Select converters with current ratings exceeding peak load by at least 20 percent. Add capacitors at converter outputs to reduce voltage ripple and handle transient loads.

Create separate power rails for different component types. Motors connect to dedicated power rails with appropriate filtering since they generate electrical noise. Sensitive electronics like cameras and computers use separate clean power rails. This isolation prevents motor noise from disrupting sensor readings or computer operation.

Implement a power distribution board that centralizes connections and includes fuses or circuit breakers for each subsystem. Fuses protect against short circuits and overloads by breaking the circuit before damage occurs. Select fuse ratings slightly above normal operating current but below wire and component ratings.

**Power Switching and Control**

Safe power systems include proper switching and emergency shutdown capabilities. The main power switch should be easily accessible and clearly marked. Mechanical switches rated for battery voltage and total system current provide reliable primary control. Illuminated switches indicate power state clearly.

Emergency stop buttons must kill power to all actuators immediately. Wire emergency stops to interrupt motor power directly, not through software control. This ensures motion stops even if computers have crashed. Implement emergency stop in both hardware and software layers for redundancy.

Software-controlled power switching using MOSFETs or relay modules allows automated power management. The main computer can switch power to subsystems based on operational state, disabling unused peripherals to conserve energy. Smart charging systems automatically manage charging when robots dock at charging stations.

**Charging Infrastructure**

Proper charging infrastructure maintains battery health and ensures robot availability. Use chargers specifically designed for your battery chemistry with appropriate charge profiles. Lithium batteries require constant-current constant-voltage charging with precise voltage regulation. Lead-acid batteries benefit from multi-stage charging with desulfation capabilities.

Designate safe charging areas with fire-resistant surfaces and good ventilation. Never charge damaged or swollen batteries. Store fully charged spare batteries when possible to minimize robot downtime. For labs with multiple robots, stagger charging schedules to avoid overloading electrical circuits.

Implement charge monitoring that logs battery charge cycles, tracks capacity degradation over time, and alerts when batteries need replacement. Battery capacity degrades gradually, typically retaining 80 percent of original capacity after 300 to 500 cycles depending on chemistry and usage patterns.

**Power Monitoring and Telemetry**

Monitor power system health continuously during operation. Voltage sensing circuits measure battery voltage and report to software. Current sensors measure instantaneous power consumption and total energy used. Publish power telemetry as ROS topics for logging and real-time monitoring.

Implement low-battery warnings that alert operators when charge falls below safe thresholds. Trigger automatic behaviors like returning to charging station when battery reaches critical levels. Log power data during operation to characterize actual power consumption and identify opportunities for optimization.

**Safety and Fire Prevention**

Lithium batteries pose fire risks if damaged, overcharged, or short-circuited. Store batteries in fire-resistant containers like LiPo bags or metal cases. Never leave charging batteries unattended. Install smoke detectors near charging areas. Keep fire extinguishers rated for electrical fires readily accessible.

Inspect batteries regularly for physical damage, swelling, or unusual heating. Replace damaged batteries immediately, disposing of them according to local regulations. Never puncture or disassemble lithium batteries. For educational environments, consider using safer lithium iron phosphate (LiFePO4) chemistry which has better thermal stability than standard lithium-ion despite lower energy density.

## Why This Matters

Compute, networking, and power infrastructure determines whether your robot functions reliably or experiences constant failures and frustrations. These foundational systems enable everything else to work.

**System Reliability Depends on Infrastructure**

Algorithms and hardware are only useful if the infrastructure supporting them functions reliably. Insufficient computing power causes algorithms to run slowly or fail. Poor networking creates communication dropouts that interrupt control or data collection. Inadequate power systems lead to unexpected shutdowns, data corruption, or hardware damage. Students working with unreliable infrastructure waste time troubleshooting infrastructure problems rather than learning robotics concepts. Investment in proper infrastructure pays dividends through reduced frustration and increased learning time.

**Safety Requires Proper Power Management**

Power systems pose the most significant safety risks in robotics labs. Incorrectly wired batteries can create fire hazards. Lack of emergency stop capability allows runaway robots to cause damage or injury. Missing overcurrent protection risks burning wires or components. Students must learn proper power system design not just for their educational projects but to develop safe practices for professional work. Labs that take shortcuts on power safety teach students dangerous habits.

**Real-Time Performance Needs Appropriate Computing**

Many robotics applications require real-time or near-real-time performance. Motor control loops must execute at consistent rates without jitter. Sensor fusion algorithms need deterministic timing. Emergency stop responses must have bounded latency. Understanding computing architecture and how to achieve real-time performance is essential for safety-critical applications like autonomous vehicles or collaborative robots. Students who only experience systems without timing constraints lack preparation for real-world robotics challenges.

**Modern Robotics is Networked**

Contemporary robotics increasingly involves distributed systems, cloud connectivity, and multi-robot coordination. Manufacturing facilities deploy fleets of coordinated robots. Autonomous vehicles communicate with infrastructure and other vehicles. Research platforms offload computation to remote servers for intensive processing. Students must understand networking fundamentals, distributed systems concepts, and how to design reliable networked robot systems. Single isolated robots no longer represent realistic applications.

**Energy Efficiency Enables Autonomy**

Mobile robots are fundamentally limited by battery capacity. Understanding power consumption, efficiency optimization, and energy management extends operating time and enables longer autonomous missions. This matters for applications from warehouse robots that must work full shifts to planetary rovers that operate on limited solar power. Students who learn to measure, model, and optimize power consumption develop critical skills for mobile robotics.

**Infrastructure Design is Systems Engineering**

Designing compute, networking, and power systems requires thinking about the entire robot as an integrated system. Component choices interact and constrain each other. A powerful GPU requires more electrical power and cooling. High-bandwidth sensors need faster networking. Safety requirements dictate power switching architecture. This systems engineering perspective where understanding interactions and trade-offs matters as much as individual component knowledge prepares students for real engineering work.

**Professional Competency and Career Preparation**

Industry expects robotics engineers to understand computing platforms, network protocols, power electronics, and system integration. Projects fail due to inadequate infrastructure as often as due to algorithm problems. Employers value engineers who can specify appropriate computing hardware for applications, debug network configuration issues, design safe power systems, and integrate subsystems into working robots. Educational programs that teach infrastructure alongside algorithms produce more capable graduates.

## Example

Consider designing and implementing the complete compute, networking, and power infrastructure for an autonomous mobile robot used in a university robotics course teaching navigation, perception, and multi-robot coordination.

**Project Requirements**

The robot must navigate autonomously using LiDAR and camera sensors, run object detection neural networks for identifying obstacles and targets, operate for at least 90 minutes on battery power, communicate wirelessly with operator laptops and other robots, and support safe emergency shutdown. The budget allows approximately 800 dollars for computing, networking, and power infrastructure.

**Computing Architecture Design**

Select an NVIDIA Jetson Orin Nano as the primary computer. This board provides 40 TOPS of AI performance sufficient for running YOLO object detection networks at 20 frames per second while simultaneously handling navigation algorithms. The board includes 8GB RAM adequate for ROS 2 and typical application loads. Cost is approximately 400 dollars. Install Ubuntu 22.04 with ROS 2 Humble following NVIDIA's setup documentation.

Add an Arduino Mega 2560 microcontroller for low-level motor control. This provides 54 digital I/O pins sufficient for controlling two motor drivers, reading encoder inputs, monitoring battery voltage, and handling emergency stop signals. The Arduino connects to the Jetson via USB and costs about 15 dollars. Upload firmware that implements a simple serial protocol receiving velocity commands and responding with encoder counts and battery voltage.

Configure the Jetson to automatically start a systemd service at boot that launches the main ROS 2 software stack. Create a launch file that starts the motor controller node, sensor drivers, navigation stack, and perception nodes. Set CPU governor to performance mode for consistent timing. Disable unnecessary services like graphical desktop to reduce resource consumption and boot time. Install all software on the 64GB eMMC storage included with the Orin Nano.

**Networking Implementation**

Equip the Jetson with a USB WiFi adapter supporting dual-band operation. Configure the built-in WiFi for connection to the lab's robot network and the USB adapter for creating an access point for direct operator connection when needed. This dual-interface setup provides redundancy and flexibility.

Create a dedicated robot network using a commercial dual-band WiFi router with gigabit Ethernet ports. Configure the 5GHz band for robot operation with WPA2 security. Assign static IP addresses to all robots using DHCP reservations based on MAC addresses. Configure the router's DHCP server to assign addresses in the 192.168.10.0/24 range with robots using 192.168.10.11 through 192.168.10.20 and operator workstations using 192.168.10.101 and above.

Set ROS_DOMAIN_ID to 10 for this robot fleet to isolate traffic from other ROS systems in the building. Configure DDS to use UDP multicast on the local network. Test discovery by running ros2 node list on an operator laptop and verifying nodes on the robot appear. Measure network latency using ping and verify latency stays below 10 milliseconds under typical conditions.

Install SSH server on the Jetson and configure key-based authentication for student accounts. Create a shared SSH key for the course so students can access any robot without password entry. Configure SSH to allow port forwarding so students can visualize ROS topics on their laptops by forwarding the necessary ports.

Set up Foxglove Studio for web-based robot monitoring. Install the Foxglove bridge on the robot that makes ROS topics accessible via WebSocket. Students can connect their web browsers to the robot's IP address and immediately see camera feeds, LiDAR data, and robot status without installing specialized software.

**Power System Design**

Select a 14.8V (4S) lithium-ion battery pack with 5000mAh capacity. This provides approximately 74 watt-hours of energy. The pack includes an integrated BMS with overcharge, overdischarge, and short circuit protection. Cost is about 60 dollars. This battery voltage works well for 12V motors while providing adequate headroom for 5V regulation.

Calculate expected power consumption: Jetson Orin Nano at 15 watts maximum, two DC motors at 10 watts each during normal operation, LiDAR at 3 watts, camera at 2.5 watts, Arduino and peripherals at 1.5 watts, totaling approximately 42 watts peak or about 30 watts average. The 74 watt-hour battery provides approximately 2.5 hours of operation at average load, exceeding the 90-minute requirement with comfortable margin.

Design power distribution using a custom PCB that incorporates fused connections for each subsystem and two high-efficiency buck converters. One converter steps battery voltage to 5V at 5A for the Jetson, camera, and LiDAR. A second converter provides 5V at 1A for the Arduino and small sensors. Motor drivers connect directly to battery voltage through 10A fuses. Total cost for power distribution components is about 40 dollars.

Add a voltage divider circuit connected to an Arduino analog input to monitor battery voltage. This allows software to calculate remaining capacity and warn when charge drops below 20 percent. Calibrate the voltage divider by measuring actual battery voltage with a multimeter at several charge levels and recording corresponding ADC values.

Install an illuminated rocker switch on the robot chassis as the main power switch. Wire this switch to interrupt the battery positive connection before the power distribution board. Position the switch on the rear of the robot where it's easily accessible. Add a large red emergency stop button on top of the robot. Wire the emergency stop in series with motor power so pressing it immediately disconnects motor power independent of computer control. The button uses a latching mechanism requiring manual reset after activation.

Create a charging station using a commercial 4S lithium-ion balance charger rated for 5A charging current. This provides approximately one-hour charge time from 20 to 80 percent capacity. Mount the charger in a fireproof charging box with ventilation holes. Label the charging area clearly and post charging procedures including never leaving batteries unattended and immediately removing batteries that become hot or swell.

**System Integration and Testing**

Assemble the complete system on the robot chassis. Mount the Jetson and Arduino on standoffs to provide cooling airflow underneath. Position the power distribution board centrally to minimize wire lengths. Secure the battery with velcro straps allowing quick removal for charging. Route all wiring with cable ties keeping high-current motor wires separated from low-current signal wires.

Connect all components following the wiring diagram. Use ferrule connectors on stranded wires connecting to screw terminals for reliable connections. Apply heat shrink to solder joints on custom circuits. Double-check polarity on all power connections before applying power for the first time.

Power on the system without motors connected initially. Verify the Jetson boots properly and all 5V rails measure correct voltage. Connect to the robot via SSH and verify network connectivity. Check that battery voltage monitoring reports accurate values. Test the emergency stop by pressing it and verifying motor power disconnects while computer power remains on.

Connect motors and test motion control. Command small velocity values and verify motors respond with correct direction. Monitor current draw during motor operation to ensure it stays within expected ranges. Test emergency stop during motion to confirm immediate shutdown. Run the robot through various maneuvers while monitoring battery voltage to characterize actual power consumption and validate operating time estimates.

Conduct multi-robot networking tests by powering on multiple robots and verifying they all appear on the network with correct IP addresses. Run ROS discovery and confirm all robot nodes are visible from operator workstations. Test bandwidth by streaming camera images from multiple robots simultaneously and verifying frame rates remain acceptable.

**Student Deployment**

Document the complete infrastructure in a student manual covering network connection procedures, SSH access steps, battery charging protocols, power-on and emergency stop procedures, and troubleshooting common issues. Create a quick-start guide students can reference during lab sessions. Record tutorial videos demonstrating safe operation procedures and typical workflows.

Provide students with the robot's IP address and ROS domain ID. Show them how to set their own ROS_DOMAIN_ID environment variable to match. Demonstrate connecting via SSH, launching software, visualizing data in Foxglove Studio, and properly shutting down the system. Explain battery charge monitoring and when to swap batteries.

Students use the infrastructure to focus on robotics algorithms rather than fighting with setup issues. They develop navigation code knowing computation is adequate for their algorithms. They run multi-robot experiments confident that networking supports coordination. They test behaviors for extended periods without battery anxiety. The reliable infrastructure enables them to learn robotics concepts effectively rather than becoming electrical engineers and network administrators.

## Practical Notes

**Computing Platform Selection Guidance**

Match computing power to actual application needs. Simple line-following robots work fine with Raspberry Pi 4. Vision-based applications need GPU acceleration from Jetson platforms. Humanoid control benefits from x86 processors with many cores. Avoid over-specifying compute power as it wastes budget and energy. Consider thermal management when selecting platforms, as some high-performance boards require active cooling. Check software compatibility, ensuring your chosen platform has good ROS 2 support and driver availability for your sensors.

**Network Troubleshooting Tips**

Network issues are common frustrations in robotics labs. When robots cannot communicate, systematically check each layer. Verify physical connection first by pinging the robot's IP address. Check WiFi signal strength as weak signals cause intermittent problems. Confirm all devices use the same ROS_DOMAIN_ID. Verify firewall rules allow DDS multicast traffic. Use tcpdump or Wireshark to observe actual network packets when other diagnostics fail. Create a network troubleshooting checklist students can follow before asking for help.

**Battery Care and Longevity**

Proper battery care significantly extends lifespan and maintains capacity. Avoid deep discharges below 20 percent remaining capacity as this stresses cells. Store batteries at 40 to 60 percent charge for long-term storage, not fully charged or depleted. Keep batteries at room temperature, avoiding hot vehicles or direct sunlight. Balance charge periodically even if not fully discharged to maintain cell balance. Track charge cycles and replace batteries when capacity degrades below 80 percent of original. Educate students on these practices so they develop good habits.

**Cost Optimization Strategies**

Labs with tight budgets can build capable infrastructure through smart choices. Raspberry Pi 4 with 4GB or 8GB RAM handles many applications at one-fifth the cost of Jetson boards. Used or previous-generation compute platforms often work fine for education. Generic motor driver boards cost less than name-brand equivalents with identical specifications. Building custom power distribution with commodity parts saves money compared to commercial power boards. Buy batteries directly from manufacturers rather than through retail channels. Bulk purchasing reduces per-unit costs when building multiple robots.

**Thermal Management**

Computing platforms generate heat that affects reliability and performance. Ensure adequate ventilation around single-board computers. Add heat sinks to processor packages, using thermal compound for good contact. For enclosed designs, add ventilation holes or active cooling fans. Monitor CPU temperatures using software tools and throttle workloads if temperatures exceed safe limits. Position heat-generating components away from temperature-sensitive sensors. In warm climates or outdoor robots, thermal management becomes critical for reliable operation.

**Wireless Range Considerations**

WiFi range limits mobile robot operation. Standard 2.4GHz WiFi provides roughly 50 to 100 meter range indoors with walls and obstacles. The 5GHz band offers higher bandwidth but shorter range. For larger operation areas, deploy multiple access points configured as a mesh or with seamless roaming. Consider directional antennas for point-to-point links over longer distances. Test actual range in your operating environment under load rather than assuming specifications apply. For truly long-range applications, investigate LoRa, cellular, or radio control systems.

**Power Connector Standards**

Standardize on reliable power connectors to prevent connection failures and enable quick swaps. XT60 or XT90 connectors handle high currents reliably for battery connections. Anderson Powerpole connectors provide polarized connections that resist vibration. JST connectors work well for low-current sensor connections. Avoid using barrel jacks for anything beyond light loads as they have high contact resistance and poor vibration resistance. Label positive and negative on all connections. Use different connector types for different voltages to prevent incorrect connections.

**Software Development Workflow**

Efficient development workflows reduce time wasted on deployment and configuration. Develop and test code on desktop computers using simulation before deploying to robots. Use version control like Git to manage code and track changes. Create deployment scripts that automatically copy new code to robots, build packages, and restart services. Implement continuous integration that automatically tests code changes. Use remote development features in modern IDEs to edit code on robots while actually running on robot hardware. Maintain a staging system for testing changes before updating production robots.

**Spare Parts and Maintenance**

Maintain inventory of critical spare parts to minimize downtime when failures occur. Stock extra microcontrollers, motor drivers, cables, connectors, and fuses. Keep at least one spare battery per three robots. Maintain backup copies of fully configured SD cards or storage drives for quick system recovery. Document part numbers and suppliers for all components. Create maintenance checklists covering regular inspection items like checking connection tightness, cleaning sensors, and testing battery capacity. Schedule regular maintenance during non-peak lab times.

**Multi-User Lab Management**

Coordinate resource access when multiple students share limited robots. Implement scheduling systems for robot reservations. Use network-attached storage for shared datasets and code repositories. Configure user accounts with appropriate permissions preventing accidental system damage. Deploy monitoring dashboards showing robot status, battery levels, and usage. Create logging systems that track who used which robot and when for accountability. Establish clear protocols for reporting problems and requesting repairs. Consider remote access queues allowing students to use robots outside normal lab hours.

**Scaling Infrastructure Growth**

Plan infrastructure to scale as your program grows. Document system architecture and design decisions for future reference. Use modular designs where components can be added without redesigning everything. Select network equipment with capacity for future expansion. Budget annually for infrastructure upgrades and replacement of aging components. Track what works well and what causes problems to inform future purchases. Learn from other robotics labs through community forums and conferences to discover better approaches.

## Summary

Compute, networking, and power systems form the essential infrastructure enabling all robot operations. Computing architecture typically uses a two-tier design combining single-board computers like Raspberry Pi or NVIDIA Jetson for high-level processing with microcontrollers like Arduino for real-time control tasks. This division ensures both computational capability and deterministic timing. Networking infrastructure supports robot communication through WiFi or Ethernet connections, requires proper ROS 2 configuration with appropriate domain IDs and DDS settings, and enables remote development through SSH and web-based interfaces. Reliable networking is essential for multi-robot coordination and remote operation. Power systems must provide appropriate voltages through efficient DC-DC conversion, protect components with fuses and battery management systems, deliver adequate capacity for desired operating times, and implement emergency stop capabilities for safety. Proper power system design prevents fires and equipment damage while enabling mobile operation. These three infrastructure domains interact closely where computing power affects power consumption, network bandwidth constrains sensor choices, and power capacity limits operating time. Understanding infrastructure design develops systems engineering thinking about trade-offs and interactions. Practical implementation requires attention to thermal management, reliable connectors, organized wiring, comprehensive documentation, and regular maintenance. Well-designed infrastructure provides the reliable foundation that allows students to focus on learning robotics algorithms, perception, and control rather than constantly troubleshooting basic system failures. Investment in proper infrastructure pays dividends through reduced frustration and increased learning effectiveness.