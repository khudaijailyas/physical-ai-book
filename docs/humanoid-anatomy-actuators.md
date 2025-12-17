# Humanoid Robot Anatomy & Actuators

## Learning Objectives

- Understand the major anatomical components of humanoid robots and how they relate to the human body
- Learn about different types of actuators used in humanoid systems and their properties
- Recognize how actuator placement and design affect robot movement and capabilities
- Identify the relationship between robot anatomy, actuators, and task performance

## Concept Explanation

### Humanoid Robot Anatomy Overview

Humanoid robot anatomy refers to the physical structure and organization of body parts in human-like robots. Just as human anatomy describes bones, muscles, and organs, robot anatomy describes frames, actuators, and sensors.

The design of humanoid anatomy follows human proportions and structure for practical reasons. This allows robots to fit through doorways, reach countertops, sit in chairs, and use tools designed for humans.

Understanding anatomy is essential because it determines what movements are possible, how much strength the robot has, and what tasks it can perform.

### The Kinematic Chain

A kinematic chain is a series of rigid segments connected by joints. In humanoids, these chains extend from the torso to the hands and feet.

Think of your arm as a kinematic chain. The upper arm connects to the forearm at the elbow joint, and the forearm connects to the hand at the wrist joint. Each joint allows certain movements, and together they enable your hand to reach many positions.

Humanoid robots use similar kinematic chains. The design of these chains determines the robot's workspace—the volume of space it can reach.

### Major Anatomical Regions

Humanoid robots are typically divided into several major regions, each with specific functions.

**The Head**

The head houses sensors for perception. Cameras serve as eyes, providing visual information about the environment. Microphones serve as ears for detecting sounds and speech.

Some humanoids have movable heads with neck joints. This allows the robot to look in different directions without moving its entire body. A typical robot head might have 2 to 3 degrees of freedom—allowing pitch (looking up and down), yaw (looking left and right), and sometimes roll (tilting sideways).

The head's position affects what the robot can see and how humans perceive the robot's attention. When the head turns toward you, you instinctively understand the robot is "looking" at you.

**The Torso**

The torso is the central body segment connecting all major body parts. It houses core components including computers, batteries, and supporting electronics.

In humans, the spine allows the torso to bend and twist. Some humanoid robots include spine-like joints in their torso, typically 1 to 3 degrees of freedom. These allow bending forward and backward, bending sideways, and twisting.

Torso flexibility improves reach and helps with balance. A robot that can bend at the waist can pick up objects from the floor without kneeling.

The torso also serves as the reference point for the robot's coordinate system. Other body parts are positioned relative to the torso.

**The Arms**

Arms enable manipulation—the ability to interact with and move objects. Each arm typically consists of an upper arm, forearm, and hand.

The shoulder joint connects the upper arm to the torso. Human shoulders have remarkable mobility with 3 degrees of freedom. Robot shoulders similarly need multiple degrees of freedom for versatile arm positioning.

The elbow joint connects the upper arm to the forearm. This joint typically has 1 degree of freedom, allowing the forearm to flex toward or away from the upper arm.

The wrist joint connects the forearm to the hand. Wrists typically have 2 to 3 degrees of freedom, allowing the hand to pitch, yaw, and roll. This wrist mobility is crucial for orienting tools and grasping objects from different angles.

A complete arm might have 7 degrees of freedom—3 in the shoulder, 1 in the elbow, and 3 in the wrist. Some designs use more for increased flexibility or fewer for simplicity.

**The Hands**

Hands are the end effectors for manipulation. End effectors are the parts that directly interact with objects and the environment.

Human hands are incredibly complex with 27 degrees of freedom. Robot hands simplify this complexity while maintaining useful grasping capabilities.

Simple robot hands might be parallel jaw grippers with just 1 degree of freedom—open and close. These work well for many tasks but limit versatility.

Anthropomorphic hands mimic human hand structure with a palm and multiple fingers. A typical design includes a thumb and two or three fingers. Each finger has 2 to 4 degrees of freedom.

More degrees of freedom enable more types of grasps. However, more complexity also means more weight, cost, and control difficulty. Designers balance capability against these constraints.

**The Pelvis and Hip**

The pelvis connects the legs to the torso. It serves as the base for leg movement and contains hip joints.

Hip joints are mechanically complex because they must support the robot's entire weight while allowing leg movement in multiple directions. Each hip typically has 3 degrees of freedom—allowing the leg to swing forward and backward, move sideways, and rotate.

The pelvis also houses components for leg actuation and often contains additional batteries or electronics.

**The Legs**

Legs provide mobility through walking or running. Each leg consists of a thigh, shin, and foot.

The knee joint connects the thigh to the shin. This joint has 1 degree of freedom, allowing the shin to flex toward the thigh. Knee flexion is essential for walking, sitting, and navigating obstacles.

Some humanoid designs include additional joints within the leg structure for increased flexibility.

**The Ankles and Feet**

The ankle joint connects the shin to the foot. Human ankles have 2 degrees of freedom—dorsiflexion/plantarflexion (pointing toes up and down) and inversion/eversion (tilting the foot side to side).

Robot ankles typically replicate these movements. Ankle flexibility is crucial for maintaining balance on uneven surfaces and for generating natural walking gaits.

Feet are the contact points with the ground. They must provide stable support while allowing controlled weight transfer during walking.

Flat-footed designs maximize contact area for stability. Some designs include toes for more human-like gait dynamics. Sensors in the feet measure ground contact forces, providing essential feedback for balance control.

### What are Actuators?

Actuators are devices that produce motion by converting energy into mechanical force. In humanoid robots, actuators function like muscles—they create the forces that move joints and body parts.

Every movable joint in a humanoid robot requires one or more actuators. The type and quality of actuators fundamentally determine the robot's movement capabilities.

Actuators must meet several requirements. They must produce sufficient force or torque to move body segments and any carried loads. They must respond quickly to control commands for smooth, coordinated motion. They must be compact and lightweight to fit within the robot's body structure.

### Types of Actuators

Different actuator technologies offer different advantages and trade-offs.

**Electric Motors**

Electric motors convert electrical energy into rotational motion. They are the most common actuators in humanoid robots.

Brushless DC motors are particularly popular. These motors are efficient, controllable, and reliable. They produce smooth rotation and can be precisely controlled using electronic drivers.

Electric motors typically produce high speed but relatively low torque. Torque is the rotational force that moves joints. To get sufficient torque, motors are paired with gearboxes.

Gearboxes are mechanical systems that trade speed for torque. A gearbox with a 100:1 ratio reduces speed by a factor of 100 while increasing torque by a factor of 100. This allows a small, fast motor to produce the high torque needed to move a robot joint.

The disadvantage of gearboxes is that they add weight, can introduce mechanical backlash (play or looseness in the mechanism), and reduce efficiency through friction.

Direct-drive motors eliminate gearboxes by producing high torque directly. These motors are larger and heavier but offer better control precision and more natural, compliant motion.

**Hydraulic Actuators**

Hydraulic actuators use pressurized fluid to create force. A hydraulic system includes a pump that pressurizes fluid, valves that control flow, and actuators (cylinders or motors) that convert fluid pressure into motion.

Hydraulic actuators provide very high power-to-weight ratios. This means they can produce large forces relative to their weight. This makes them attractive for dynamic movements like running and jumping.

The Boston Dynamics Atlas robot uses hydraulic actuators for this reason. The high power output enables impressive athletic capabilities.

However, hydraulic systems are complex. They require pumps, fluid reservoirs, hoses, and careful sealing to prevent leaks. They can be noisy and require regular maintenance. Fluid leaks pose environmental and safety concerns.

**Pneumatic Actuators**

Pneumatic actuators use compressed air instead of fluid. They are lighter and simpler than hydraulic systems but provide less precise control and lower forces.

Pneumatic actuators are more common in soft robotics and compliant systems where precise positioning is less critical than safe, gentle interaction.

Few full-size humanoids use pneumatic actuation as the primary system, though some incorporate pneumatic elements for specific functions.

**Series Elastic Actuators**

Series elastic actuators, or SEAs, incorporate a compliant spring element between the motor and the load. The spring deflects under force, and measuring this deflection provides accurate force sensing.

SEAs enable precise force control. The robot can apply gentle, consistent forces for delicate manipulation or safe physical interaction with humans.

The spring also provides natural compliance. If the robot contacts an unexpected obstacle, the spring absorbs impact rather than transmitting full motor force. This mechanical compliance improves safety.

The disadvantage is reduced bandwidth—the system cannot respond as quickly to control commands because of the spring dynamics. For highly dynamic movements, traditional stiff actuators may be preferable.

**Variable Stiffness Actuators**

Variable stiffness actuators can adjust their compliance. They can be stiff for precise positioning or compliant for safe interaction, changing properties as needed.

These actuators are more complex, typically using additional motors or mechanisms to adjust stiffness. However, they offer flexibility for robots that need both precise control and safe interaction.

### Actuator Placement and Configuration

Where and how actuators are mounted significantly affects robot performance.

**Proximal vs Distal Placement**

Proximal placement means mounting actuators close to the body center, near the torso. Distal placement means mounting actuators far from the center, near the end of limbs.

Proximal placement keeps weight centralized, improving balance and reducing the load on joints. However, it requires long mechanical transmissions (cables, belts, or linkages) to transfer power to distant joints.

Distal placement puts actuators directly at the joints they control. This eliminates transmission losses and provides more direct control. However, it adds weight to the extremities, increasing the inertia that other joints must overcome.

Many designs use hybrid approaches—proximal placement for some joints, distal for others, based on specific requirements.

**Parallel and Serial Linkages**

Joints can be actuated through serial or parallel mechanisms.

Serial linkages connect components in a chain. One joint connects to the next in sequence. This is simple but means errors accumulate—a position error in one joint affects all subsequent joints in the chain.

Parallel linkages use multiple actuators working together to control a single point or segment. Parallel mechanisms can be stiffer and more precise but are mechanically more complex.

Some robot ankles use parallel linkages where two or more actuators work together to control foot position. This provides the stiffness needed to support body weight.

**Cable and Tendon Drive Systems**

Some humanoid robots use cables or tendons to transmit force from proximal actuators to distal joints. This mimics how human muscles and tendons work.

Motors in the torso pull on cables that run through the arms to the hands. This keeps arm weight low while still providing actuation.

Cable systems require careful design to prevent slack, ensure smooth operation, and minimize friction. They can be more difficult to control than direct actuation but offer significant weight advantages.

### Actuator Selection Considerations

Choosing appropriate actuators requires balancing multiple factors.

**Force and Torque Requirements**

Calculate the forces and torques each joint must produce. This depends on the weight of body segments, expected payload, and desired accelerations.

Hip and knee joints must support and move the entire robot weight, requiring high torques. Finger joints move light segments against smaller forces, requiring less torque.

Include safety margins in calculations. Actuators should not operate at their maximum capacity during normal use.

**Speed Requirements**

Different joints need different speeds. Legs must move quickly during walking—perhaps completing a step in 0.5 seconds. Hands might need even faster motion for catching objects.

Actuator speed ratings must meet these requirements after accounting for gear ratios.

**Control Bandwidth**

Control bandwidth refers to how quickly the actuator can respond to changing commands. High bandwidth enables precise, responsive control.

Dynamic tasks like balance control require high bandwidth. The robot must respond quickly to disturbances. Lower bandwidth is acceptable for slow, deliberate movements.

**Power Consumption**

Actuators consume electrical power, which drains batteries. More powerful actuators generally consume more power.

Consider duty cycle—how much of the time the actuator operates at high power. Continuous high power operation drains batteries quickly. Intermittent operation allows battery life to extend.

Balance performance needs against battery capacity and desired operation time.

**Weight and Size**

Every gram of actuator weight must be supported by other actuators and the structure. Heavier actuators require stronger structures, which adds more weight in a cascade effect.

Actuators must physically fit within the robot's body envelope. This is especially constraining in compact designs or limbs with small cross-sections.

**Cost and Reliability**

More sophisticated actuators cost more. For robots with dozens of actuators, cost per unit significantly impacts total system cost.

Reliability is crucial. Actuator failure can disable the robot or require extensive maintenance. Choose proven, reliable actuator technologies for critical joints.

### Gearboxes and Transmissions

Most electric motor systems require gearboxes to match motor characteristics to joint requirements.

**Gear Ratios**

The gear ratio determines the relationship between motor speed and output torque. A 100:1 ratio means the output rotates 100 times slower than the motor but provides 100 times more torque (minus losses to friction).

Higher gear ratios provide more torque but less speed. Lower ratios provide more speed but less torque. Select ratios based on joint requirements.

**Types of Gearboxes**

Spur gears are simple and efficient but produce some backlash. Backlash is play or looseness in the mechanism where small movements don't immediately transfer to the output.

Planetary gearboxes are compact and provide high ratios in small packages. They're common in humanoid joints where space is limited.

Harmonic drives use flexible elements to achieve high ratios with minimal backlash. They're more expensive but provide excellent precision for joints requiring accurate positioning.

Cycloidal drives offer high ratios and good shock resistance. They're sometimes used in joints that experience high impact loads.

**Backlash and Compliance**

Backlash complicates control because there's a dead zone where motor motion doesn't immediately cause output motion. Minimizing backlash improves control precision.

Some compliance in the transmission can be beneficial. It provides shock absorption and allows for force sensing through deflection measurement.

The optimal amount of compliance depends on the application. Precise positioning tasks benefit from stiff transmissions. Safe interaction tasks benefit from compliant transmissions.

### Mechanical Design Principles

The mechanical structure supporting actuators must be carefully designed.

**Structural Loading**

Robot structures experience tension, compression, bending, and torsional loads. The structure must withstand these loads without excessive deflection or failure.

Use finite element analysis in design to predict stress and deflection under expected loads. This ensures structures are strong enough without excessive weight.

**Material Selection**

Aluminum alloys offer good strength-to-weight ratios and are easy to machine. They're common for structural components and housings.

Carbon fiber composites provide even better strength-to-weight ratios but are more expensive and difficult to work with. They're used in weight-critical applications.

Steel is used where high strength is needed in small areas, such as gear shafts and bearings.

Plastics and composites work well for covers, fairings, and non-structural components.

**Bearings and Joints**

Every rotating joint needs bearings to reduce friction and support loads. Ball bearings and roller bearings are common choices.

Bearings must be sized appropriately for expected loads and speeds. Undersized bearings wear quickly and can fail. Oversized bearings add unnecessary weight and friction.

Proper lubrication is essential for bearing life. Sealed bearings retain lubrication and exclude contaminants.

**Thermal Management**

Actuators generate heat during operation. This heat must be dissipated to prevent overheating and performance degradation.

Design adequate heat flow paths from actuators to the environment. Heat sinks, fans, or liquid cooling may be necessary for high-power systems.

Monitor actuator temperatures during operation. Implement thermal protection that reduces power or stops operation if temperatures exceed safe limits.

## Why This Matters

### Enabling Human-Like Capabilities

The anatomy and actuators of humanoid robots directly determine what tasks they can perform. Well-designed anatomy with appropriate actuators enables robots to replicate human capabilities.

A robot with dexterous hands can manipulate tools, turn knobs, and grasp irregular objects. A robot with powerful leg actuators can walk on varied terrain, climb stairs, and carry heavy loads.

Understanding anatomy and actuators is essential for designing robots that meet application requirements.

### Working in Human Environments

Humanoid anatomy allows robots to navigate and function in spaces designed for humans. Doorways, hallways, and stairways all assume human proportions.

Standard furniture, equipment, and tools are designed for human bodies. A humanoid robot with appropriate anatomy can use these without requiring environmental modifications.

This compatibility makes humanoid robots practical for homes, offices, hospitals, and public spaces where redesigning infrastructure would be impractical.

### Safe Human-Robot Interaction

The choice of actuators affects safety during human-robot interaction. Compliant actuators can yield when they contact a person, reducing impact forces.

Smaller, lighter actuators reduce the inertia of moving limbs. Lower inertia means less energy in collisions and safer interaction.

Force-controllable actuators enable the robot to regulate interaction forces. The robot can shake hands gently or provide physical assistance without injury risk.

### Economic and Practical Constraints

Actuator selection affects robot cost, maintenance requirements, and operational costs.

Expensive actuators increase initial cost but might provide better performance or reliability. Less expensive actuators reduce upfront cost but might require more frequent maintenance or replacement.

Power consumption affects operating costs. More efficient actuators extend battery life and reduce charging frequency, important for practical deployment.

Understanding these trade-offs helps designers create economically viable humanoid systems.

### Advancing the Field

Research in humanoid anatomy and actuation drives progress in robotics. New actuator technologies enable new capabilities.

Developments in one area often transfer to other applications. Actuators designed for humanoids might find use in prosthetics, exoskeletons, or industrial robots.

Understanding current capabilities and limitations guides future research directions and innovation.

## Example

### The Unitree H1 Humanoid Robot

The Unitree H1 is a full-size humanoid robot that demonstrates practical anatomical design and actuator integration. Understanding its design illustrates key concepts in humanoid anatomy and actuation.

**Overall Structure**

The H1 stands approximately 1.8 meters tall with proportions similar to an average human adult. This allows it to use standard furniture and navigate typical human spaces.

The robot weighs about 47 kilograms, which is light for a full-size humanoid. This light weight reduces actuator load and power consumption.

The structural frame uses aluminum alloys for strength with minimal weight. External covers are lightweight plastics that protect internal components while maintaining a clean appearance.

**Actuator System**

The H1 uses custom electric actuators throughout its body. These are brushless DC motors paired with precision gearboxes.

Each actuator is a compact integrated unit containing the motor, gearbox, motor driver electronics, and sensors for measuring position and torque. This integration reduces wiring complexity and makes the system modular.

The H1 has a total of 25 degrees of freedom distributed across its body. This provides versatile movement capabilities while keeping complexity manageable.

**Head and Neck**

The head contains vision sensors including RGB cameras and depth sensors for 3D perception. These sensors provide the visual information needed for navigation and manipulation.

The neck has 2 degrees of freedom—pitch and yaw. This allows the robot to look up and down and turn its head left and right.

The head actuators are small and light since they only need to move the relatively light head mass. They use low gear ratios for faster head movement, enabling the robot to quickly scan its environment.

**Torso Design**

The torso has 3 degrees of freedom implemented through a flexible spine mechanism. This allows the robot to bend forward and backward, bend sideways, and twist.

Torso flexibility extends the reach of the arms and helps with balance control. When reaching for low objects, the robot can bend at the waist rather than kneeling, which is faster and more energy-efficient.

The torso houses the main computer system and battery pack. Centralizing these heavy components keeps the center of mass near the body center, improving balance.

**Arm Configuration**

Each arm has 7 degrees of freedom: 3 in the shoulder, 1 in the elbow, and 3 in the wrist. This provides human-like arm mobility.

The shoulder actuators are mounted proximally in the upper torso. This keeps arm weight low. Mechanical linkages transfer motion from these actuators to the shoulder joint.

The elbow actuator is mounted in the upper arm. This direct mounting provides responsive control for the elbow joint.

The wrist actuators are mounted in the forearm. This keeps hand weight manageable while providing the wrist flexibility needed for manipulation.

All arm actuators use relatively high gear ratios (around 50:1 to 100:1) to provide the torque needed for lifting and manipulating objects.

**Hand Design**

The H1 uses a three-finger anthropomorphic hand design. Each hand has a thumb and two fingers.

Each finger has 3 degrees of freedom, allowing it to curl around objects. The thumb is opposable, enabling a variety of grasp types.

The hand actuators are small but powerful. They use high-reduction gearboxes to provide sufficient grip force while maintaining compact size.

Force sensors in the fingertips provide feedback about grip pressure. The control system uses this feedback to adjust grip force—holding objects securely without crushing them.

**Leg Structure**

Each leg has 6 degrees of freedom: 3 in the hip, 1 in the knee, and 2 in the ankle. This configuration supports stable walking and balance control.

The hip actuators are the most powerful in the robot. They must support the entire robot weight during walking. These actuators use high gear ratios (around 150:1) to provide maximum torque.

All three hip actuators are mounted close together at the pelvis. This proximal placement keeps leg mass low, reducing the inertia the actuators must overcome during leg swing.

The knee actuator is mounted in the thigh. It uses a gear ratio around 100:1 to provide the torque needed for standing and walking.

**Ankle and Foot**

The ankle has 2 degrees of freedom—dorsiflexion/plantarflexion and inversion/eversion. These movements are essential for adapting to uneven ground and maintaining balance.

The ankle actuators are mounted in the shin with direct coupling to the ankle joint. This provides responsive control crucial for balance.

The foot is a flat platform with force sensors at each corner. These sensors measure ground reaction forces—how hard each part of the foot presses against the ground.

The control system uses force sensor data to determine the center of pressure (where the robot's weight effectively acts) and adjust balance accordingly.

**Walking Mechanics Step-by-Step**

First, the H1 stands with feet parallel and weight evenly distributed. The hip, knee, and ankle actuators maintain this posture against gravity.

Second, to initiate a step, the control system shifts weight to the left leg. The right leg becomes unloaded, ready to lift.

Third, hip and knee actuators on the right side flex to lift and swing the right leg forward. The ankle actuator maintains foot orientation.

Fourth, while the right leg swings, the left leg actuators continuously adjust to maintain balance. The torso and arms may also move slightly to maintain the center of mass above the support foot.

Fifth, the right leg extends as it swings forward. The knee actuator straightens the leg in preparation for ground contact.

Sixth, the right foot contacts the ground. Force sensors detect this contact and the control system begins transferring weight to the right leg.

Seventh, the process reverses—weight shifts to the right leg, the left leg lifts and swings forward, and the cycle repeats.

Throughout walking, the control system runs at high frequency (500 to 1000 Hz), continuously adjusting all joint actuators to maintain smooth, stable motion.

**Power and Efficiency**

The H1's light weight and efficient actuators enable approximately 2 hours of walking operation on a single battery charge.

During standing still, power consumption is minimal—only enough to hold position against gravity. During walking, consumption increases but remains reasonable due to efficient motor and control algorithms.

The battery management system monitors cell voltages and temperatures, ensuring safe operation and maximum battery life.

**Real-World Capabilities**

The H1 can walk at speeds up to 1.5 meters per second, similar to a comfortable human walking pace. It can navigate uneven terrain, climb stairs, and maintain balance when pushed.

The arms can reach and manipulate objects up to about 5 kilograms. This enables useful tasks like carrying boxes, opening doors, or operating tools.

The integration of appropriate anatomy and actuators enables these practical capabilities, demonstrating how design choices affect real-world performance.

## Practical Notes

### Selecting Actuators for Humanoid Projects

When building or modifying humanoid robots, careful actuator selection is critical.

**Assessing Requirements**

Begin by determining the maximum torque each joint must produce. Calculate static torques (holding position against gravity) and dynamic torques (accelerating body segments).

For a joint supporting a 2 kilogram arm extended horizontally 0.5 meters from the joint, the static torque requirement is approximately 10 Newton-meters (2 kg × 9.8 m/s² × 0.5 m).

Add safety margin—typically 50% to 100%—to account for payloads and unexpected loads. This joint would need an actuator capable of at least 15 to 20 Newton-meters.

Consider speed requirements. A joint completing a 90-degree rotation in 0.5 seconds needs approximately 180 degrees per second or 3.14 radians per second angular velocity.

**Comparing Actuator Options**

Create a comparison matrix including torque, speed, weight, size, cost, and power consumption for candidate actuators.

For hobbyist and research projects, commercial servo motors designed for robotics are widely available. These integrate motor, gearbox, driver, and sensors in a compact package.

For higher-performance applications, consider industrial servo motors with separate gearboxes. These offer better performance but require more integration effort.

**Testing Before Integration**

Test individual actuators before integrating them into the robot. Verify torque output, speed, control response, and power consumption match specifications.

Many actuators perform worse than specifications under real-world conditions. Testing reveals actual capabilities and limitations.

### Working with Gearboxes

Gearboxes are essential but introduce complexity and potential issues.

**Backlash Management**

Measure backlash in assembled gearboxes. Small amounts (less than 1 degree) are usually acceptable. Larger backlash degrades control precision.

For critical joints requiring precision, choose low-backlash gearbox types like harmonic drives or carefully-assembled planetary gearboxes.

Software can partially compensate for backlash through careful control algorithms, but mechanical reduction is always preferable.

**Lubrication and Maintenance**

Properly lubricate gearboxes according to manufacturer recommendations. Under-lubrication causes excessive wear. Over-lubrication can cause drag and attract contaminants.

Plan for periodic maintenance. Gearbox lubrication should be refreshed every 6 to 12 months depending on usage intensity.

**Thermal Considerations**

Gearboxes generate heat through friction. Monitor gearbox temperatures during operation, especially under high load.

If gearboxes consistently run hot (above 60-70°C), improve heat dissipation or reduce load. Excessive heat accelerates lubricant degradation and component wear.

### Mechanical Assembly Best Practices

Proper assembly ensures actuators perform as intended and last longer.

**Alignment**

Ensure actuator shafts align precisely with joints. Misalignment causes binding, increased friction, and accelerated wear.

Use alignment fixtures and precision measurement tools during assembly. Small misalignments (even 0.5 mm) can cause significant issues.

**Fastener Selection and Torque**

Use appropriate fasteners for all connections. High-strength bolts are necessary for high-stress joints like hips and knees.

Torque fasteners to proper specifications. Under-torquing allows loosening over time. Over-torquing can strip threads or damage components.

Apply thread-locking compound to critical fasteners to prevent loosening from vibration.

**Cable Management**

Organize cables carefully to prevent interference with moving parts. Cables that snag on actuators can damage wiring or impede movement.

Use cable carriers or conduits for cables that must move with joints. These protect cables from abrasion and maintain proper routing.

Leave adequate slack in cables to accommodate full joint range of motion. Calculate required cable length through the full motion range before cutting.

### Sensor Integration

Sensors provide essential feedback for actuator control.

**Encoders**

Encoders measure actuator position. Incremental encoders track position changes, while absolute encoders provide position directly.

Mount encoders securely to minimize vibration and ensure accurate readings. Loose encoder mounting causes position errors.

Calibrate encoder zero positions during assembly. The control system needs to know which encoder value corresponds to which physical position.

**Force and Torque Sensors**

Force sensors measure interaction forces. They can be integrated into joints, end effectors, or contact surfaces like feet.

Proper mechanical mounting is critical. Sensors must be in the force path but protected from off-axis loads that could damage them.

Calibrate force sensors regularly. Sensor outputs drift over time, requiring periodic recalibration for accuracy.

**Current Sensing**

Many motor drivers include current sensing that estimates torque. While not as accurate as dedicated torque sensors, current sensing provides useful feedback.

Use current monitoring for overload protection. If current exceeds safe limits, reduce load or stop the actuator to prevent damage.

### Safety Considerations

Actuators powerful enough to move humanoid robots can cause injury if not properly managed.

**Emergency Stop Systems**

Implement hardware emergency stop circuits that immediately cut power to all actuators. These should be independent of software control.

Position emergency stop buttons where operators can quickly reach them. Multiple stop buttons improve safety in large workspaces.

**Force and Position Limiting**

Implement software limits on actuator forces and joint positions. Prevent actuators from exceeding safe torque limits or moving beyond physical joint limits.

Use multiple layers of limiting—software limits as the first line of defense and mechanical hard stops as backup.

**Collision Detection**

Monitor actuator currents and accelerations to detect unexpected collisions. When detected, immediately stop or reduce actuator forces.

This prevents injuries and reduces damage from unintended contacts.

**Protective Enclosures**

During development and testing, operate robots within protective enclosures or barriers. This prevents access to moving parts during operation.

Use transparent barriers where possible so operators can observe robot behavior while remaining protected.

**Gradual Capability Increase**

When first testing new actuator systems, reduce maximum speeds and forces to safe levels. Verify basic functionality before enabling full performance.

Gradually increase limits as confidence in the system grows through successful testing.

## Summary

Humanoid robot anatomy organizes body components into regions including head, torso, arms, hands, pelvis, legs, and feet. Each region serves specific functions and typically contains multiple joints with associated actuators.

Actuators are devices that produce motion, functioning as robot muscles. The main types include electric motors (most common), hydraulic actuators (high power-to-weight ratio), series elastic actuators (force control and compliance), and variable stiffness actuators (adjustable compliance).

Electric motors typically require gearboxes to match motor characteristics to joint requirements. Gearboxes trade speed for torque, with higher gear ratios providing more torque but less speed.

Actuator placement affects robot performance. Proximal placement near the torso reduces limb weight but requires long mechanical transmissions. Distal placement provides direct control but adds weight to extremities.

Selecting actuators requires balancing torque requirements, speed needs, control bandwidth, power consumption, weight, size, cost, and reliability. Each joint has specific requirements based on its function and loading.

Mechanical design must provide adequate structural strength while minimizing weight. Material selection, bearing design, and thermal management all affect performance and longevity.

Understanding anatomy and actuators is essential because they determine what tasks humanoid robots can perform, enable operation in human environments, affect safety during interaction, influence economic viability, and guide future research directions.

Practical implementation requires careful actuator selection based on calculated requirements, proper gearbox integration with attention to backlash and lubrication, precise mechanical assembly with correct alignment and fastening, appropriate sensor integration for control feedback, and comprehensive safety systems including emergency stops and force limiting.

The design and integration of anatomy and actuators represent fundamental engineering challenges in humanoid robotics, directly connecting mechanical hardware to robot capabilities and performance.