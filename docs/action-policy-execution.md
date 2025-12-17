# Action & Policy Execution

## Learning Objectives

- Understand how robots translate high-level understanding into physical actions
- Learn what policies are and how they guide robot behavior
- Recognize different approaches to action execution in robotics
- Identify the connection between vision-language understanding and physical movement

## Concept Explanation

### What is Action Execution?

Action execution is the process of making a robot perform physical movements in the real world. After a robot understands what to do through vision and language, it must actually move its body to accomplish tasks.

Action execution bridges the gap between digital intelligence and physical reality. Understanding that you need to pick up a cup is different from successfully moving your arm, opening your gripper, and grasping that cup without dropping it.

In robotics, action execution involves controlling motors, managing forces, coordinating multiple joints, and responding to physical feedback. This is where abstract plans become concrete movements.

### Understanding Policies

A policy is a strategy that determines what actions a robot should take in different situations. It maps observations to actions, essentially answering the question: "Given what I see and know, what should I do next?"

Think of a policy as a decision-making rule. If the robot observes an obstacle ahead, the policy might dictate: "Turn left." If it sees the target object, the policy says: "Move gripper to object position."

Policies can be simple or complex. A simple policy might be a fixed sequence of steps. A complex policy adapts to varying situations and learns from experience.

**Deterministic Policies**

Deterministic policies always produce the same action for the same observation. If you're in situation A, you always do action X.

These policies are predictable and easy to understand. They work well when environments are consistent and tasks are clearly defined.

For example, a deterministic policy might specify: "If the door sensor reads closed, execute the door-opening sequence." The same input always produces the same output.

**Stochastic Policies**

Stochastic policies introduce randomness. For the same observation, they might choose different actions based on probabilities.

These policies help robots explore different solutions and handle uncertainty. If one approach fails, the randomness might lead to trying an alternative.

A stochastic policy might say: "If blocked, turn left 60% of the time and turn right 40% of the time." This variability can help escape local problems.

### Types of Action Representation

Actions can be represented in different ways, each with advantages for different robot types and tasks.

**Joint-Level Actions**

Joint-level actions directly specify the position, velocity, or torque for each robot joint. A humanoid robot might have dozens of joints in its arms, legs, and body.

This low-level representation provides precise control. You specify exactly how much each motor should move or how much force it should apply.

Joint-level control is complex because you must coordinate many joints simultaneously. Moving an arm to grasp an object requires calculating appropriate values for shoulder, elbow, and wrist joints.

**End-Effector Actions**

End-effector actions specify where the robot's hand or tool should move in 3D space. Instead of controlling each joint individually, you specify the desired position and orientation of the gripper.

A motion planning system then calculates the necessary joint movements to achieve this end-effector position. This process is called inverse kinematics.

End-effector actions are more intuitive. You think about where you want the hand to go, not about individual joint angles.

**Task-Space Actions**

Task-space actions describe movements relative to objects or goals. Examples include "move 10 cm toward the cup" or "rotate the object 45 degrees."

These actions are object-centric rather than robot-centric. They define movements in terms of the task, not the robot's body.

Task-space representations help transfer policies between different robots. The same task-space action can be executed by robots with different physical structures.

**Discrete vs Continuous Actions**

Actions can be discrete or continuous. Discrete actions come from a fixed set of options: move forward, turn left, turn right, grasp, release.

Continuous actions can take any value within a range. Joint angles, velocities, and forces are typically continuous.

Discrete actions are simpler to learn and plan with. Continuous actions provide smoother, more precise control but are more challenging to optimize.

### From Vision-Language to Actions

Connecting vision-language understanding to physical actions requires several steps.

**Step 1: Instruction Parsing**

First, the language model interprets the human instruction. "Pick up the red box" gets parsed into structured information: action type is pick-and-place, target object is red box, source location needs visual identification.

This parsing extracts the essential task parameters from natural language.

**Step 2: Visual Grounding**

The vision system identifies the red box in camera images. It determines the box's 3D position, orientation, and dimensions.

Visual grounding connects the language reference to actual pixels and spatial coordinates. The abstract concept "red box" becomes a specific object at specific coordinates.

**Step 3: Action Planning**

Given the task goal and object location, the action planning system determines what sequence of movements will accomplish the task.

For picking up a box, the plan might include: move arm to pre-grasp position above box, lower gripper around box, close gripper, lift box to safe height.

This plan considers robot capabilities, workspace constraints, and collision avoidance.

**Step 4: Policy Execution**

The policy executes the planned actions. It controls motors to move the arm along the planned trajectory, monitors sensors for contact with the object, and adjusts grip force appropriately.

During execution, the policy continuously processes feedback. If the gripper contacts the box earlier than expected, it adjusts the motion accordingly.

**Step 5: Monitoring and Adjustment**

While executing actions, the robot monitors progress through vision and other sensors. If something unexpected happens, the policy must adapt.

If the box starts slipping, the policy tightens the grip. If an obstacle appears, the policy modifies the trajectory. This real-time adjustment ensures robust task completion.

### Policy Learning Approaches

Robots acquire policies through different learning methods.

**Manual Programming**

Engineers manually write code that implements the policy. They specify exactly what the robot should do in each situation based on their understanding of the task.

Manual programming works for well-defined, repetitive tasks in controlled environments. Factory assembly robots often use manually programmed policies.

However, manual programming becomes impractical for complex, variable tasks. It's difficult to account for all possible situations the robot might encounter.

**Imitation Learning**

Imitation learning means the robot learns by watching demonstrations. A human performs the task while the robot observes, and the robot learns a policy that mimics these demonstrations.

The robot might watch a human pick up objects in various ways, learning to replicate similar movements for new objects.

Imitation learning is efficient because it leverages human expertise. Humans show the robot what good behavior looks like, and the robot learns to reproduce it.

**Reinforcement Learning**

Reinforcement learning means the robot learns through trial and error. It tries different actions, receives rewards for successful outcomes, and gradually learns which actions work best.

The robot might attempt to grasp an object many times, receiving positive rewards for successful grasps and negative rewards for failures. Over time, it learns an effective grasping policy.

Reinforcement learning can discover solutions humans might not think of, but it typically requires many practice attempts.

**Vision-Language-Action Models**

Modern VLA models integrate vision, language, and action learning into unified systems. These models learn policies directly from combined visual, linguistic, and action data.

A VLA model might observe videos of tasks with language descriptions and learn to predict appropriate actions. Given new visual scenes and instructions, it generates action sequences.

These integrated models capture the connections between what robots see, what they're told, and what they should do.

### Action Execution Challenges

Several challenges make action execution difficult in real-world robotics.

**Sensor Noise and Uncertainty**

Sensors provide imperfect information. Camera images might be blurry, depth sensors might have errors, and joint encoders might drift.

Policies must handle this uncertainty. They can't assume perfect information about the world or the robot's own state.

Robust policies incorporate safety margins and verify actions through multiple sensor modalities.

**Dynamics and Contact**

Physical interactions involve complex dynamics. Objects have mass, friction, and elasticity. When a robot grasps something, contact forces are unpredictable.

Policies must account for these physical properties. Grasping a rigid metal part requires different forces than grasping a soft foam object.

Learning these physical interactions often requires real-world experience, as simulation doesn't perfectly capture reality.

**Latency and Real-Time Constraints**

Robot control happens on fast timescales. Motors need new commands every few milliseconds to achieve smooth motion.

Computing actions this quickly is challenging, especially when using complex vision or language models. Policies must run fast enough to meet real-time requirements.

Some systems use hierarchical control where slow high-level planning runs alongside fast low-level control.

**Generalization to New Situations**

Policies trained on specific scenarios often fail when conditions change. A policy that works on one table might fail on a different surface height or lighting condition.

Good policies generalize across variations in environment, objects, and task parameters. Achieving this generalization requires diverse training data and appropriate model architectures.

**Long-Horizon Tasks**

Many useful tasks require long sequences of actions. Making a sandwich involves dozens of individual manipulation steps spread over minutes.

Policies must maintain context and goals over these long horizons. They must remember what's been accomplished and what remains to be done.

Long-horizon execution also increases the chance of errors somewhere in the sequence, requiring robust error recovery.

### Closed-Loop vs Open-Loop Control

Action execution can operate in closed-loop or open-loop modes.

**Open-Loop Execution**

Open-loop execution follows pre-planned action sequences without feedback. The robot executes a series of commands regardless of what happens.

This approach is simple and fast. The robot doesn't need to process sensor data during execution.

However, open-loop control is fragile. If anything deviates from expectations, the robot cannot adjust. A slight position error at the start compounds throughout the sequence.

**Closed-Loop Execution**

Closed-loop execution continuously monitors sensors and adjusts actions based on feedback. The robot observes the effects of its actions and corrects errors in real time.

Visual servoing is a common closed-loop technique. The robot continuously looks at its gripper and the target object, adjusting its motion to keep them aligned.

Closed-loop control is more robust but computationally intensive. Processing sensor feedback and recalculating actions takes time and resources.

Most modern robots use closed-loop control for critical operations and open-loop control for simple, predictable movements.

### Multi-Modal Action Policies

Advanced policies integrate multiple types of information to make better action decisions.

**Vision-Guided Policies**

Vision-guided policies use camera images as primary inputs. They see objects, obstacles, and workspace conditions, adjusting actions accordingly.

These policies excel at tasks requiring visual precision like threading a needle or inserting a plug into a socket.

**Force-Guided Policies**

Force-guided policies respond to tactile feedback. They sense contact forces and pressures, adjusting grip strength and push forces.

These policies are essential for delicate manipulation like handling eggs or feeling for object edges.

**Hybrid Policies**

Hybrid policies combine vision, force, and other sensory modalities. They use vision for coarse positioning and force for fine adjustments.

For example, a robot might use vision to guide its hand near an object, then switch to force control for final contact and grasping.

## Why This Matters

### Enabling Physical Task Completion

All the vision and language understanding in the world accomplishes nothing without action execution. Policies transform understanding into useful physical work.

Robots exist to perform tasks in the physical world. Action execution is where this purpose is realized. Without effective action policies, robots are merely passive observers.

### Ensuring Safe Human-Robot Interaction

How robots execute actions directly impacts safety. Policies must ensure smooth, controlled movements that don't endanger people.

Force-limited policies prevent robots from applying excessive pressure. Collision detection policies stop movement when unexpected contact occurs.

Safe action execution enables robots to work alongside humans in shared spaces.

### Adapting to Real-World Variability

Real environments are messy and unpredictable. Objects aren't perfectly positioned, surfaces aren't perfectly flat, and lighting changes throughout the day.

Adaptive policies handle this variability through feedback and adjustment. They don't require perfect conditions to function.

This adaptability is crucial for service robots operating in homes, hospitals, and public spaces where environments constantly change.

### Enabling Dexterous Manipulation

Complex manipulation tasks require sophisticated action execution. Assembling products, preparing food, or performing surgery demands precise, coordinated movements.

Advanced policies enable these dexterous behaviors. They coordinate multiple joints, balance stability with mobility, and execute fine motor control.

As policies improve, robots gain capabilities approaching human-level manipulation.

### Learning from Experience

Policies that learn improve over time. Each task execution provides information about what works and what doesn't.

This learning capability allows robots to become more effective with experience. Early attempts might be clumsy, but the robot gradually masters tasks.

Learning policies reduce the need for manual programming and enable robots to adapt to new tasks more quickly.

## Example

### A Robotic Arm Assembling a Simple Product

Let's follow how a robotic arm uses action policies to assemble a simple product—attaching a wheel to an axle. This example shows the complete pipeline from vision-language understanding to physical execution.

**Initial Setup and Instruction**

A human supervisor tells the robot: "Attach the wheel to the axle on the base." The workspace contains a base with a protruding axle and a wheel lying flat on the table.

The robot's cameras capture images of the workspace. The language model parses the instruction, identifying this as an assembly task requiring picking up the wheel and placing it onto the axle.

**Step 1: Visual Scene Analysis**

The robot's vision system processes the camera images. It detects two key objects: the wheel and the base with axle.

For each object, the vision system determines 3D position and orientation. The wheel is lying flat at coordinates (X1, Y1, Z1). The axle is vertical at position (X2, Y2, Z2) with the mounting end facing upward.

The vision system also identifies the wheel's center hole and estimates its diameter. This information is critical for alignment during assembly.

**Step 2: Grasp Planning**

The policy first plans how to grasp the wheel. Given the wheel's flat orientation, the best approach is to grasp it from the side.

The policy computes a pre-grasp position slightly offset from the wheel's edge. This position keeps the gripper clear during approach.

The grasp policy specifies gripper width, approach angle, and contact points. For this wheel, the policy plans to grip two opposite edges parallel to the wheel face.

**Step 3: Executing the Grasp**

The robot executes the grasp in closed-loop mode. As the arm moves toward the planned position, the vision system continuously tracks both the gripper and the wheel.

The policy adjusts the trajectory in real time based on visual feedback. If the gripper is slightly off-target, it corrects the path.

As the gripper reaches the wheel, force sensors detect contact. The policy closes the gripper with controlled force—enough to hold the wheel securely but not so much that it damages the material.

Force feedback confirms a successful grasp. The policy verifies that both gripper fingers made contact and that the grasp is stable.

**Step 4: Lifting and Reorienting**

With the wheel grasped, the policy executes a lift movement to clear the table surface. The robot raises the arm 10 cm to ensure safe clearance.

Now the policy must reorient the wheel. Currently horizontal, the wheel needs to be vertical to align with the vertical axle.

The policy computes a rotation sequence. It rotates the wrist joint to bring the wheel into vertical orientation. During this rotation, the policy monitors accelerometer data to ensure the wheel doesn't slip in the gripper.

**Step 5: Visual Alignment**

The robot moves the wheel above the axle. Visual servoing begins—a closed-loop process that uses continuous vision feedback.

The vision system identifies both the wheel's center hole and the axle tip. It calculates the offset between them.

The policy generates small corrective movements to align these two features. Move 2 mm left, 1 mm forward, rotate 3 degrees. Each adjustment brings the wheel closer to perfect alignment with the axle.

This visual servoing continues until the alignment error is below a threshold, perhaps 1 mm.

**Step 6: Insertion with Force Control**

Now comes the critical insertion phase. The policy switches from position control to a hybrid position-force control mode.

The policy commands downward motion while monitoring vertical force. As the wheel contacts the axle tip, force sensors detect the contact.

The policy adjusts downward force to press the wheel onto the axle. It pushes with enough force to overcome friction but not so much that it damages components.

During insertion, the policy monitors lateral forces. If it detects unexpected side forces, it indicates misalignment. The policy responds by slightly adjusting the wheel's lateral position and rotation while continuing downward pressure.

**Step 7: Verification and Release**

As the wheel slides onto the axle, force feedback indicates progress. When the wheel reaches the mounting position, insertion force suddenly drops—the wheel has seated properly.

The policy verifies successful assembly through multiple checks. Vision confirms the wheel is at the correct height on the axle. Force sensors confirm the wheel is seated with appropriate pressure.

The policy then opens the gripper to release the wheel. It monitors that the wheel remains stable after release—it doesn't fall or shift position.

**Step 8: Retreat and Final Confirmation**

The robot retracts its arm away from the assembled product. The policy plans a safe retreat path that avoids colliding with the newly assembled wheel.

Final vision-based verification occurs. The camera captures an image of the completed assembly. The vision system confirms the wheel is properly mounted on the axle at the correct position and orientation.

The robot reports completion: "Wheel attached to axle successfully."

**Handling a Complication**

Suppose during insertion, the policy detects unusual resistance—the wheel isn't sliding smoothly onto the axle. This indicates a potential misalignment or interference.

The policy implements an error recovery procedure. It retracts the wheel slightly, performs a small circular search motion to find better alignment, then attempts insertion again.

If repeated attempts fail, the policy escalates to the human supervisor: "I'm having difficulty inserting the wheel. The axle may be misaligned or obstructed. Please check the assembly."

This example demonstrates how action policies integrate vision feedback, force control, error detection, and adaptive behavior to accomplish a physical assembly task reliably.

## Practical Notes

### Policy Implementation Frameworks

Several frameworks facilitate policy development and deployment for robots.

**PyTorch and TensorFlow**

Most learned policies use deep learning frameworks like PyTorch or TensorFlow. These frameworks provide tools for building, training, and deploying neural network policies.

PyTorch is popular in research for its flexibility and debugging capabilities. TensorFlow offers better deployment tools for production systems.

Both frameworks support GPU acceleration, which is essential for training complex policies on large datasets.

**ROS Control**

ROS Control provides interfaces between high-level policies and low-level robot hardware. It manages real-time control loops, safety limits, and hardware abstraction.

Use ROS Control to implement policies that work across different robot models. The same policy code can control various arms or mobile bases through standardized interfaces.

**MuJoCo and Isaac Gym**

Before deploying policies on real robots, test them in physics simulation. MuJoCo and Isaac Gym are physics engines designed for robotics.

Simulate thousands of policy executions to identify failures and edge cases. Train reinforcement learning policies entirely in simulation before real-world deployment.

These simulators model dynamics, contacts, and sensors with reasonable accuracy, though sim-to-real transfer always requires careful validation.

**OpenAI Gym and Gymnasium**

These frameworks provide standardized interfaces for reinforcement learning environments. They make it easy to train and compare different policy learning algorithms.

Many robotics simulators provide Gym-compatible interfaces, enabling seamless integration with standard RL algorithms.

### Designing Action Spaces

Carefully design your robot's action space for the tasks you need to accomplish.

**Choosing Action Representation**

For precise manipulation tasks, continuous end-effector actions often work best. They provide fine control over gripper position and orientation.

For navigation tasks, discrete or continuous velocity commands are typical. Specify linear and angular velocities to control mobile base movement.

For dynamic tasks like throwing or hitting, joint torque control provides the most direct control over forces and accelerations.

Match your action representation to task requirements and your robot's capabilities.

**Action Scaling and Normalization**

Normalize action values to consistent ranges, typically -1 to 1 or 0 to 1. This makes learning more stable and policies more robust.

Scale normalized actions to actual robot commands based on safety limits and hardware capabilities. A normalized action of 0.5 might map to 50% of maximum joint velocity.

Implement hard limits to prevent actions outside safe ranges. Clip commanded actions that would violate joint limits or speed constraints.

**Temporal Discretization**

Decide how frequently your policy generates new actions. Control rates typically range from 10 Hz for slow tasks to 1000 Hz for dynamic tasks.

Higher rates provide smoother control but require faster computation. Lower rates are easier to achieve but may miss fast dynamics.

Many systems use hierarchical control with slow high-level policies (1-10 Hz) setting goals for fast low-level controllers (100-1000 Hz).

### Training Policies

Different tasks require different training approaches.

**Collecting Training Data**

For imitation learning, collect demonstrations of the task. Humans can teleoperate the robot, or you can record people performing the task for the robot to learn from.

Collect diverse demonstrations covering different object positions, orientations, and conditions. More diversity helps policies generalize better.

For reinforcement learning, define a reward function that captures task success. Positive rewards for desired outcomes, negative rewards for failures and inefficient actions.

**Simulation Training**

Train policies in simulation first. Simulation is faster, safer, and allows exploring failure cases without risk.

Use domain randomization—vary physics parameters, visual appearance, and object properties during simulation training. This helps policies transfer to the real world where conditions differ from simulation.

**Real-World Fine-Tuning**

After simulation training, fine-tune policies with real-world data. Even limited real-world experience significantly improves performance.

Start with safe, simple scenarios and gradually increase difficulty. Monitor policies closely during early real-world testing.

**Continuous Learning**

Implement systems that continue learning during deployment. As robots encounter new situations, they update policies based on outcomes.

Store successful and failed executions for later analysis and retraining. This data helps identify policy weaknesses and guides improvements.

### Safety in Action Execution

Safe action execution is paramount when robots operate near people or valuable equipment.

**Velocity and Force Limiting**

Implement hard limits on maximum velocity and force. These limits should be enforced at the hardware level when possible.

Collaborative robots often use torque-sensing in every joint to immediately detect unexpected contact and stop.

Set conservative limits during development and testing. Gradually increase limits only after extensive validation.

**Collision Detection and Avoidance**

Use multiple sensors to detect potential collisions before they occur. Cameras, proximity sensors, and tactile sensors all contribute to safety.

Implement emergency stop behaviors that activate upon detecting unexpected contact or proximity to humans.

Plan trajectories that maintain safety margins around people and obstacles. Slow down or stop when safety margins decrease.

**Bounded Workspaces**

Define workspace boundaries that the robot must not exceed. These might be physical barriers or virtual boundaries enforced in software.

Policies should naturally stay within boundaries, but implement hard stops as backup safety measures.

**Human Monitoring**

During development and early deployment, maintain human supervision. Operators should have emergency stop buttons readily accessible.

Use simulation and extensive testing to verify safety before unsupervised operation.

**Graceful Degradation**

Design policies that degrade gracefully when sensors fail or uncertainty increases. Slow down, increase caution, and request help rather than continuing blindly.

### Debugging Action Policies

When policies fail, systematic debugging identifies and fixes problems.

**Logging and Visualization**

Log all sensor data, policy outputs, and executed actions during operation. These logs are essential for understanding failures.

Visualize policy behavior. Plot trajectories, display what the policy sees, and show internal activations. Visualization reveals issues invisible in raw numbers.

**Isolating Failure Sources**

Is the problem in vision, language understanding, or action execution? Test each component separately to isolate the failure source.

Replace real sensors with simulated perfect data. If the policy works with perfect data, the issue is in perception. If it still fails, the problem is in the policy itself.

**Gradual Complexity Increase**

Start with the simplest possible scenario where the policy works. Gradually add complexity until failure occurs. This identifies what specific factor causes problems.

If grasping works for one object but not another, systematically vary object properties to find the critical difference.

**Systematic Parameter Tuning**

Policies have many parameters—learning rates, reward weights, control gains. Systematic tuning improves performance.

Use structured approaches like grid search or Bayesian optimization rather than random trial and error.

Document all parameter changes and their effects. This builds understanding of how parameters affect behavior.

## Summary

Action and policy execution transforms robot understanding into physical behavior. After vision systems perceive the environment and language models interpret instructions, policies determine and execute appropriate actions.

Policies are decision-making strategies that map observations to actions. They can be deterministic or stochastic, manually programmed or learned from data. Modern approaches include imitation learning, reinforcement learning, and integrated vision-language-action models.

Actions can be represented at different levels: joint angles for precise control, end-effector positions for intuitive specification, or task-space commands for transferability. The choice depends on the task and robot capabilities.

The pipeline from understanding to action involves instruction parsing, visual grounding, action planning, policy execution, and continuous monitoring. Each step connects abstract understanding to concrete physical movements.

Action execution faces challenges including sensor noise, complex dynamics, real-time constraints, generalization requirements, and long-horizon task completion. Closed-loop control using continuous feedback provides robustness against these challenges.

Effective action policies enable physical task completion, ensure human safety, adapt to real-world variability, enable dexterous manipulation, and improve through experience. They are essential for robots to perform useful work in real environments.

Practical implementation uses frameworks like PyTorch, ROS Control, and physics simulators. Careful action space design, diverse training data, simulation-to-real transfer, and continuous learning all contribute to effective policies.

Safety requires velocity and force limiting, collision detection, bounded workspaces, human monitoring, and graceful degradation. Systematic debugging through logging, visualization, isolation, and parameter tuning improves policy reliability.

As policy learning advances, robots gain increasingly sophisticated physical capabilities. The integration of vision, language, and action through learned policies represents a powerful approach to creating intelligent, capable robot systems.