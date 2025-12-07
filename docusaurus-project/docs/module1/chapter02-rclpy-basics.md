---
sidebar_position: 2
---

# Chapter 2: Bridging Python Agents to ROS Controllers with rclpy

## Introduction to `rclpy`: Python client library for ROS 2

`rclpy` is the Python client library for ROS 2. It provides a Python API for developing ROS 2 nodes and interacting with the ROS 2 ecosystem. `rclpy` allows Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and use actions - all from Python code.

The `rclpy` library abstracts the complexity of the underlying ROS 2 middleware (RMW - ROS Middleware), allowing Python developers to focus on their application logic rather than the communication infrastructure. It's designed to be intuitive for Python developers while maintaining compatibility with the ROS 2 architecture.

## Developing ROS 2 nodes in Python

Creating a ROS 2 node in Python using `rclpy` involves several key steps:

1. **Initialize**: Initialize the `rclpy` library
2. **Create Node**: Create an instance of a node class that inherits from `rclpy.node.Node`
3. **Add Functionality**: Add publishers, subscribers, services, or actions to the node
4. **Spin**: Keep the node running to process callbacks

Here's a basic structure of a Python ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, services, etc. here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interfacing AI agents (e.g., reinforcement learning agents) with ROS 2

One of the key applications of `rclpy` is connecting AI agents to ROS 2-based robotic systems. This creates a bridge between high-level decision-making algorithms and low-level robot control systems.

### Reinforcement Learning Integration
Reinforcement learning agents can be integrated with ROS 2 in several ways:

1. **State Observation**: The RL agent receives state information from ROS 2 topics (e.g., sensor data, robot pose)
2. **Action Execution**: The RL agent sends action commands to ROS 2 topics or services (e.g., velocity commands, joint positions)
3. **Reward Calculation**: The RL agent calculates rewards based on ROS 2 data (e.g., reaching a goal, avoiding obstacles)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class RlAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent_node')

        # Publisher for sending actions to the robot
        self.action_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for receiving state information
        self.state_subscription = self.create_subscription(
            Float32,
            '/robot_state',
            self.state_callback,
            10
        )

        # Timer for the agent's decision-making loop
        self.timer = self.create_timer(0.1, self.agent_loop)

        # Initialize your RL agent here
        self.rl_agent = self.initialize_rl_agent()

        # Store current state
        self.current_state = None

    def state_callback(self, msg):
        self.current_state = msg.data

    def agent_loop(self):
        if self.current_state is not None:
            # Get action from RL agent
            action = self.rl_agent.get_action(self.current_state)

            # Publish action to robot
            twist_msg = Twist()
            twist_msg.linear.x = action[0]  # linear velocity
            twist_msg.angular.z = action[1]  # angular velocity
            self.action_publisher.publish(twist_msg)

    def initialize_rl_agent(self):
        # Initialize your RL agent here
        # This could be a pre-trained model or a new instance
        return SimpleAgent()  # Placeholder for your actual agent

class SimpleAgent:
    def get_action(self, state):
        # Simple example: move forward if no obstacle is detected
        if state > 1.0:  # if obstacle is far enough
            return [0.5, 0.0]  # move forward
        else:
            return [0.0, 0.5]  # turn to avoid obstacle
```

## Practical: Controlling a simulated robot joint via a Python script and `rclpy`

Let's create a practical example of controlling a simulated robot joint with enhanced robustness and error handling:

### Advanced Joint Controller with Safety Features

```python
#!/usr/bin/env python3
# advanced_joint_controller.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import time
import numpy as np
from threading import Lock

class AdvancedJointController(Node):
    def __init__(self):
        super().__init__('advanced_joint_controller')

        # Configuration parameters
        self.declare_parameter('controller_name', 'joint_trajectory_controller')
        self.declare_parameter('joint_names', ['joint1'])
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_effort', 100.0)
        self.declare_parameter('safety_timeout', 5.0)

        # Get parameters
        controller_name = self.get_parameter('controller_name').value
        self.joint_names = self.get_parameter('joint_names').value
        control_frequency = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_effort = self.get_parameter('max_effort').value
        self.safety_timeout = self.get_parameter('safety_timeout').value

        # Initialize joint state tracking
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.current_velocities = {name: 0.0 for name in self.joint_names}
        self.current_efforts = {name: 0.0 for name in self.joint_names}
        self.last_update_time = self.get_clock().now()

        # Threading lock for safe access to shared data
        self.joint_state_lock = Lock()

        # Configure QoS for joint state (typically uses BEST_EFFORT for efficiency)
        joint_state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Publishers and subscribers
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            f'/{controller_name}/commands',
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            joint_state_qos
        )

        # Timer for control loop with dynamic frequency
        self.control_timer = self.create_timer(1.0/control_frequency, self.control_loop)

        # Initialize control targets
        self.target_positions = {name: 0.0 for name in self.joint_names}
        self.control_enabled = True
        self.emergency_stop = False

        # Initialize to move to a target position
        self.get_logger().info(f'Advanced Joint Controller initialized for joints: {self.joint_names}')
        self.move_to_positions({name: 0.0 for name in self.joint_names})  # Move to zero positions

        # Safety timer to detect communication timeouts
        self.safety_timer = self.create_timer(self.safety_timeout, self.safety_check)

    def joint_state_callback(self, msg):
        """Callback for joint state updates with safety checks"""
        with self.joint_state_lock:
            try:
                # Update joint positions, velocities, and efforts
                for i, name in enumerate(msg.name):
                    if name in self.current_positions:
                        if i < len(msg.position):
                            self.current_positions[name] = msg.position[i]
                        if i < len(msg.velocity):
                            self.current_velocities[name] = msg.velocity[i]
                        if i < len(msg.effort):
                            self.current_efforts[name] = msg.effort[i]

                # Update last update time
                self.last_update_time = self.get_clock().now()

                # Check for safety violations
                self.check_safety_limits()

            except Exception as e:
                self.get_logger().error(f'Error in joint state callback: {e}')

    def check_safety_limits(self):
        """Check for safety limit violations"""
        for joint_name in self.joint_names:
            # Check velocity limits
            if abs(self.current_velocities[joint_name]) > self.max_velocity:
                self.get_logger().warn(f'Velocity limit exceeded for {joint_name}: {self.current_velocities[joint_name]}')
                self.emergency_stop = True

    def move_to_positions(self, target_positions):
        """Move joints to specified positions"""
        if not isinstance(target_positions, dict):
            self.get_logger().error('Target positions must be a dictionary')
            return

        for joint_name, position in target_positions.items():
            if joint_name in self.target_positions:
                self.target_positions[joint_name] = position
            else:
                self.get_logger().warn(f'Unknown joint: {joint_name}')

    def control_loop(self):
        """Main control loop with safety checks"""
        if not self.control_enabled or self.emergency_stop:
            # Publish zero commands to stop joints safely
            self.publish_zero_commands()
            return

        with self.joint_state_lock:
            # Check if joint state data is fresh
            time_since_update = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
            if time_since_update > self.safety_timeout:
                self.get_logger().error('Joint state timeout - stopping control')
                self.emergency_stop = True
                self.publish_zero_commands()
                return

            # Implement control algorithm (e.g., PID)
            commands = Float64MultiArray()
            command_values = []

            for joint_name in self.joint_names:
                # Calculate control command (simplified PD controller)
                pos_error = self.target_positions[joint_name] - self.current_positions[joint_name]

                # Apply safety limits
                pos_error = max(min(pos_error, 0.1), -0.1)  # Limit position error

                # Simple PD control
                kp = 5.0  # Proportional gain
                kd = 0.1  # Derivative gain (simplified)

                control_output = kp * pos_error - kd * self.current_velocities[joint_name]

                # Apply velocity and effort limits
                control_output = max(min(control_output, self.max_velocity), -self.max_velocity)

                command_values.append(control_output)

            commands.data = command_values
            self.joint_command_publisher.publish(commands)

    def publish_zero_commands(self):
        """Publish zero commands to stop all joints"""
        commands = Float64MultiArray()
        commands.data = [0.0 for _ in self.joint_names]
        self.joint_command_publisher.publish(commands)

    def safety_check(self):
        """Periodic safety check"""
        time_since_update = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        if time_since_update > self.safety_timeout:
            self.get_logger().error('Safety timeout - joint states not updating')
            self.emergency_stop = True
            self.publish_zero_commands()

    def enable_control(self, enable=True):
        """Enable or disable control"""
        self.control_enabled = enable
        if not enable:
            self.publish_zero_commands()
        self.get_logger().info(f'Control {"enabled" if enable else "disabled"}')

    def emergency_stop_all(self):
        """Emergency stop - disable control and publish zero commands"""
        self.emergency_stop = True
        self.enable_control(False)
        self.get_logger().warn('Emergency stop activated')

def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
        controller.emergency_stop_all()
    except Exception as e:
        controller.get_logger().error(f'Unexpected error: {e}')
        controller.emergency_stop_all()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced PID Controller with Adaptive Parameters

```python
class AdaptivePIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01,
                 kp_adapt_rate=0.01, ki_adapt_rate=0.01, kd_adapt_rate=0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        # Adaptation rates for self-tuning
        self.kp_adapt_rate = kp_adapt_rate
        self.ki_adapt_rate = ki_adapt_rate
        self.kd_adapt_rate = kd_adapt_rate

        self.previous_error = 0
        self.integral = 0
        self.error_history = []
        self.max_history = 100  # Keep last 100 errors for analysis

    def compute(self, setpoint, measured_value):
        """Compute PID output with adaptive parameters"""
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * self.dt
        # Anti-windup: limit integral to prevent saturation
        max_integral = 1.0 / self.ki if self.ki != 0 else 1.0
        self.integral = max(min(self.integral, max_integral), -max_integral)
        i_term = self.ki * self.integral

        # Derivative term with noise filtering
        derivative = (error - self.previous_error) / self.dt
        # Apply simple low-pass filter to derivative
        alpha = 0.2  # Filter coefficient
        self.filtered_derivative = alpha * derivative + (1 - alpha) * getattr(self, 'filtered_derivative', derivative)
        d_term = self.kd * self.filtered_derivative

        # Store error for next iteration
        self.previous_error = error

        # Store error in history for adaptation
        self.error_history.append(error)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)

        # Adaptive tuning based on error characteristics
        self.adapt_parameters()

        # Compute output
        output = p_term + i_term + d_term
        return output

    def adapt_parameters(self):
        """Adapt PID parameters based on error characteristics"""
        if len(self.error_history) < 10:
            return

        # Calculate error statistics
        recent_errors = self.error_history[-10:]
        mean_error = sum(recent_errors) / len(recent_errors)
        error_variance = sum((e - mean_error) ** 2 for e in recent_errors) / len(recent_errors)

        # Adapt based on error characteristics
        if abs(mean_error) > 0.1:  # Steady-state error
            self.kp += self.kp_adapt_rate * np.sign(mean_error)
        elif error_variance < 0.01:  # Oscillating
            self.kd += self.kd_adapt_rate * 0.1
        else:  # General adaptation
            self.ki += self.ki_adapt_rate * np.sign(mean_error) * 0.1

        # Keep parameters within reasonable bounds
        self.kp = max(0.1, min(self.kp, 100.0))
        self.ki = max(0.0, min(self.ki, 10.0))
        self.kd = max(0.0, min(self.kd, 10.0))

    def reset(self):
        """Reset the controller state"""
        self.previous_error = 0
        self.integral = 0
        self.error_history = []
```

### Advanced Troubleshooting Tips

**Common Issues and Solutions:**
- **Joint oscillation**: Reduce proportional gain (Kp) or increase derivative gain (Kd)
- **Slow response**: Increase proportional gain (Kp) or integral gain (Ki)
- **Steady-state error**: Increase integral gain (Ki) but watch for oscillation
- **Communication timeouts**: Check controller connection and update rates
- **Safety violations**: Review joint limits and velocity constraints
- **Integration windup**: Implement anti-windup mechanisms in PID controllers

## Exercises: Implement a basic PID controller in Python communicating with ROS 2

### Exercise 1: PID Controller Implementation
1. Implement a PID controller class in Python that can be used to control a robot joint
2. Integrate the PID controller with ROS 2 using `rclpy`
3. Test the controller by commanding the joint to move to different positions

```python
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d_term = self.kd * derivative

        # Store error for next iteration
        self.previous_error = error

        # Compute output
        output = p_term + i_term + d_term
        return output
```

### Exercise 2: Advanced Integration
1. Create a ROS 2 node that uses the PID controller to control multiple joints simultaneously
2. Implement safety checks to prevent dangerous movements
3. Add logging to track the controller's performance

## Summary
This chapter covered the `rclpy` library and how to bridge Python-based AI agents with ROS 2 controllers. You learned how to create ROS 2 nodes in Python, interface AI agents with ROS 2 systems, and implement practical control examples. The exercises provided hands-on experience with PID controllers and advanced integration techniques, preparing you for more complex AI-robotics applications.