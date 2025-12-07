---
sidebar_position: 1
---

# Chapter 1: ROS 2 Nodes, Topics, and Services

## Introduction to ROS 2: What it is and why it's used in robotics

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 is the successor to ROS 1, designed to address the limitations of ROS 1 and to enable robotics applications to be used in production environments.

ROS 2 is particularly useful in robotics for several reasons:
- **Hardware Abstraction**: ROS 2 provides a common interface to interact with various hardware components.
- **Device Drivers**: It offers a wide range of device drivers for common robotic hardware.
- **Libraries**: It includes numerous libraries for common robotic tasks.
- **Visualization Tools**: ROS 2 provides tools for visualizing robot data and behavior.
- **Simulation Tools**: It offers simulation environments for testing robotic applications.
- **Message Passing**: ROS 2 enables communication between different parts of a robot system through messages.
- **Package Management**: It provides a system for organizing and managing robot software packages.

## ROS 2 Concepts: Nodes, Topics, Services, Actions, Messages

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 program. They are organized to form a complete robotic application. You can think of a node as a single process in a larger application.

### Topics
Topics are named buses over which nodes exchange messages. Topics are a unidirectional communication mechanism where nodes can publish data to a topic and other nodes can subscribe to that topic to receive the data. This enables a decoupled architecture where publishers and subscribers don't need to know about each other.

### Services
Services provide a request/response communication pattern. A service client sends a request to a service server, which processes the request and returns a response. Services are synchronous and block until the response is received.

### Actions
Actions are like services, but they are designed for long-running tasks. They provide feedback during execution and can be canceled before completion.

### Messages
Messages are the data structures that are passed between nodes. They are defined in special files with a .msg extension and are used to structure the data that is exchanged between nodes via topics or services.

## Hands-on: Creating a simple publisher-subscriber using `ros2 run`

To demonstrate the publisher-subscriber pattern, let's create a simple example:

1. First, create a publisher node that will publish messages to a topic.
2. Create a subscriber node that will listen to messages on the same topic.
3. Run both nodes and observe the communication.

### Robust Publisher Node with Error Handling

```python
#!/usr/bin/env python3
# publisher_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import sys

class RobustPublisher(Node):
    def __init__(self):
        super().__init__('robust_publisher')

        # Configure QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Add parameter support for runtime configuration
        self.declare_parameter('publish_frequency', 2.0)
        self.declare_parameter('message_prefix', 'Hello World')

        # Handle timer period changes dynamically
        self.update_timer_period()

    def update_timer_period(self):
        """Update timer period based on parameter"""
        freq = self.get_parameter('publish_frequency').value
        if freq > 0:
            self.timer.timer_period_ns = int(1.0 / freq * 1_000_000_000)
            self.get_logger().info(f'Updated publish frequency to {freq} Hz')
        else:
            self.get_logger().warn('Invalid frequency parameter, using default')

    def timer_callback(self):
        """Callback function for the timer"""
        try:
            msg = String()
            prefix = self.get_parameter('message_prefix').value
            msg.data = f'{prefix}: {self.i}'

            # Check if publisher has subscribers before publishing
            if self.publisher_.get_subscription_count() > 0:
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')
            else:
                self.get_logger().warn('No subscribers for topic')

            self.i += 1
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    robust_publisher = RobustPublisher()

    try:
        rclpy.spin(robust_publisher)
    except KeyboardInterrupt:
        robust_publisher.get_logger().info('Interrupted by user')
    except Exception as e:
        robust_publisher.get_logger().error(f'Unexpected error: {e}')
        return 1
    finally:
        robust_publisher.destroy_node()
        rclpy.shutdown()
        return 0

if __name__ == '__main__':
    sys.exit(main())
```

### Robust Subscriber Node with Error Handling

```python
#!/usr/bin/env python3
# subscriber_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import json

class RobustSubscriber(Node):
    def __init__(self):
        super().__init__('robust_subscriber')

        # Configure QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            qos_profile
        )

        # Initialize statistics tracking
        self.message_count = 0
        self.last_message_time = None

        # Prevent unused variable warning
        self.subscription  # type: ignore

    def listener_callback(self, msg):
        """Callback function for receiving messages"""
        try:
            self.message_count += 1

            # Log message with timestamp
            current_time = self.get_clock().now()
            self.get_logger().info(f'Received: "{msg.data}" (Count: {self.message_count})')

            # Perform message validation
            if len(msg.data) == 0:
                self.get_logger().warn('Received empty message')
                return

            # Example of message processing
            self.process_message(msg.data)

            # Update last message time for monitoring
            self.last_message_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {e}')

    def process_message(self, data):
        """Process received message data"""
        try:
            # Example: parse structured data if available
            if data.startswith('{') and data.endswith('}'):
                try:
                    json_data = json.loads(data)
                    self.get_logger().info(f'Parsed JSON: {json_data}')
                except json.JSONDecodeError:
                    # Not JSON, treat as plain text
                    pass

            # Add custom message processing logic here
            # For example: trigger actions, update state, etc.

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

def main(args=None):
    rclpy.init(args=args)
    robust_subscriber = RobustSubscriber()

    try:
        rclpy.spin(robust_subscriber)
    except KeyboardInterrupt:
        robust_subscriber.get_logger().info('Interrupted by user')
    except Exception as e:
        robust_subscriber.get_logger().error(f'Unexpected error: {e}')
        return 1
    finally:
        robust_subscriber.destroy_node()
        rclpy.shutdown()
        return 0

if __name__ == '__main__':
    import sys
    sys.exit(main())
```

### Advanced Troubleshooting Tips

**Common Issues and Solutions:**
- **No messages received**: Check if both nodes are on the same ROS domain ID
- **High latency**: Consider using different QoS profiles (e.g., BEST_EFFORT for less critical data)
- **Memory issues**: Monitor publisher queue depth and adjust accordingly
- **Clock synchronization**: Use `use_sim_time` parameter when working with simulation

## Practical examples: Teleoperation with a joystick, basic sensor data publishing

### Teleoperation with a Joystick
ROS 2 provides packages for teleoperating robots using various input devices, including joysticks. The `joy` package can be used to interface with a joystick and convert its inputs into ROS 2 messages that can control a robot.

### Basic Sensor Data Publishing
Robots typically have various sensors (cameras, LiDAR, IMU, etc.) that publish data. This data is usually published to topics in a structured format using standard message types. For example, a camera sensor might publish images to the `/camera/image_raw` topic using the `sensor_msgs/Image` message type.

## Exercises: Modify message types, create a custom service

### Exercise 1: Modify Message Types
1. Create a custom message type that includes additional information (e.g., timestamp, sensor ID).
2. Modify the publisher to use this custom message type.
3. Update the subscriber to handle the new message type.

### Exercise 2: Create a Custom Service
1. Define a custom service type with request and response fields.
2. Implement a service server that performs a specific robot task (e.g., moving to a specific position).
3. Create a service client that calls the service with appropriate parameters.

## Summary
This chapter introduced the fundamental concepts of ROS 2, including nodes, topics, services, actions, and messages. You learned how to create simple publisher-subscriber examples and explored practical applications like teleoperation and sensor data publishing. The exercises provided hands-on experience with custom message types and services, preparing you for more complex robotic applications.