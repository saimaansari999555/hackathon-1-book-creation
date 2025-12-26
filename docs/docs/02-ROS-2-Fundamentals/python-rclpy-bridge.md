---
sidebar_position: 2
---

# ROS 2 Fundamentals: Python with rclpy

## Learning Objectives

- Learn how to create a basic ROS 2 Python package.
- Understand the structure of a ROS 2 Python node using `rclpy`.
- Implement simple publisher and subscriber nodes in Python.
- Work with ROS 2 message types in Python.

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides a convenient and idiomatic Python interface to interact with the ROS 2 graph, allowing you to create nodes, publishers, subscribers, services, and actions using Python.

## Creating a ROS 2 Python Package

Before writing any Python code, you need to create a ROS 2 package. This organizes your code and makes it discoverable by the ROS 2 build system (`colcon`).

```bash
# Navigate to your workspace src directory
# Example: ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_python_pkg --dependencies rclpy std_msgs
```

This command creates a new package named `my_python_pkg` with the `ament_python` build type and declares `rclpy` and `std_msgs` as its dependencies.

## Writing a Simple Publisher

A publisher node sends messages to a topic. Let's create a simple "talker" that publishes string messages.

First, create a Python file, e.g., `my_python_pkg/my_python_pkg/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 for Python
    simple_publisher = SimplePublisher() # Create the node
    rclpy.spin(simple_publisher) # Keep the node alive until it's shut down
    simple_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```

## Writing a Simple Subscriber

A subscriber node receives messages from a topic. Let's create a simple "listener" that prints received string messages.

Create another Python file, e.g., `my_python_pkg/my_python_pkg/listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Making the Executables Installable

To run these Python nodes using `ros2 run`, you need to configure `setup.py` in your package.

Open `my_python_pkg/setup.py` and add the following inside the `setup()` function's `entry_points` dictionary:

```python
    entry_points={
        'console_scripts': [
            'talker = my_python_pkg.talker:main',
            'listener = my_python_pkg.listener:main',
        ],
    },
```

## Building and Running

1.  **Build your package**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_python_pkg
    ```

2.  **Source the setup files**:
    ```bash
    source install/setup.bash
    ```

3.  **Run the publisher in one terminal**:
    ```bash
    ros2 run my_python_pkg talker
    ```

4.  **Run the subscriber in another terminal**:
    ```bash
    ros2 run my_python_pkg listener
    ```

You should see the `listener` terminal printing the messages published by the `talker`.

This demonstrates the basic workflow for creating and running ROS 2 Python nodes using `rclpy` to facilitate communication via topics.
