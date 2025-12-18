---
sidebar_position: 1
---

# Lesson 1: Understanding ROS 2 Nodes

Welcome to your first technical lesson in ROS 2! In this section, we'll explore the most fundamental concept in ROS (Robot Operating System): **Nodes**.

## What is a Node?

Think of a complex robot. It has many jobs to do simultaneously. It might have one process for reading sensor data from its camera, another for controlling the motors in its wheels, and a third for planning a path.

In ROS 2, each of these independent processes is called a **node**.

> **A node is the smallest unit of computation in ROS 2.** It's a program that performs a specific task.

By breaking a complex system down into many small, single-purpose nodes, ROS 2 creates a system that is modular, robust, and easy to debug. If your path planning node crashes, your motor control node can continue to function (for example, by stopping the robot).

## The ROS 2 Graph

Nodes don't operate in isolation. They need to communicate with each other. For example, the camera node needs to send image data to a node that processes the image.

This network of communicating nodes is called the **ROS Graph**.

Here's a simple visualization:

```
[Camera Node] ---- (sends image data) ----> [Image Processing Node]
   ^                                                 |
   |                                                 v
(controls camera)                         (sends commands)
   |                                                 |
[Camera Control Node] <---- (receives status) <---- [Motor Control Node]
```

## How Nodes Communicate

Nodes communicate using a few key mechanisms, which we will explore in detail in the next lessons:

1.  **Topics**: For continuous streams of data (like sensor readings). One node *publishes* data to a topic, and any number of other nodes can *subscribe* to that topic to receive the data.
2.  **Services**: For request/response interactions. One node offers a service, and another node can call that service and wait for a response.
3.  **Actions**: For long-running tasks that provide feedback. For example, telling a robot to navigate to a location. The robot can provide updates on its progress as it executes the action.

## Creating a Node in Python

Let's look at a simple "Hello World" example of a ROS 2 node in Python. Don't worry about understanding every line yet; we'll break it down in the upcoming lab.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello, ROS 2 World!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Parts:

-   `rclpy`: This is the ROS 2 client library for Python.
-   `rclpy.init()`: Initializes the ROS 2 communication.
-   `Node`: We create our own class that inherits from the base `Node` class.
-   `super().__init__('hello_world_node')`: We name our node `hello_world_node`. This name must be unique in the ROS graph.
-   `self.get_logger().info(...)`: A simple way to print messages to the console.
-   `rclpy.spin(node)`: This enters a loop that keeps the node running so it can communicate with other nodes.

## Next Steps

In the next lesson, we will create our own ROS 2 package and write our first real nodes: a publisher and a subscriber.
