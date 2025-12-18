---
sidebar_position: 2
---

# Lab 1: Your First Publisher and Subscriber

This lab will guide you through creating and running your first ROS 2 publisher and subscriber nodes. This is the "Hello World" of robotics.

## Objective

-   Create a ROS 2 package.
-   Write a Python node that publishes a simple string message.
-   Write a Python node that subscribes to that message and prints it.
-   Build and run the nodes to see them communicate.

## Prerequisites

-   You have a working ROS 2 Humble environment set up (preferably via the Docker container).
-   You have cloned the project repository.

## Step 1: Create the ROS 2 Package

First, we need to create a package to hold our code.

1.  Navigate to the `labs/module-1-ros` directory.
2.  Run the `ros2 pkg create` command:
    ```bash
    # Inside the Docker container or your sourced ROS 2 environment
    ros2 pkg create --build-type ament_python lab_1_pubsub --dependencies rclpy
    ```
    This command creates a new directory `lab_1_pubsub` with the necessary files for a Python-based ROS 2 package.

*(This step was completed in task T011).*

## Step 2: Write the Publisher Node

The publisher node will continuously send out a "Hello World" message.

1.  Inside your package, open the file `lab_1_pubsub/publisher.py`.
2.  Add the following code:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        rclpy.init(args=args)
        simple_publisher = SimplePublisher()
        rclpy.spin(simple_publisher)
        simple_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

*(This step was completed in task T012).*

## Step 3: Write the Subscriber Node

The subscriber node will listen for messages on the topic and print them.

1.  Create a new file `lab_1_pubsub/subscriber.py`.
2.  Add the following code:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleSubscriber(Node):
        def __init__(self):
            super().__init__('simple_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.listener_callback,
                10)
            self.subscription

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
*(This step was completed in task T012).*

## Step 4: Add Entry Points

To make our Python scripts executable, we need to add them to the `setup.py` file.

1.  Open `setup.py` in the `lab_1_pubsub` directory.
2.  Modify the `entry_points` section:
    ```python
    'console_scripts': [
        'publisher = lab_1_pubsub.publisher:main',
        'subscriber = lab_1_pubsub.subscriber:main',
    ],
    ```
*(This step was completed in task T012).*

## Step 5: Build and Run

Now, let's build our package and run the nodes.

1.  Navigate to the root of the `labs` directory.
2.  Build the workspace using `colcon`:
    ```bash
    # Inside the Docker container
    colcon build
    ```
3.  Source the new setup files:
    ```bash
    source install/setup.bash
    ```
4.  Open two new terminals. In each, navigate to the `labs` directory and source the setup file again.
5.  In the first terminal, run the publisher:
    ```bash
    ros2 run lab_1_pubsub publisher
    ```
6.  In the second terminal, run the subscriber:
    ```bash
    ros2 run lab_1_pubsub subscriber
    ```

## Expected Outcome

You should see the publisher terminal printing "Publishing..." messages, and the subscriber terminal printing "I heard..." messages.

Congratulations! You have successfully created your first ROS 2 publisher and subscriber.
