---
sidebar_position: 1
---

# ROS 2 Fundamentals: Nodes, Topics, and Services

## Learning Objectives

- Understand the fundamental concepts of ROS 2: nodes, topics, and services.
- Differentiate between the publish/subscribe communication model and the client/service request/response model.
- Learn how to inspect ROS 2 communication using command-line tools.

## Introduction to ROS 2

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications. At its core, ROS 2 facilitates communication between different parts of a robot's software system.

## Nodes: The Building Blocks

In ROS 2, a **node** is an executable process that performs a specific task. For instance, one node might be responsible for reading data from a laser scanner, another for controlling motors, and yet another for processing sensor data to detect objects. Nodes are designed to be modular and reusable, allowing developers to create complex systems by combining many small, focused executables.

Each node typically has a unique name within the ROS graph to prevent conflicts and ensure clear identification.

## Topics: Asynchronous Communication (Publish/Subscribe)

**Topics** are the most common way for nodes to exchange data in an asynchronous, many-to-many fashion. This communication pattern is often referred to as "publish/subscribe."

- A node that sends data to a topic is called a **publisher**.
- A node that receives data from a topic is called a **subscriber**.

When a publisher sends a message to a topic, all nodes subscribed to that topic will receive a copy of the message. Topics are ideal for streaming data, such as sensor readings (e.g., camera images, lidar scans), motor commands, or robot odometry.

**Key characteristics of Topics:**
- **Asynchronous**: Publishers don't wait for subscribers to receive messages.
- **Decoupled**: Publishers and subscribers don't need direct knowledge of each other, only of the topic name and message type.
- **Message Types**: All messages transmitted over a topic must conform to a predefined message type (e.g., `sensor_msgs/msg/LaserScan`, `std_msgs/msg/String`).

### Example: A simple talker and listener

Imagine a `talker` node publishing string messages to a `/chatter` topic, and a `listener` node subscribing to `/chatter` to print the messages.

## Services: Synchronous Communication (Request/Response)

**Services** provide a synchronous, one-to-one communication mechanism between nodes. This is a request/response model, similar to a client-server interaction.

- A node that offers a service is called a **service server**.
- A node that requests a service is called a **service client**.

When a service client makes a request to a service server, the client waits for the server to process the request and send back a response. Services are typically used for tasks that involve a single request for computation or action, such as:
- Triggering a robot to perform a specific action (e.g., "take a picture").
- Querying the robot's current state (e.g., "get battery level").
- Performing a complex calculation (e.g., "plan a path").

**Key characteristics of Services:**
- **Synchronous**: The client blocks until it receives a response from the server.
- **Request/Response**: A single request message triggers a single response message.
- **Service Types**: Services also have predefined service types, which define the structure of both the request and response messages.

### Example: A simple add two integers service

A `sum_server` node might offer an `/add_two_ints` service, and a `sum_client` node can request this service with two integer arguments, receiving their sum as a response.

## Inspecting the ROS Graph

ROS 2 provides powerful command-line tools to inspect the active ROS graph and understand how your nodes are communicating:

- `ros2 node list`: Lists all active nodes.
- `ros2 topic list`: Lists all active topics.
- `ros2 topic info <topic_name>`: Shows details about a specific topic, including its message type and the nodes publishing/subscribing to it.
- `ros2 topic echo <topic_name>`: Displays messages being published on a topic in real-time.
- `ros2 service list`: Lists all available services.
- `ros2 service info <service_name>`: Shows details about a specific service.
- `ros2 service call <service_name> <service_type> <arguments>`: Calls a service with specified arguments.

These tools are invaluable for debugging and understanding the flow of data in your ROS 2 applications.
