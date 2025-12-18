---
sidebar_position: 2
---

# Lab: ROS 2 Control of a URDF Robot in Isaac Sim

In this lab, you will learn how to import a URDF (Unified Robot Description Format) model into NVIDIA Isaac Sim and control it using ROS 2. This is a fundamental step in bridging your ROS 2 knowledge with the advanced simulation capabilities of Isaac Sim to create a digital twin.

## Learning Objectives

- Understand the process of importing URDF models into Isaac Sim.
- Establish ROS 2 communication between your ROS 2 environment (e.g., Docker container) and Isaac Sim.
- Control a robot model in Isaac Sim using ROS 2 topics.

## Prerequisites

- Completion of Module 4 Introduction.
- A running instance of NVIDIA Isaac Sim (either locally or on a remote machine).
- Your ROS 2 Humble environment (preferably within the Docker container set up in the Quickstart guide).
- Basic understanding of ROS 2 topics and publishers/subscribers.

## Lab Setup

1.  **Launch Isaac Sim**:
    Start NVIDIA Isaac Sim. Once loaded, you should see the main interface.

2.  **Open a New Stage**:
    Go to `File -> New Stage` to start with a blank simulation environment.

3.  **Import Your URDF Model**:
    Isaac Sim allows you to import URDF models. For this lab, we'll use a simple robot model.
    - Go to `File -> Open` and navigate to the `labs/module-2-isaac/lab_1_isaac_integration/` directory you created. (Once we have a robot model there).
    - Alternatively, you can use one of Isaac Sim's built-in examples (e.g., a Franka Emika Panda robot). For now, let's assume you have a `simple_robot.urdf` file in your `lab_1_isaac_integration` directory.
    - Drag and drop your `simple_robot.urdf` file into the viewport or use the `Content` browser to locate and import it. Ensure the robot appears in the stage.

4.  **Enable ROS 2 Bridge**:
    Isaac Sim includes a built-in ROS 2 bridge.
    - In the menu, go to `Window -> Simulation -> ROS`.
    - Ensure the `ROS 2 Bridge` is enabled. You should see a list of available ROS 2 topics and services.

5.  **Run ROS 2 Environment**:
    Open a terminal and launch your ROS 2 Docker container (or activate your ROS 2 environment).
    ```bash
    # From your project root
    cd labs
    docker run -it --rm -v $(pwd):/workdir physical-ai-book
    ```
    Inside the container:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

## Task 1: Verify ROS 2 Connection

We will first verify that your ROS 2 environment can communicate with Isaac Sim.

1.  **Check ROS 2 Topics**:
    In your ROS 2 terminal (inside the Docker container), list available topics:
    ```bash
    ros2 topic list
    ```
    You should see topics published by Isaac Sim, such as `/ros2_imu`, `/tf`, `/odom`, and potentially joint state topics if your URDF has them configured with the ROS 2 bridge.

2.  **Echo a Topic (Optional)**:
    If you see a relevant topic (e.g., `/ros2_imu`), you can echo it to confirm data flow:
    ```bash
    ros2 topic echo /ros2_imu
    ```
    You should see sensor data being published from Isaac Sim.

## Task 2: Control Robot Joints via ROS 2

Now, let's control a joint of your robot in Isaac Sim using a ROS 2 command. This assumes your robot has at least one controllable joint and the URDF is configured with a ROS 2 controller.

1.  **Identify Joint Control Topic**:
    In Isaac Sim's ROS 2 bridge panel, look for topics related to joint control. Typically, this would be a topic like `/joint_commands` or `/set_joint_position`.

2.  **Publish a Joint Command**:
    Assuming your robot has a joint named `joint_1` (replace with actual joint name from your URDF and Isaac Sim), you can publish a command to set its position.
    In your ROS 2 terminal:
    ```bash
    # Example: Set joint_1 to 0.5 radians
    ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -1
    ```
    Observe your robot in Isaac Sim. `joint_1` should move to the specified position. You might need to adjust the message type and joint names based on your specific URDF and Isaac Sim configuration.

    **Note**: For more complex robots, you would typically use a `ros2_control` setup with controllers for position, velocity, or effort control. This lab provides a simplified direct command for illustration.

## Next Steps

You have successfully imported a URDF into Isaac Sim and established basic ROS 2 control. In future labs, we will delve deeper into integrating sensors, building more sophisticated controllers, and leveraging Isaac ROS for accelerated AI perception.
