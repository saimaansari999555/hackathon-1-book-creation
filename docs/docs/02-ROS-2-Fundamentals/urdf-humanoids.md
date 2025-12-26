---
sidebar_position: 3
---

# ROS 2 Fundamentals: URDF and Humanoids

## Learning Objectives

- Understand the purpose and structure of the Unified Robot Description Format (URDF).
- Identify key URDF elements: links, joints, and their attributes.
- Learn how to visualize URDF models in ROS 2.
- Grasp the concept of Xacro for modular and parametric robot descriptions.
- Discuss special considerations for modeling humanoid robots.

## What is URDF?

The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS to describe the physical characteristics of a robot. It allows you to define the robot's kinematic and dynamic properties, visual appearance, and collision geometry. URDF files are essential for many ROS tools, including:
- **Simulation**: To represent the robot in physics engines like Gazebo.
- **Motion Planning**: For libraries like MoveIt! to plan collision-free paths.
- **Visualization**: To display the robot model in tools like RViz.

## URDF Structure: Links and Joints

A URDF model is essentially a tree-like structure composed of two primary elements:

1.  **Links**: Represent the rigid bodies of the robot (e.g., torso, head, upper arm, lower arm). Links have properties such as:
    -   `visual`: Describes the visual appearance (geometry, material, color).
    -   `collision`: Defines the collision geometry, which can be simpler than the visual geometry for performance.
    -   `inertial`: Specifies the mass, center of mass, and inertia tensor for physics simulation.

2.  **Joints**: Connect two links and define their relative motion. Joints have properties such as:
    -   `parent`: The link closer to the robot's base.
    -   `child`: The link further from the robot's base.
    -   `type`: The type of motion allowed (e.g., `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`).
    -   `axis`: The axis of rotation or translation for the joint.
    -   `origin`: The pose (position and orientation) of the child link relative to the parent link.
    -   `limit`: For `revolute` and `prismatic` joints, defines the upper and lower bounds of motion.

### Example URDF Snippet

```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>
</robot>
```

## Visualizing URDF Models

ROS 2 provides tools to visualize your URDF models:

-   **`robot_state_publisher`**: A ROS 2 node that reads the robot's joint states and publishes the 3D transforms of the links.
-   **`joint_state_publisher`**: A GUI tool or node that allows you to manually control and publish joint states.
-   **RViz**: The primary 3D visualization tool in ROS. By adding a `RobotModel` display and configuring it to listen to `robot_description` and `joint_states` topics, you can see your robot model.

Typically, you would run these together in a launch file:

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## Xacro: XML Macros for URDF

Writing complex robot descriptions in pure URDF can become repetitive and difficult to manage, especially for robots with many similar components (like humanoid fingers). **Xacro (XML Macros)** is an XML macro language that allows you to:
-   Define reusable macros for common robot components.
-   Use properties and mathematical expressions for parametric robot design.
-   Include other Xacro files for modularity.

Xacro files (`.urdf.xacro`) are processed into standard URDF files before being used by ROS tools.

## Humanoid Robot Considerations

Modeling humanoid robots in URDF presents unique challenges and considerations:

1.  **High Degree of Freedom (DoF)**: Humanoids typically have many joints, requiring careful kinematic chain design and parameterization.
2.  **Symmetry**: Xacro is invaluable for defining symmetric components like arms and legs efficiently.
3.  **Balance and Stability**: Inertial properties become critical for dynamic simulation and balance control.
4.  **End Effectors**: Detailed modeling of hands and grippers for interaction with the environment.
5.  **Sensors**: Integration of cameras, depth sensors, IMUs, and force-torque sensors within the URDF.
6.  **Self-Collision**: Careful definition of collision geometries to prevent the robot from colliding with itself.

Accurate URDF modeling is the foundation for realistic simulation and effective control of humanoid robots.
