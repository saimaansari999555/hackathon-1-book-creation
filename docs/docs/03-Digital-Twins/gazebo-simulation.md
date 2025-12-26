---
sidebar_position: 1
---

# Digital Twins: Gazebo Simulation

## Learning Objectives

- Understand the concept of a digital twin in robotics.
- Learn to use Gazebo as a primary simulation platform for ROS 2 robots.
- Integrate ROS 2 nodes with Gazebo simulations using `ros_gz_bridge`.
- Create basic Gazebo worlds and spawn URDF models within them.

## What is a Digital Twin?

A **digital twin** is a virtual representation of a physical object or system. In robotics, a digital twin allows us to simulate the behavior of a robot and its environment in a virtual space, mimicking real-world conditions. This enables:
- **Safe Development**: Test new algorithms and controls without risking damage to physical hardware.
- **Rapid Prototyping**: Iterate on designs and software much faster than with physical robots.
- **Reproducible Experiments**: Run the same scenario multiple times with precise control over variables.
- **Data Generation**: Create large datasets for training machine learning models (e.g., for perception).

## Introducing Gazebo

**Gazebo** is a powerful 3D robotics simulator widely used in the ROS community. It can accurately simulate complex physics, generate realistic sensor data, and includes a rich set of models and environments. Gazebo is an excellent tool for creating digital twins due to its:
- **Physics Engine**: High-fidelity simulation of rigid body dynamics.
- **Sensor Simulation**: Cameras, LiDAR, IMUs, force/torque sensors, etc.
- **Plugins**: Extend functionality and integrate with external software, notably ROS 2.
- **Visualization**: A 3D user interface to observe simulations.

## Integrating ROS 2 with Gazebo

ROS 2 and Gazebo communicate through a set of Gazebo plugins and the `ros_gz_bridge` package. The `ros_gz_bridge` allows for bidirectional communication, translating ROS 2 messages to Gazebo messages and vice-versa, enabling your ROS 2 nodes to interact with the simulated environment and robot.

### Key Integration Components:

-   **`gazebo_ros_pkgs`**: A collection of ROS 2 packages that provide Gazebo integration.
-   **Gazebo Plugins**: Specific plugins (`libgazebo_ros_init.so`, `libgazebo_ros_force_system.so`, `libgazebo_ros_diff_drive.so`, etc.) are added to your URDF or SDF (Simulation Description Format) files to enable ROS 2 communication (e.g., publishing joint states, subscribing to motor commands).
-   **`ros_gz_bridge`**: Connects ROS 2 topics to Gazebo topics. This is crucial for passing sensor data from Gazebo to ROS 2, and control commands from ROS 2 to Gazebo.

## Creating a Simple Gazebo World

Gazebo worlds are defined using SDF files (`.world`). These files describe the environment, including terrain, objects, lights, and physics properties.

A minimal world file might look like this:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light name="sun" type="directional">
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

You can launch Gazebo with a specific world file:

```bash
gazebo --verbose path/to/your/world_file.world
```

## Spawning URDF Models in Gazebo

Once Gazebo is running, you can spawn your robot models. This is typically done using the `spawn_entity.py` script provided by `ros_gz_sim_demos` or by including the model directly in your `.world` file.

Using `spawn_entity.py` (from a ROS 2 launch file):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'your_robot_description_package' # Replace with your package name
    urdf_file_name = 'my_robot.urdf' # Replace with your URDF file

    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name
    )

    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot',
                       '-x', '0',
                       '-y', '0',
                       '-z', '0.5',
                       '-topic', 'robot_description'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_path}]
        ),
        # You may need to add ros_gz_bridge nodes here
    ])
```

Effective use of Gazebo as a digital twin platform is crucial for developing and testing complex robotic systems before deployment to physical hardware.
