---
sidebar_position: 1
---

# Welcome to the Physical AI & Humanoid Robotics Technical Book

This book is designed for advanced AI/CS students, robotics engineers, and educators who wish to bridge the gap between digital artificial intelligence and physical humanoid robots. Our goal is to provide a comprehensive and hands-on guide to embodied intelligence, leveraging modern tools and frameworks like ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) models.

## What You Will Learn

This technical book will guide you through the complete pipeline of embodied AI development:
- **Physical AI Fundamentals**: Understand the core concepts that enable AI to interact with the physical world.
- **ROS 2 Control and URDF**: Learn how to use the Robot Operating System 2 to control robots and model their physical structures.
- **Digital Twins with Gazebo**: Explore the creation and use of high-fidelity simulations for robotics development.
- **Advanced Simulation with NVIDIA Isaac**: Dive into NVIDIA Isaac Sim, Isaac ROS, VSLAM, and Nav2 for realistic and powerful robotics simulations.
- **Vision-Language-Action Models**: Discover how to integrate VLA models for conversational robotics and intuitive control.

## Getting Started

To get the most out of this book, you'll need to set up your development environment. We recommend using Ubuntu 22.04 (Jammy Jellyfish) as the primary operating system for ROS 2 Humble.

### Prerequisites

Before you begin, ensure you have the following installed:
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **Node.js**: Version 18.x or later
- **Python**: Version 3.9+
- **Git**: For cloning the repository
- **Docker**: (Optional, but recommended) For containerized lab environments

### Book Setup

To run the Docusaurus book locally:

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>/frontend
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```

3.  **Run the Development Server**:
    ```bash
    npm start
    ```

    Open your web browser to `http://localhost:3000` to see the book site running.

### ROS 2 and Simulation Setup

For the robotics labs, we provide a recommended Docker-based setup for consistency and reproducibility.

#### Option A: Docker (Recommended)

1.  **Build the Docker Image**:
    - A `Dockerfile` will be provided at the root of the `labs/` directory.
    ```bash
    cd ../labs # navigate from the frontend directory
    docker build -t physical-ai-book .
    ```

2.  **Run the Docker Container**:
    - This command will start the container and provide you with a shell inside it.
    ```bash
    docker run -it --rm -v $(pwd):/workdir physical-ai-book
    ```

3.  **Verify the ROS 2 Installation**:
    - Inside the container, run:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 doctor
    ```
    `ros2 doctor` should confirm your environment is correctly configured.

#### Option B: Manual Installation (Bare Metal)

If you prefer a manual setup, follow these steps:

1.  **Install ROS 2 Humble**:
    - Refer to the official ROS 2 Humble installation guide for Ubuntu:
    - [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2.  **Install Colcon**:
    - Install `colcon`, the build tool for ROS 2 packages:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

3.  **Install Gazebo**:
    - Install Gazebo Fortress, the recommended version for ROS 2 Humble:
    - [https://gazebosim.org/docs/fortress/install](https://gazebosim.org/docs/fortress/install)

With your environment ready, you can now proceed to the first module and begin your hands-on journey into Physical AI and Humanoid Robotics!
