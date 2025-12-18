# Quickstart: Setting Up Your Environment

**Date**: 2025-12-16
**Status**: Draft

This guide provides the initial steps to set up the development environment for both the Docusaurus book and the ROS 2 labs.

## Prerequisites

- **OS**: Ubuntu 22.04 (Jammy Jellyfish) is the primary supported OS for ROS 2 Humble and this guide.
- **Node.js**: Version 18.x or later.
- **Python**: Version 3.9+
- **Git**: For cloning the repository.
- **Docker**: For containerized environments (optional, but recommended).

## Part 1: Docusaurus Book Setup

This part covers setting up and running the technical book website locally.

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

4.  **Verify**:
    - Open your web browser to `http://localhost:3000`.
    - You should see the Docusaurus website homepage.

## Part 2: ROS 2 and Simulation Setup

This part covers the setup for the robotics labs. We recommend using the provided Docker container for a consistent and reproducible environment.

### Option A: Docker (Recommended)

1.  **Build the Docker Image**:
    - A `Dockerfile` will be provided at the root of the `labs/` directory.
    ```bash
    cd ../labs # from the frontend directory
    docker build -t physical-ai-book .
    ```

2.  **Run the Docker Container**:
    ```bash
    # This command will start the container and give you a shell inside it
    docker run -it --rm -v $(pwd):/workdir physical-ai-book
    ```

3.  **Verify the ROS 2 Installation**:
    - Inside the container, run:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 doctor
    ```
    - `ros2 doctor` should report that your environment is set up correctly.

### Option B: Manual Installation (Bare Metal)

If you prefer not to use Docker, you can follow the official installation guides.

1.  **Install ROS 2 Humble**:
    - Follow the official ROS 2 Humble installation guide for Ubuntu:
    - [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2.  **Install Colcon**:
    - `colcon` is the build tool for ROS 2 packages.
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

3.  **Install Gazebo**:
    - Install Gazebo Fortress, which is the version recommended for ROS 2 Humble.
    - [https://gazebosim.org/docs/fortress/install](https://gazebosim.org/docs/fortress/install)

4.  **Verify**:
    - Run `ros2 doctor` in your terminal.
    - Run `gazebo` to ensure the simulation environment launches.

## Next Steps

With the environment set up, you can now proceed to the first module in the book and start with the hands-on labs.
