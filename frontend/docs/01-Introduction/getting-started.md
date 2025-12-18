---
sidebar_position: 1
---

# Getting Started

Welcome to the "Physical AI & Humanoid Robotics" technical book! This guide will walk you through setting up your environment to follow along with the book and its hands-on labs.

## Philosophy

This book is built on the principle of "learning by doing." Each module is accompanied by labs that are designed to be reproducible and practical. We believe the best way to learn about embodied AI is to build real systems.

## Environment Setup

You have two primary options for setting up your development environment. We strongly recommend the Docker-based approach for a hassle-free and consistent experience.

### Option 1: Docker (Recommended)

Using Docker ensures that you have the exact same environment that the labs were tested in, avoiding issues with dependency versions or operating system differences.

1.  **Install Docker**: If you don't have Docker installed, please follow the official installation guide for your operating system: [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)

2.  **Clone the Project Repository**:
    ```bash
    git clone https://github.com/your-github-org/physical-ai-book.git
    cd physical-ai-book
    ```

3.  **Build and Run the Docker Container**:
    The `Dockerfile` in the `labs/` directory contains all the necessary setup for ROS 2.
    ```bash
    # From the root of the project
    cd labs
    docker build -t physical-ai-book .
    docker run -it --rm -v $(pwd):/workdir physical-ai-book
    ```
    You will now be inside a container with ROS 2 and all necessary tools installed.

### Option 2: Manual Installation (Advanced)

If you are an experienced Linux user and prefer to install everything on your host machine, you can follow the manual installation path.

- **OS**: This guide assumes you are on **Ubuntu 22.04 (Jammy Jellyfish)**.
- Follow the instructions in the project's `quickstart.md` file to install ROS 2 Humble, Gazebo, and other dependencies.

## Running the Book Website Locally

The book is built as a Docusaurus website. You can run it locally to have a better reading experience and to ensure you always have the latest content.

1.  **Navigate to the `frontend` Directory**:
    ```bash
    # From the root of the project
    cd frontend
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```

3.  **Start the Development Server**:
    ```bash
    npm start
    ```

4.  **View the Book**:
    - Open your web browser and navigate to `http://localhost:3000`.
    - You should now see the book's homepage.

You are now ready to start your journey into the world of Physical AI!
