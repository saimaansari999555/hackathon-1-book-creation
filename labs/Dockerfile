# Use the official ROS 2 Humble image as a base
FROM ros:humble-ros-base-jammy

# Set up the working directory
WORKDIR /workdir

# Copy the labs source code into the container
COPY . .

# Install dependencies (e.g., colcon)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up the entrypoint to source ROS 2 and then execute the command
CMD ["bash"]
