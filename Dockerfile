# Use the ROS2 Humble image
FROM ros:humble

# Install dev tools and ament_cmake
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
RUN pip3 install --no-cache-dir openai python-dotenv sounddevice whisper scipy

# Copy workspace
WORKDIR /home/ros/nav2_gpt
COPY . /home/ros/nav2_gpt

# Source ROS and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source and run
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
                         source install/setup.bash"]