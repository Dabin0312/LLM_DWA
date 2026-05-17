FROM osrf/ros:humble-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    python3-pip \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    xvfb x11vnc novnc websockify fluxbox xterm \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir python-dotenv "openai<1"

# ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

WORKDIR /workspace/ros2_ws
COPY ./ros2_ws/src ./src

# Copy the startup script
COPY start-novnc.sh /root/start-novnc.sh
RUN chmod +x /root/start-novnc.sh

EXPOSE 6080

CMD ["/root/start-novnc.sh"]
