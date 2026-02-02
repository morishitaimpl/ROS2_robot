FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ===== 基本ツール =====
RUN apt update && apt install -y \
    curl gnupg2 lsb-release sudo \
    build-essential \
    software-properties-common \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps

# ===== ROS2 repository =====
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ===== Gazebo (gz / Ignition) repository =====
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg

RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ===== ROS2 Humble + Gazebo Fortress =====
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gz \
    python3-rosdep

# ===== rosdep =====
RUN rosdep init && rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
