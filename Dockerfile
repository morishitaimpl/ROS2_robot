FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ENV PIP_NO_CACHE_DIR=1

# ===== 基本ツール =====
RUN apt update && apt install -y \
    curl gnupg2 lsb-release sudo \
    git \
    build-essential \
    software-properties-common \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    pkg-config \
 && rm -rf /var/lib/apt/lists/*

# ===== Python / CV / ML (optional) =====
# - OpenCVは aarch64 でも安定して使えるように apt の python3-opencv を推奨
# - PyTorchは pip (CPU) を想定。必要なら --build-arg INSTALL_PYTORCH=1 で有効化
ARG INSTALL_OPENCV=1
ARG INSTALL_PYTORCH=0

RUN if [ "$INSTALL_OPENCV" = "1" ]; then \
      apt update && apt install -y --no-install-recommends \
        python3-numpy \
        python3-opencv \
        libopencv-dev \
      && rm -rf /var/lib/apt/lists/*; \
    fi

RUN if [ "$INSTALL_PYTORCH" = "1" ]; then \
      python3 -m pip install --upgrade pip setuptools wheel && \
      python3 -m pip install \
        --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision torchaudio; \
    fi

# ===== ROS2 repository =====
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ===== Gazebo (gz / Ignition) repository =====
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ===== ROS2 Humble + Gazebo Fortress (ROS2 Humbleのros_gz系と整合) =====
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gz \
    gz-fortress \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# ===== noVNC GUI (XQuartzがGLXで落ちる場合の代替) =====
RUN apt update && apt install -y \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
 && rm -rf /var/lib/apt/lists/*

# ===== rosdep =====
RUN rosdep init || true
RUN rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN chmod +x /usr/local/bin/start-vnc.sh

EXPOSE 6080 5900

CMD ["bash"]
