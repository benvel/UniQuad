
# ==============================================================
# Base: Ubuntu 22.04 + Python 3.10 + ROS 2 Humble
# ==============================================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --------------------------------------------------------------
# System essentials
# --------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    apt-utils build-essential cmake curl git wget vim sudo \
    gnupg2 lsb-release locales software-properties-common \
    python3-pip python3.10 python3.10-venv python3.10-dev \
    libgl1 libglew-dev libosmesa6-dev patchelf python3-pil \
 && locale-gen en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Upgrade pip
RUN python3 -m pip install --upgrade pip setuptools wheel

# --------------------------------------------------------------
# Python packages for simulation
# --------------------------------------------------------------
RUN python3 -m pip install numpy matplotlib scipy

# Symlink python / pip
RUN ln -sf /usr/bin/python3.10 /usr/bin/python && \
    ln -sf /usr/bin/pip3 /usr/bin/pip

# --------------------------------------------------------------
# ROS 2 Humble Desktop + CycloneDDS
# --------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-argcomplete \
        python3-rosdep \
        python3-vcstool \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# --------------------------------------------------------------
# MuJoCo Python bindings
# --------------------------------------------------------------
RUN pip install mujoco==3.* mujoco-python-viewer

# Optional environment variable for MuJoCo models
ENV MUJOCO_PY_MUJOCO_PATH=/root/unitree_mujoco/unitree_robots

# --------------------------------------------------------------
# Unitree SDK2 Python
# --------------------------------------------------------------
WORKDIR /root
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git && \
    cd unitree_sdk2_python && pip install -e .

# --------------------------------------------------------------
# Unitree ROS2 packages
# --------------------------------------------------------------
WORKDIR /root/unitree_ros2_ws/src
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git

WORKDIR /root/unitree_ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# --------------------------------------------------------------
# Unitree MuJoCo Python simulation
# --------------------------------------------------------------
WORKDIR /root
RUN git clone https://github.com/unitreerobotics/unitree_mujoco.git

# Install Python dependencies
RUN python3 -m pip install mujoco pygame numpy scipy matplotlib

# --------------------------------------------------------------
# Environment setup
# --------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "source /root/unitree_ros2_ws/install/setup.bash" >> /etc/bash.bashrc

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
