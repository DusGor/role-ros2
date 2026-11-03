# ROS2 Humble + UF850 (xArm SDK) + Allegro Hand V4 (ros2_control).
# Based on docker/ros2_allegro/Dockerfile; role-ros2 is bind-mounted and built by the entrypoint.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# ============================================================================
# Base system
# ============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        build-essential \
        sudo \
        git \
        curl \
        wget \
        ca-certificates \
        gnupg2 \
        lsb-release \
        locales \
        iputils-ping \
        iproute2 \
        net-tools \
        pkg-config \
        python3-pip \
        python3-dev \
        can-utils \
        kmod \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================================
# ROS 2 Humble
# ============================================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rmw-cyclonedds-cpp \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && \
    rosdep update || true

# ============================================================================
# ros2_control + controllers + helpers (everything the Allegro driver needs)
# ============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-joint-state-broadcaster \
        ros-humble-forward-command-controller \
        ros-humble-imu-sensor-broadcaster \
        ros-humble-position-controllers \
        ros-humble-effort-controllers \
        ros-humble-diagnostic-aggregator \
        ros-humble-diagnostic-updater \
        ros-humble-realtime-tools \
        ros-humble-control-toolbox \
        ros-humble-rclcpp-action \
    && rm -rf /var/lib/apt/lists/*

# Allegro driver system deps; gmock: allegro_hand_v4_hardware and
# allegro_hand_position_effort_controller find_package(ament_cmake_gmock) under BUILD_TESTING
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libspdlog-dev \
        libyaml-cpp-dev \
        libboost-all-dev \
        libgmock-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Python deps (system Python 3.10 — same interpreter as Humble's rclpy)
# ============================================================================
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir \
        xarm-python-sdk \
        pyserial \
        pyzmq \
        numpy \
        scipy \
        pyyaml \
        pytest

# ============================================================================
# Underlay: external repos at pinned commits
# ============================================================================
# Not built: the Gazebo plugin (needs Gazebo) and the grasp controllers
# (Wonik's prebuilt grasp library needs pinocchio 3.6.0).
# allegro_hand_ros2_v4: fork with the right thumb tip fix (branch hand_description_fix).
ARG XARM_ROS2_COMMIT=62936f7ea1846a85f7350de2c4c18f39e6d19715
ARG ALLEGRO_HAND_ROS2_COMMIT=aecb1e94960a8ec02076ad656b86edceff83dce3
RUN mkdir -p /opt/ext_ws/src && cd /opt/ext_ws/src && \
    git clone -b humble https://github.com/xArm-Developer/xarm_ros2.git && \
    git -C xarm_ros2 checkout ${XARM_ROS2_COMMIT} && \
    git clone https://github.com/DusGor/allegro_hand_ros2_v4.git allegro_hand_ros2 && \
    git -C allegro_hand_ros2 checkout ${ALLEGRO_HAND_ROS2_COMMIT} && \
    touch allegro_hand_ros2/allegro_hand_hardwares/gazebo/plugin/COLCON_IGNORE && \
    cd /opt/ext_ws && /bin/bash -c "source /opt/ros/humble/setup.bash && \
        colcon build --merge-install \
            --packages-up-to xarm_description \
                allegro_hand_bringup allegro_hand_control_msgs allegro_hand_v4_description \
                allegro_hand_v4_hardware allegro_hand_position_effort_controller" && \
    rm -rf /opt/ext_ws/build /opt/ext_ws/log

# ============================================================================
# ROS 2 workspace skeleton (role-ros2 is bind-mounted by docker-compose)
# ============================================================================
RUN mkdir -p /app/ros2_ws/src
WORKDIR /app/ros2_ws

# ============================================================================
# Container scripts
# Build context is the repo root (../..) per docker-compose.yaml.
# ============================================================================
COPY docker/ros2_xarm_allegro/entrypoint.sh /entrypoint.sh
COPY docker/ros2_xarm_allegro/ros2_xarm_allegro.env /etc/ros2_xarm_allegro/ros2_xarm_allegro.env
COPY docker/ros2_xarm_allegro/smoke_test.sh /smoke_test.sh
RUN chmod +x /entrypoint.sh /smoke_test.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ext_ws/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/ros2_ws/install/setup.bash ]; then source /app/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /etc/ros2_xarm_allegro/ros2_xarm_allegro.env ]; then source /etc/ros2_xarm_allegro/ros2_xarm_allegro.env; fi" >> /root/.bashrc && \
    echo "export PS1='[ros2_xarm_allegro]\\u@\\h:\\w\\$ '" >> /root/.bashrc

# Fail the build if anything the setup needs is missing
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /opt/ext_ws/install/setup.bash && \
    python3 -c 'import rclpy; from xarm.wrapper import XArmAPI; from allegro_hand_control_msgs.action import GraspCommand' && \
    ros2 pkg executables controller_manager | grep -q ros2_control_node && \
    for p in xarm_description allegro_hand_bringup allegro_hand_v4_description allegro_hand_v4_hardware \
             allegro_hand_position_effort_controller joint_state_broadcaster forward_command_controller \
             imu_sensor_broadcaster diagnostic_aggregator; do \
        ros2 pkg prefix \$p >/dev/null || { echo \"missing package: \$p\"; exit 1; }; \
    done"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
