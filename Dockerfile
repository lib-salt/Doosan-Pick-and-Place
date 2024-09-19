# IMAGE NAME: sort_and_segregate

ARG ROS_DISTRO=humble
FROM ros:humble

ENV ROS_DISTRO=${ROS_DISTRO}
# Change the default shell to Bash
SHELL [ "/bin/bash", "-c" ]

# Install necessary / useful debians
RUN apt update \
  && apt-get install -y -qq --no-install-recommends \
    byobu \
    curl \
    dbus-x11 \
    vim \ 
    iputils-ping \
    libgtk-3-dev \
    libpoco-dev \
    liburdfdom-tools \
    libyaml-cpp-dev \
    usbutils \
    python3-pip \
    docker.io \
    libignition-gazebo6-dev \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-ament-cmake-python \
    ros-$ROS_DISTRO-ament-lint \
    ros-$ROS_DISTRO-aruco \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-gazebo-msgs \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-hardware-interface-testing \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-view \
    ros-$ROS_DISTRO-interactive-markers \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-launch-testing \
    ros-$ROS_DISTRO-launch-testing-ament-cmake \
    ros-$ROS_DISTRO-launch-testing-ros \
    ros-$ROS_DISTRO-librealsense2* \
    ros-$ROS_DISTRO-moveit* \
    ros-$ROS_DISTRO-moveit-configs-utils \
    ros-$ROS_DISTRO-moveit-msgs \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-perception-pcl \
    ros-$ROS_DISTRO-realsense2-* \
    ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-ros-gz* \
    ros-$ROS_DISTRO-ros2-control* \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rosbag2-storage-mcap* \
    ros-$ROS_DISTRO-rosbridge-msgs \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-rviz-2d-overlay-msgs \
    ros-$ROS_DISTRO-rviz-2d-overlay-plugins \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-xacro \
    ros-dev-tools \
    ros-$ROS_DISTRO-diagnostic-updater \
&& rm -rf /var/lib/apt/lists/*

RUN pip install pyrealsense2

# Git LFS
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash \
  && apt update \
  && apt-get install -y -qq --no-install-recommends \
    git-lfs \
  && rm -rf /var/lib/apt/lists/*

# nividia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics, utility, compute

# Create a workspace and copy the Sort-and-Segregate packages into it
RUN mkdir -p /ros2_ws/src
COPY ./Sort-and-Segregate /ros2_ws/src

# Create User ROS workspace
RUN source /opt/ros/humble/setup.bash \
  && cd /ros2_ws/src \
#   && chmod +x install_emulator.sh \
#   && ./install_emulator.sh \
  && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y \
  && cd /ros2_ws \
  && colcon build --cmake-force-configure \
  && ros2 daemon stop \
  && ros2 daemon start \
  && source /ros2_ws/install/setup.bash

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

WORKDIR /ros2_ws
CMD [ "bash" ]

