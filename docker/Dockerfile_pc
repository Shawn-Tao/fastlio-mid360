# Shared CPU-only Humble image for amd64 PC and arm64 Jetson.
# Official ros: image is multi-architecture; do not use amd64-only osrf desktop.
FROM ros:humble-ros-base-jammy
ARG USERNAME=fastlio
ARG USER_UID=1000
ARG USER_GID=1000
ARG INSTALL_RVIZ=0
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble SHELL=/bin/bash
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config zsh iproute2 iputils-ping unzip zip \
      python3-dev python3-colcon-common-extensions python3-rosdep python3-yaml \
      python3-numpy python3-matplotlib libeigen3-dev libpcl-dev libboost-all-dev \
      libapr1-dev ros-humble-ament-cmake-auto ros-humble-rosidl-default-generators \
      ros-humble-rclcpp ros-humble-rclcpp-components ros-humble-rcl-interfaces \
      ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
      ros-humble-std-msgs ros-humble-std-srvs ros-humble-visualization-msgs \
      ros-humble-tf2 ros-humble-tf2-ros ros-humble-pcl-ros ros-humble-pcl-conversions \
      ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2 \
    && if [[ "$INSTALL_RVIZ" == 1 ]]; then \
         apt-get install -y --no-install-recommends ros-humble-rviz2 libgl1-mesa-dri; \
       fi \
    && rm -rf /var/lib/apt/lists/*
# Preserve host UID/GID so bind-mounted maps/builds remain user-owned.
RUN if ! getent group "$USER_GID" >/dev/null; then groupadd --gid "$USER_GID" "$USERNAME"; fi \
    && useradd --uid "$USER_UID" --gid "$USER_GID" --create-home --shell /bin/bash "$USERNAME" \
    && mkdir -p /workspace/fastlio-mid360_space \
    && chown "$USER_UID:$USER_GID" /workspace/fastlio-mid360_space \
    && printf '%s\n' 'source /opt/ros/humble/setup.bash' > "/home/$USERNAME/.bashrc" \
    && printf '%s\n' 'source /opt/ros/humble/setup.zsh' > "/home/$USERNAME/.zshrc" \
    && chown "$USER_UID:$USER_GID" "/home/$USERNAME/.bashrc" "/home/$USERNAME/.zshrc"
WORKDIR /workspace/fastlio-mid360_space
USER ${USERNAME}
CMD ["bash"]
