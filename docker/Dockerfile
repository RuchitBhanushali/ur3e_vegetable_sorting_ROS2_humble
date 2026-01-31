FROM ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Basic tooling for building ROS workspaces
RUN apt-get update && apt-get install -y \
    git \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# rosdep init/update
RUN rosdep init || true
RUN rosdep update

# Workspace
WORKDIR /ws
RUN mkdir -p /ws/src

# Import pinned third-party dependencies
COPY third_party.repos /ws/third_party.repos
RUN vcs import /ws/src < /ws/third_party.repos

# Copy your project packages (only what is in repo/src)
COPY src/ /ws/src/

# Install all system deps for everything in /ws/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths /ws/src --ignore-src -r -y"

# Build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
