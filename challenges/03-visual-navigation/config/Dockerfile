FROM rosclaw/ros2-humble-bridge:latest
RUN apt-get update -o Acquire::Retries=2 \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update -o Acquire::Retries=2 \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
