# Build on an aarch64 host, or use docker buildx --platform linux/arm64.
FROM ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-rosbridge-suite ros-jazzy-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*
