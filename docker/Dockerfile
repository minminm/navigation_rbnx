FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-bringup \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-smoother \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-msgs \
    ros-humble-dwb-core \
    ros-humble-dwb-critics \
    ros-humble-dwb-plugins \
    ros-humble-navfn \
    ros-humble-pointcloud-to-laserscan \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir grpcio grpcio-tools protobuf pyyaml

WORKDIR /ws

COPY config/ /ws/config/
COPY launch/ /ws/launch/
COPY src/ /ws/src/
COPY docker/entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

ENTRYPOINT ["/ws/entrypoint.sh"]
