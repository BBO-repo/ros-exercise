FROM osrf/ros:noetic-desktop-full

WORKDIR /workspace

RUN apt update && apt install -y \
  nano git wget software-properties-common \
  ros-noetic-ackermann-msgs ros-noetic-geometry2 \
  ros-noetic-hector-gazebo ros-noetic-hector-models \
  ros-noetic-jsk-rviz-plugins ros-noetic-ros-control \
  ros-noetic-ros-controllers ros-noetic-velodyne-simulator \
  ros-noetic-robot-localization ros-noetic-geographic-msgs \
  python3-tk \
  && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
  apt install -y libgtsam-dev libgtsam-unstable-dev

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
