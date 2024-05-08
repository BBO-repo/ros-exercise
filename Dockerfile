FROM osrf/ros:noetic-desktop-full

WORKDIR /workspace

RUN apt update && apt install -y \
  nano git wget software-properties-common\
  ros-noetic-ackermann-msgs ros-noetic-geometry2 \
  ros-noetic-hector-gazebo ros-noetic-hector-models \
  ros-noetic-jsk-rviz-plugins ros-noetic-ros-control \
  ros-noetic-ros-controllers ros-noetic-velodyne-simulator \
  ros-noetic-robot-localization ros-noetic-geographic-msgs \
  && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:borglab/gtsam-release-4.0 && \
  apt install -y libgtsam-dev libgtsam-unstable-dev

# make the ros workspace
# RUN mkdir -p /workspace/gem_ws/src/
# # clone the POLARIS_GEM_e2 package
# WORKDIR /workspace/gem_ws/src/
# RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2
# RUN wget -O POLARIS_GEM_e2/simu_update.patch https://www.dropbox.com/scl/fo/hr304092kolzn4g051y6q/AJK1ESrm2AAQyPbtoZ7eE4U/simu_update.patch?rlkey=nvmbw6is5c0pe0633ynlqhh2k&e=1&dl=0

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
