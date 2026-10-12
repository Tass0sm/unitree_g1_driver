# Control image for the G1's Jetson (arm64): ros2_control, the FZI cartesian controllers and
# CycloneDDS. The workspace (g1_ws) is mounted at /g1_ws and built inside the container.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    git \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
  && rm -rf /var/lib/apt/lists/*

# Cartesian controllers from the fork used on the PC (ur_overlay), pinned to the same commit
# (upstream ros2 + "Replace auto_declare with declare_parameter" + solver.synchronize_joint_positions).
ARG CARTESIAN_CONTROLLERS_COMMIT=430bbfc
WORKDIR /opt/deps_ws/src
RUN git clone https://github.com/Tass0sm/cartesian_controllers.git \
  && git -C cartesian_controllers checkout ${CARTESIAN_CONTROLLERS_COMMIT}
WORKDIR /opt/deps_ws
RUN apt-get update \
  && rosdep update --rosdistro jazzy \
  && rosdep install --from-paths src/cartesian_controllers/cartesian_motion_controller \
       src/cartesian_controllers/cartesian_controller_handles \
       src/cartesian_controllers/cartesian_controller_base \
       --ignore-src -y --rosdistro jazzy \
  && rm -rf /var/lib/apt/lists/* \
  && . /opt/ros/jazzy/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --packages-up-to cartesian_motion_controller cartesian_controller_handles \
  && rm -rf build log

# The Unitree services live on DDS domain 0.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
COPY cyclonedds_robot.xml /etc/cyclonedds.xml

RUN echo "source /opt/deps_ws/install/setup.bash" >> /root/.bashrc \
  && echo "[ -f /g1_ws/install/setup.bash ] && source /g1_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /g1_ws
CMD ["bash"]
