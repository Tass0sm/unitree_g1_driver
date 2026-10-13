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

# PlaCo (QP inverse kinematics, https://github.com/rhoban/placo) for the cartesian controllers'
# "placo" IK solver (obstacle avoidance). Dependencies from the ROS apt repository; PlaCo from source,
# pinned to the commit used on the PC, with placo.patch:
#   - installs the CMake package (placoConfig.cmake, target placo::libplaco) that upstream doesn't;
#   - Frame::parent -> parentJoint, for pinocchio 4 (apt's; the PC has 3.8, which has both);
#   - option PLACO_PYTHON, off here: the bindings need doxystub, and the controller doesn't use them.
# PLACO_JOBS limits parallel compiles: pinocchio-heavy translation units need a lot of memory.
ARG PLACO_COMMIT=ba2b43f
ARG PLACO_JOBS=4
COPY placo.patch /tmp/placo.patch
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-pinocchio \
      ros-jazzy-eiquadprog \
      libjsoncpp-dev \
  && rm -rf /var/lib/apt/lists/* \
  && git clone https://github.com/rhoban/placo.git /tmp/placo \
  && cd /tmp/placo \
  && git checkout ${PLACO_COMMIT} \
  && git apply /tmp/placo.patch \
  && . /opt/ros/jazzy/setup.sh \
  && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPLACO_PYTHON=OFF \
  && cmake --build build -j${PLACO_JOBS} \
  && cmake --install build \
  && ldconfig \
  && cd / && rm -rf /tmp/placo /tmp/placo.patch

# Cartesian controllers from the fork used on the PC (ur_overlay), pinned to the same commit
# (upstream ros2 + "Replace auto_declare with declare_parameter" + solver.synchronize_joint_positions
# + the PlaCo IK solver with obstacle avoidance and a posture task).
ARG CARTESIAN_CONTROLLERS_COMMIT=376c04e
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
