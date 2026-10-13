# unitree_g1_driver

ros2_control hardware interface for the upper body of a Unitree G1 (29 DoF, `g1_29dof_rev_1_0`,
`mode_machine` 5), plus a Cartesian motion control demo using the FZI `cartesian_controllers`.

Unitree's locomotion controller keeps the robot standing. This package commands the arms and waist
through Unitree's **arm SDK** (`rt/arm_sdk`), which blends our commands with Unitree's using a
weight (motor slot 29). Joint state comes from `rt/lowstate`.

## How it works

- `G1ArmSdkSystem` (`src/g1_arm_sdk_system.cpp`):
  - All 29 joints have position/velocity/effort state interfaces; legs are state only.
  - Waist and arm joints (17) have a `position` command interface. Joints no controller claims hold
    the pose measured at activation.
  - **Activate:** waits for a fresh `lowstate`, then ramps the weight 0 → 1 over
    `weight_ramp_time` (2 s) while holding the measured pose.
  - **Deactivate, error or Ctrl+C:** holds the last command and ramps the weight 1 → 0
    (this blocks the controller manager for the ramp time).
  - **Stale `lowstate`** (older than `state_timeout`, 0.1 s) while active: error, which releases.
  - **Safety limits:** every sent position is clamped to URDF limits and rate limited to
    `max_joint_velocity` (1 rad/s). PD gains are per joint in `urdf/g1.ros2_control.xacro`.
- Controllers (`config/controllers.yaml`): `{left,right}_arm_motion_controller` with their
  `{left,right}_arm_handle` interactive markers, meeting on `/{left,right}_target_frame`.
  Chain: `torso_link` → `{left,right}_rubber_hand`.
- `use_mock:=true` swaps in `mock_components/GenericSystem`: commands are copied straight to
  states, so it has no gravity, lag or rate limit.

## Where things run

| Robot (Docker `g1_control`, domain 0, DDS on `eth0`) | PC (domain 0, DDS on `robot0`) |
|---|---|
| controller manager, hardware interface, controllers, robot_state_publisher | RViz (`rviz.launch.py`) |

`unitree_hg` in this workspace is vendored from unitree_ros2, with the Connext IDL step made
optional.

## Mock on the PC

```bash
cd g1_ws && direnv allow    # or source .envrc
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
ROS_DOMAIN_ID=77 ros2 launch unitree_g1_driver g1_control.launch.py use_mock:=true arms:=both rviz:=true
```

Use a domain other than 0 for the mock, so it stays off the robot's domain.

## Real robot

Requirements: the robot is standing under Unitree's controller (not damped, not in debug mode),
and someone is at the remote or e-stop.

**Sync and build (PC → robot):**
```bash
rsync -a --delete g1_ws/src/ g1:tassos/g1_ws/src/
ssh g1
~/tassos/g1_ws/src/unitree_g1_driver/docker/build.sh       # only when the Dockerfile changes
~/tassos/g1_ws/src/unitree_g1_driver/docker/run.sh         # a second call opens another shell
colcon build --symlink-install && source install/setup.bash # inside the container
```

**Run:**
```bash
# robot (container): hardware and arm controllers start INACTIVE; nothing is sent on arm_sdk
ros2 launch unitree_g1_driver g1_control.launch.py use_mock:=false arms:=left

# PC: check that /joint_states matches the real robot before going further
ros2 launch unitree_g1_driver rviz.launch.py

# robot (second container shell): take the upper body (2 s blend, holds current pose)
ros2 control set_hardware_component_state G1ArmSdk active
# start Cartesian control and the marker (the controller must be activated before its handle)
ros2 control switch_controllers --activate left_arm_motion_controller left_arm_handle

# hand back
ros2 control switch_controllers --deactivate left_arm_handle left_arm_motion_controller
ros2 control set_hardware_component_state G1ArmSdk inactive   # 2 s blend back to Unitree
```

**Arm sag:** the joints are pure PD, so they droop under gravity until the position error is
large enough (a few mm at the hand). Raise `arm_kp`/`wrist_kp` (and the matching `kd`) at the top
of the `g1_ros2_control` macro in `urdf/g1.ros2_control.xacro`, then relaunch. Keep
`scripts/release_arm_sdk.py`'s `GAINS` in sync.

**If the controller manager dies** without releasing the arms (killed with `-9`, crash), run:
```bash
ros2 run unitree_g1_driver release_arm_sdk.py
```

**Obstacle avoidance (`ik_solver:=placo`):** `g1_control.launch.py ik_solver:=placo` loads
`config/placo.yaml` on top of `controllers.yaml`: the arm controllers use the PlaCo IK solver
(Tass0sm/cartesian_controllers, `placo_obstacles` branch), which keeps spheres approximating each
forearm and hand (`placo.collision_spheres`) at least `placo.obstacles.margin` from the boxes and
spheres published as markers on `/<side>_arm_motion_controller/obstacles` (e.g. by
`g1_perception/table_estimator`). The spheres are shown in RViz
(`/<side>_arm_motion_controller/collision_spheres`). A low-weight posture task
(`placo.posture`) keeps the redundant joints near a nominal posture; above ~1e-5 it visibly pulls
the hand off its target, and much below it the null space is unregulated and the arm jitters. The
image builds PlaCo from source (`docker/Dockerfile`, with `docker/placo.patch`).

**Hands (`hand:=inspire|rubber`, default inspire):** picks the URDF,
`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf` (Unitree's, with fixed `{left,right}_rubber_hand`
frames added at the Inspire fingertips) or the stock one. With `inspire`, the launch also starts
`inspire_hand_joint_states.py`, which shows the fingers from the hand drivers' `~/state` (approximate,
display only). The hand driver itself is `scripts/inspire_hand.py` (Modbus TCP,
`config/inspire_hand.yaml`).

**Walking (`loco_bridge.launch.py`):** `g1_loco_bridge.py` turns body-frame `/g1/cmd_vel` (Twist)
into Unitree's loco `SetVelocity` on `/api/sport/request` (`unitree_api` messages, from
unitree_ros2), so the legs stay with Unitree's controller while `arm_sdk` has the arms. Commands are
capped, rate limited, and stop when `/g1/cmd_vel` goes stale; each one lasts only
`command_duration` on the robot. On firmware 1.5.x the remote's walk/run mode (FSM 802) accepts and
ignores SDK velocity: the SDK walking state is 501, reached with damp → stand up → 501:
```bash
ros2 launch unitree_g1_driver loco_bridge.launch.py
# damps first (the robot goes limp): on the gantry or held
ros2 service call /g1_loco_bridge/enter_walk std_srvs/srv/Trigger
ros2 topic pub -r 20 /g1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"
```
`~/stop` stops and ignores `/g1/cmd_vel` until `~/resume`. Limits and timeouts are in
`config/loco_bridge.yaml`.

**Notes:**
- Tuning: `solver.error_scale` and `pd_gains` in `config/controllers.yaml`. Joint gains are in the
  xacro.
- `solver.synchronize_joint_positions: false` (from the Tass0sm fork of cartesian_controllers) stops
  the arm controllers from resetting their internal model to the measured joints every cycle. The
  G1's joints lag their commands, and with the reset the hand hunts around the target instead of
  settling.
- The "Failed to parse type hash" warnings on domain 0 come from Unitree's onboard publishers and
  are harmless.
- RViz's fixed frame is `torso_link` because the marker is stamped at configure time. With any
  other fixed frame, the marker client loops on a TF lookup into the past.
