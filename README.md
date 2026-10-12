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
large enough. Raise `arm_kp`/`wrist_kp` (and the matching `kd`) at the top of the
`g1_ros2_control` macro in `urdf/g1.ros2_control.xacro`, then relaunch. Keep
`scripts/release_arm_sdk.py`'s `GAINS` in sync.

**If the controller manager dies** without releasing the arms (killed with `-9`, crash), run:
```bash
ros2 run unitree_g1_driver release_arm_sdk.py
```

**Notes:**
- Tuning: `solver.error_scale` and `pd_gains` in `config/controllers.yaml`. Joint gains are in the
  xacro.
- The "Failed to parse type hash" warnings on domain 0 come from Unitree's onboard publishers and
  are harmless.
- RViz's fixed frame is `torso_link` because the marker is stamped at configure time. With any
  other fixed frame, the marker client loops on a TF lookup into the past.
