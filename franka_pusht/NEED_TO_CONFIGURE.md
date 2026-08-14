# Real Robot Configuration Checklist

This package is delivered with fail-closed real-robot defaults. Complete this
checklist on the ROS 2 Jazzy robot PC before allowing nonzero commands.

The real values are configured in:

```text
franka_pusht/config/pusht_robot.yaml
```

Do not replace the values in `pusht_mock.yaml`; that file must remain a safe,
repeatable local test configuration.

## 1. OptiTrack T-marker and EE-marker topics

`mocap4r2_optitrack_driver` (in `src/optitrack_bridge`) publishes every tracked
rigid body together on one topic, `/rigid_bodies`
(`mocap4r2_msgs/msg/RigidBodies`), not per-marker `PoseStamped`. `franka_pusht`
ships `optitrack_bridge_node`, which subscribes to `/rigid_bodies` and
republishes the two markers you name below as independent
`geometry_msgs/msg/PoseStamped` topics, which `observation_node` consumes.
It is already wired into `pusht_collect.launch.py` and `pusht_deploy.launch.py`.

Steps:

1. In Motive, name the two tracked rigid bodies clearly, e.g. `t_block` and
   `ee_marker`.
2. Launch the driver (separate package, separate lifecycle) and activate it:

   ```bash
   ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
   ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
   ```

   The launch file only configures the node; it does not auto-activate.
3. Verify data is flowing:

   ```bash
   ros2 topic echo /rigid_bodies --once
   ```

Configure in `pusht_robot.yaml`:

```yaml
block_rigid_body_name: <name given in Motive for the T-marker>
ee_rigid_body_name: <name given in Motive for the EE-marker>
block_pose_topic: /pusht/t_block_pose   # optitrack_bridge_node output, already set
ee_pose_topic: /pusht/ee_marker_pose    # optitrack_bridge_node output, already set
optitrack_world_frame: map              # driver hardcodes header.frame_id="map"
```

`optitrack_world_frame` must exactly equal `header.frame_id` in both messages;
since the driver hardcodes `"map"` and the bridge node passes the header
through unchanged, this is already correct and should not need editing.

Also fill in the driver's own network settings before it will connect at all:

```text
src/optitrack_bridge/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
```

Set `server_address` to the Motive PC's IP and `local_address` to this
machine's IP (adjust `connection_type` to `Unicast` if multicast isn't
available on your network).

## 2. T-marker to canonical T-block calibration

Configure:

```yaml
block_marker_to_object_translation: [x, y, z]
block_marker_to_object_quaternion_wxyz: [w, x, y, z]
```

This is the fixed transform:

```text
tracked T-marker rigid-body frame -> canonical T-block frame
```

Translation is in metres and is expressed in the tracked marker frame.
Quaternions use scalar-first `[w, x, y, z]` order.

Prefer defining the rigid-body origin and axes correctly in OptiTrack Motive. If
the tracked frame already equals the desired T-block frame, use:

```yaml
block_marker_to_object_translation: [0.0, 0.0, 0.0]
block_marker_to_object_quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
```

Validation: place the T at the goal pose. The first seven observation values
should approach:

```text
position:    [0, 0, 0]
orientation: [1, 0, 0, 0]
```

Inspect with:

```bash
ros2 topic echo /pusht/observation --once
```

## 3. EE-marker to pusher TCP calibration

Configure:

```yaml
ee_marker_to_tcp_translation: [x, y, z]
ee_marker_to_tcp_quaternion_wxyz: [w, x, y, z]
```

This is the fixed transform:

```text
tracked EE-marker frame -> fr3_pusher_tcp
```

First verify the robot-model TCP:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_pusher_tcp
```

Collect simultaneous static poses of the tracked EE marker and robot TCP at
multiple robot configurations. Solve for one constant marker-to-TCP transform,
then validate at additional poses. A single pose is generally insufficient to
verify orientation calibration.

## 4. Fixed goal pose

The current setup assumes no OptiTrack marker on the goal:

```yaml
goal_is_fixed: true
goal_pose_topic: /unused_when_goal_is_fixed
fixed_goal_position: [x, y, z]
fixed_goal_quaternion_wxyz: [w, x, y, z]
```

The fixed goal pose is expressed in `optitrack_world_frame`. Measure the intended
T center and orientation at the goal, not an arbitrary point on the visual goal
marker.

If a tracked goal rigid body is added later:

```yaml
goal_is_fixed: false
goal_pose_topic: /actual/goal/pose
```

## 5. Goal-frame to robot-base rotation

Configure:

```yaml
goal_to_robot_quaternion_wxyz: [w, x, y, z]
```

Policy and teleoperation actions are defined in the goal frame. MoveIt Servo
commands are sent in `fr3_link0`. The safety node applies:

```text
velocity_in_fr3_link0 = R_goal_to_robot * velocity_in_goal
```

For aligned axes use identity:

```yaml
goal_to_robot_quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
```

For a planar yaw angle `theta`:

```text
[cos(theta/2), 0, 0, sin(theta/2)]
```

Validate at low speed: positive goal X must move physically along positive goal
X, and positive goal Y along positive goal Y.

## 6. Safe TCP workspace

Workspace values are the tracked pusher TCP position relative to the goal. They
are not absolute `fr3_link0` coordinates.

Configure:

```yaml
workspace_configured: false
workspace_x: [x_min, x_max]
workspace_y: [y_min, y_max]
workspace_z: [z_min, z_max]
```

Move the TCP manually to each intended safe edge and read observation indices
`7:10`. Add conservative inward margins. Because PushT uses fixed height, Z
should be a narrow interval around the intended relative TCP height.

Only after verifying every bound change:

```yaml
workspace_configured: true
```

With `workspace_configured: false`, the safety node outputs zero.

## 7. Joint-state topic

The default is:

```yaml
joint_state_topic: /joint_states
```

Verify:

```bash
ros2 topic echo /joint_states --once
```

The message must contain positions and velocities for `fr3_joint1` through
`fr3_joint7`. Incoming order does not matter because the observation node maps
entries by joint name.

## 8. PS5 mapping

Real collection uses the standard ROS `joy_node` to publish `/joy`, then the
`franka_pusht` package's `ps5_teleop_node` to apply the controller mapping. It
publishes `/pusht/teleop_action` and never publishes directly to Servo. The
existing `ps5` package remains untouched.

Inspect the controller:

```bash
ros2 topic echo /joy
```

Move or press one control at a time. Configure overrides in `pusht_robot.yaml`:

```yaml
joy_topic: /joy
axis_x: 1
axis_y: 0
speed_axis: 5
deadman_button: 4
deadzone: 0.05
joy_timeout_s: 0.15
```

The current trigger conversion assumes the speed trigger rests at `+1` and is
`-1` when fully pressed. Confirm this on the robot PC.

## 9. Calibration enable

Keep this disabled while any transform is unknown:

```yaml
calibration_configured: false
```

After validating the block offset, TCP offset, fixed goal and goal-to-robot
rotation, enable:

```yaml
calibration_configured: true
```

Both `calibration_configured` and `workspace_configured` must be true for the
safety node to accept nonzero actions.

## 10. Initial low-speed limit

The normal shared limit is `0.05 m/s` in `pusht_common.yaml`. For initial real
tests override it in `pusht_robot.yaml`:

```yaml
max_linear_speed: 0.01
```

After direction, dead-man, timeout, tracking-loss and workspace tests pass,
change the override to `0.05` or remove it.

The safety node already outputs physical m/s. MoveIt Servo must therefore use:

```yaml
command_in_type: speed_units
```

Do not apply another factor of `0.05` inside Servo.

## 11. Jazzy build and launch

On the robot PC:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/franka_workspace
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-up-to franka_pusht franka_fr3_moveit_config
source install/setup.bash
```

Start the robot model with the custom pusher:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=ROBOT_IP \
  use_fake_hardware:=false \
  load_gripper:=false \
  ee_id:=custom_pusher_ee
```

Then start collection nodes:

```bash
ros2 launch franka_pusht pusht_collect.launch.py
```

Before enabling motion verify:

```bash
ros2 topic echo /pusht/observation_valid
ros2 topic echo /pusht/observation --once
ros2 topic echo /pusht/executed_action
ros2 topic echo /servo_node/delta_twist_cmds
```

The last two must remain zero until tracking, calibration, workspace, fresh
joystick input and the dead-man are all valid.

## 12. Remaining recommended safety configuration

The current node commands zero angular velocity and constrains Z through the
workspace. Before real policy deployment, add explicit tracked-pose checks for:

```yaml
fixed_tcp_quaternion_wxyz: [w, x, y, z]
max_tcp_orientation_error_rad: 0.05
fixed_tcp_z: value_relative_to_goal
max_tcp_z_error: value
```

These are not implemented yet. Keep the physical test supervised and rely on
MoveIt Servo/Franka safety plus conservative workspace bounds until they are
added and validated.
