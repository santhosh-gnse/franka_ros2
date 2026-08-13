# Franka PushT

ROS 2 application package for real FR3 PushT demonstrations and policy deployment.

See `NEED_TO_CONFIGURE.md` for the complete robot-PC configuration and
calibration checklist.

The package is fail-closed: `pusht_robot.yaml` ships with tracking/calibration
placeholders, disabled workspace bounds, and therefore cannot command motion.

## Interfaces

- OptiTrack `PoseStamped`: T marker and EE marker; the goal may be fixed or tracked.
- `/joint_states`: named FR3 positions and velocities.
- `/pusht/observation`: 24 floats in PPO-FB order.
- `/pusht/teleop_action`, `/pusht/policy_action`: normalized XY actions.
- `/pusht/executed_action`: accepted normalized action.
- `/servo_node/delta_twist_cmds`: planar TCP velocity in `fr3_link0`.
- `/pusht/start_episode`, `/pusht/finish_episode`, `/pusht/abort_episode`.

## Local smoke test

Launch fake FR3 hardware separately with `ee_id:=custom_pusher_ee`, then run:

```bash
ros2 launch franka_pusht pusht_mock.launch.py
ros2 topic hz /pusht/observation
ros2 topic echo /pusht/observation --once
```

The mock launch supplies fixed FR3 home joint states. A `/joy` stream is needed
only for nonzero actions. These application-level mocks work on Humble and Jazzy;
the actual Franka/MoveIt stack must be validated on Jazzy.

## Scaling contract

The safety node converts normalized action to metres/second using
`max_linear_speed=0.05`. Its output is therefore physical velocity. Configure
MoveIt Servo with `command_in_type: speed_units`; do not apply a second 0.05 scale.

Policy actions and workspace limits are expressed in the goal frame. Configure
`goal_to_robot_quaternion_wxyz` to rotate those commands into `fr3_link0`.

Real collection starts the standard ROS `joy_node`, followed by
`franka_pusht/ps5_teleop_node`. This PushT-specific adapter publishes normalized
actions only; the safety node remains the sole Servo command publisher. The
existing `ps5` package is not modified or launched by this package.
