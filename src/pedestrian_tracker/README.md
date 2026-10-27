# YOLO to BRNE integration

This package converts the existing YOLO 3D tracks and robot pose into the two
inputs expected by `brne_social_nav`:

- `/pedestrians` (`crowd_nav_interfaces/msg/PedestrianArray`) in `brne_odom`
- `/brne_odom` (`nav_msgs/msg/Odometry`) in `brne_odom`

The robot transform uses `/lf/sportmodestate` by default because it provides
continuous 20 Hz odometry. `/utlidar/robot_pose` was observed updating far too
slowly for camera ego-motion compensation and remains available only with
`use_sport_state_pose:=false`.

The pedestrian adapter transforms each camera-relative 3D measurement into the
global planning frame at the original YOLO image timestamp, then estimates
position and velocity there.
This ego-motion compensation prevents a stationary pedestrian from appearing
to move when the robot moves. Isolated one-frame tracks and sub-0.15 m/s
velocity jitter are suppressed. YOLO string track IDs are mapped to compact
integer IDs because the C++ BRNE node uses IDs as vector indices.

RealSense uses a device-clock timestamp while the robot TF is stamped in the
host ROS clock. The adapter estimates the clock offset from detection arrival
times and maps the source timestamp into the TF clock domain.

The green pedestrian-center marker uses the current YOLO box-center height
(`z_smoothing:=0.0`) so it does not vertically lag the 3D debug box. Frames
without historical TF are skipped instead of being transformed with a newer
robot pose and creating false pedestrian motion.

When a detection is slightly newer than the latest robot TF, the adapter now
extrapolates the complete camera pose instead of reusing a stale orientation.
For the TF gap `dt`, it applies

```text
delta_yaw = yaw_rate * dt
p_base(t+dt) = p_base(t) + R(yaw) * v_body * dt
r_camera(t+dt) = Rz(delta_yaw) * r_camera(t)
q_camera(t+dt) = qz(delta_yaw) * q_camera(t)
```

It then transforms the camera-relative pedestrian measurement with that
advanced pose. Rotating the camera lever arm as well as its quaternion prevents
an in-place robot turn from being interpreted as pedestrian translation. The
30-degree camera pitch is preserved by quaternion composition.

The YOLO integration launch disables BRNE's fixed corridor sample mask by
default with `use_corridor_mask:=false`. The legacy wall display is controlled
separately by `use_walls` and is also off by default. Enable the mask only when
`y_min` and `y_max` describe real physical boundaries in `brne_odom`.

For the open-space BRNE experiment, the planner publishes the exact internal
sample arrays as blue robot line strips on `/brne/robot_samples` and orange
pedestrian GP line strips on `/brne/pedestrian_samples`. The controller uses a
fixed goal-heading law (`omega_nom = heading_gain * wrapped_heading_error`);
its heading error, nominal command, and final BRNE-weighted command are reported
on `/brne/heading_diagnostics`.

Build the integration packages from the workspace root:

```bash
colcon build --packages-select \
  crowd_nav_interfaces brnelib crowd_nav pedestrian_tracker
source install/setup.bash
```

With the configured YOLO stack already running, validate perception and BRNE
without sending commands to the robot:

```bash
ros2 launch pedestrian_tracker brne_yolo.launch.py use_rviz:=true
```

Check that `/pedestrians` and `/brne_odom` are updating and that their frame is
`brne_odom`. After confirming the TF tree and camera mounting offsets, first
test the controller on a topic that is not connected to the robot:

```bash
ros2 launch pedestrian_tracker brne_yolo.launch.py \
  start_controller:=true cmd_vel_topic:=/cmd_vel_test
```

The optional Go2 bridge converts `geometry_msgs/msg/Twist` into Unitree Sport
API Move (1008) and StopMove (1003) requests. It is disabled and disarmed by
default. It scales commands to at most 0.20 m/s and 0.40 rad/s while retaining
the planned path curvature. It sends StopMove if `cmd_vel` becomes stale for
0.30 seconds or if robot roll/pitch exceeds 0.35 radians.

In `brne_yolo.launch.py`, a newly received RViz goal automatically arms the
bridge after SportModeState and tilt checks pass. An explicit disarm stops the
robot immediately; the next new goal arms it again. Set
`unitree_auto_arm_on_goal:=false` to require the service call for every trial.

For a supervised hardware test, start the controller and bridge while leaving
the bridge disarmed:

```bash
ros2 launch pedestrian_tracker brne_yolo.launch.py \
  use_rviz:=true start_controller:=true start_unitree_bridge:=true
```

Clear the area, keep the wireless controller ready, and arm motion explicitly:

```bash
ros2 service call /unitree_cmd_vel_bridge/enable std_srvs/srv/SetBool \
  "{data: true}"
```

Disarm and stop the robot with:

```bash
ros2 service call /unitree_cmd_vel_bridge/enable std_srvs/srv/SetBool \
  "{data: false}"
```

The launch defaults match this workspace:

- YOLO detections: `/yolo/detections_3d`
- robot pose: `/utlidar/robot_pose`
- detection frame: `camera_link`
- planning frame: `brne_odom`
- camera pose in `base_link`: `(0.34, 0.0, 0.08)` metres, tilted 30 degrees up

Every topic, frame, and camera translation above is a launch argument. For
example:

```bash
ros2 launch pedestrian_tracker brne_yolo.launch.py \
  detections_topic:=/different/detections \
  robot_pose_topic:=/different/pose \
  camera_x:=0.30 camera_y:=0.02 camera_z:=0.15 \
  camera_upward_pitch_deg:=25.0 heading_gain:=1.0
```
