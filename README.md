# RealSense-YOLO-BRNE Navigation for Unitree Go2

ROS 2 Jazzy workspace for physical pedestrian-aware navigation on a Unitree
Go2. An Intel RealSense camera provides aligned RGB-D images, YOLOv8 Pose and
ByteTrack produce tracked 3D pedestrians, and BRNE generates an avoidance
trajectory toward an RViz goal.

This repository contains the locally modified source for:

- `yolo_ros`: RGB-D human detection and ByteTrack tracking;
- `pedestrian_tracker`: timestamp-aware camera-to-odometry transformation,
  stationary-person filtering, robot odometry, and the Unitree command bridge;
- `brne_social_nav`: BRNE sampling, goal-directed control, diagnostics, and
  RViz visualization.

## System pipeline

```text
RealSense RGB + aligned depth
            |
            v
YOLOv8 Pose + ByteTrack --> /yolo/detections_3d
            |
            v
pedestrian_state_node --> /pedestrians (brne_odom)
Unitree SportModeState --> /brne_odom and TF
            |
            v
BRNE --> /cmd_buf --> controller --> /cmd_vel
            |
            v
Unitree Sport API bridge --> /api/sport/request
```

Pedestrian measurements are transformed into `brne_odom` at their observation
time. The transformation compensates both robot translation and rotation so a
stationary pedestrian does not acquire false velocity when the robot moves.

## Requirements

- Ubuntu with ROS 2 Jazzy;
- Intel RealSense and `realsense2_camera`;
- the Unitree ROS 2 messages (`unitree_go`, `unitree_api`);
- Cyclone DDS (`rmw_cyclonedds_cpp`);
- Python dependencies required by `yolo_ros`, including Ultralytics;
- Armadillo and OpenMP for BRNE.

The Unitree message workspace is external to this repository and must be
sourced before building and running.

## Build

```bash
source /opt/ros/jazzy/setup.bash
source /home/user/go2/install/setup.bash
cd /home/user/pedestrian
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "zed_interfaces zed_components"
colcon build --symlink-install --packages-select \
  yolo_msgs yolo_ros yolo_bringup \
  crowd_nav_interfaces brnelib crowd_nav pedestrian_tracker
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The legacy upstream `pedestrian_tracking` package targets a ZED camera and is
not part of this RealSense pipeline, so it is intentionally excluded from the
build command.

## Run the physical stack

Use three terminals. Source ROS 2, the Unitree message workspace, and this
workspace in every terminal.

Terminal 1 - RealSense:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true enable_sync:=false \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30
```

Terminal 2 - YOLOv8 Pose and ByteTrack:

```bash
ros2 launch yolo_bringup yolov8.launch.py \
  device:=cpu model:=yolov8n-pose.pt \
  input_image_topic:=/camera/camera/color/image_raw \
  input_depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  input_depth_info_topic:=/camera/camera/aligned_depth_to_color/camera_info \
  image_reliability:=2 depth_image_reliability:=2 depth_info_reliability:=2 \
  use_tracking:=True use_3d:=True use_debug:=True \
  target_frame:=camera_link
```

Terminal 3 - pedestrian adapter, BRNE, RViz, controller, and Go2 bridge:

```bash
ros2 launch pedestrian_tracker brne_yolo.launch.py \
  use_rviz:=true start_controller:=true start_unitree_bridge:=true \
  use_walls:=false use_corridor_mask:=false \
  publish_robot_samples:=true publish_pedestrian_samples:=true \
  use_sport_state_pose:=true \
  camera_x:=0.34 camera_y:=0.0 camera_z:=0.08 \
  camera_upward_pitch_deg:=30.0 z_smoothing:=0.0 heading_gain:=1.0 \
  unitree_auto_arm_on_goal:=true \
  unitree_max_linear_x:=0.20 unitree_max_angular_z:=0.40
```

The bridge starts disarmed and, with `unitree_auto_arm_on_goal:=true`, arms only
after a new RViz goal and valid robot-state safety checks. To stop and disarm:

```bash
ros2 service call /unitree_cmd_vel_bridge/enable std_srvs/srv/SetBool \
  "{data: false}"
```

## Important frames and topics

| Purpose | Name |
|---|---|
| Robot state | `/lf/sportmodestate` |
| Planning odometry | `/brne_odom` |
| Planning TF | `brne_odom -> base_link -> camera_link` |
| Camera mount | `x=0.34 m`, `y=0.0 m`, `z=0.08 m`, `30 deg` upward pitch |
| 3D detections | `/yolo/detections_3d` |
| BRNE pedestrians | `/pedestrians` |
| RViz goal | `/goal_pose` |
| BRNE command buffer | `/cmd_buf` |
| Velocity command | `/cmd_vel` |
| Unitree API request | `/api/sport/request` |

## Safety

This software controls a physical quadruped. Begin with the bridge disarmed,
verify perception and TF in RViz, keep a separate safety operator with immediate
stop access, and use a clear indoor test area. Never infer physical safety from
visualization or a small number of trials.

## Upstream projects and licensing

The workspace includes modified copies of
[`brne_social_nav`](https://github.com/katie-hughes/brne_social_nav) and
[`yolo_ros`](https://github.com/mgonzs13/yolo_ros). Their original notices and
licenses remain in their source directories. In particular, `yolo_ros` is
distributed under GPL-3.0; review all component licenses before redistribution.

