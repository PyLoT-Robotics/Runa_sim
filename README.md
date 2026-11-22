# Runa_sim

**Runa mobile-manipulator simulation for ROS 2 Jazzy / Gazebo Harmonic.**

Run a differential-drive base, movable camera head, five-axis arm, parallel gripper,
lift, RGB-D cameras, 2D LiDAR and IMU in the SOBITS RCJO2025 arena. Optional
SLAM Toolbox + Nav2 provide map-frame navigation and rotation actions.
The robot model lives in [Runa_description](https://github.com/PyLoT-Robotics/Runa_description).
This repository contains simulation code and a small public Python API; private
application and competition-task code are not required or included.

## Setup (Ubuntu 24.04 / Jazzy)

Install ROS 2 Jazzy and initialize rosdep first. Use system Python, not Conda.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-pytest \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-realsense2-camera-msgs ros-jazzy-cv-bridge
mkdir -p ~/runa_ws/src
cd ~/runa_ws/src
git clone https://github.com/PyLoT-Robotics/Runa_sim.git
cd ~/runa_ws
vcs import src < src/Runa_sim/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to runa_sim \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch runa_sim sim.launch.py
```

The dependencies file supplies the description, the **specified PyLoT SOBITS fork**,
and public `dynamixel_sdk_custom_interfaces` definitions. The latter is the PyLoT
fork's contract including `LiftControl`; an upstream package with the same name
may not contain it. Only message definitions are needed, not physical servo drivers.
ROS package names are lowercase even though GitHub repository names use capitals.
Do not run the physical robot's drivers in the simulation ROS domain.

New terminals must source both `/opt/ros/jazzy/setup.bash` and
`~/runa_ws/install/setup.bash`. For a server-only run:

```bash
ros2 launch runa_sim sim.launch.py gui:=false
```

Headless cameras / GPU LiDAR still require working EGL/OpenGL rendering. For CPU
machines try Mesa with `LIBGL_ALWAYS_SOFTWARE=1`; graphics-driver support matters.
`navigation:=false` disables SLAM/Nav2 for a smaller controller/sensor test.
Launch arguments also include `x` (−2 m), `y` (1 m), `yaw` (0 rad) and `world`
(default `rcjo2025_arena.world.xacro`).

## Field and assets

The field is loaded from
[PyLoT-Robotics/sobits_gazebo_worlds](https://github.com/PyLoT-Robotics/sobits_gazebo_worlds),
pinned in `dependencies.repos`. The fork's `humble-devel` branch contains SDF/model
assets; Runa_sim expands its world xacro and runs it with Harmonic directly, without
using its Humble-era install script or changing shell startup files.

Default `full_field:=false` retains the fork's floor, arena walls and all furniture
available inside that fork. Includes needing external WRS/AWS model repositories
are omitted **with their URIs logged**; they are not replaced by invented assets.
For the full upstream furniture placement:

```bash
cd ~/runa_ws
vcs import src < src/Runa_sim/full-field.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tmc_wrs_gz_worlds aws_small_house_world \
  --cmake-args -DBUILD_TESTING=OFF
colcon build --symlink-install --packages-up-to runa_sim \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch runa_sim sim.launch.py full_field:=true
```

Full-field launch checks model paths and fails clearly if required models are
missing. `GZ_SIM_RESOURCE_PATH` is extended only for the launched processes.

## ROS endpoints / existing Robot API integration

| API capability | Simulated endpoint |
|---|---|
| Base velocity | `/cmd_vel` (`geometry_msgs/msg/Twist`), feedback `/odom`, TF `odom → base_footprint` |
| Absolute navigation | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`), `map` frame |
| In-place rotation | `/spin` (`nav2_msgs/action/Spin`) |
| Arm poses / joint commands | `/arm_controller/joint_trajectory` and `/arm_controller/follow_joint_trajectory` |
| Gripper | `/gripper_controller/joint_trajectory` and `/gripper_controller/follow_joint_trajectory` |
| Joint feedback | `/joint_states` (arm, gripper, head and lift) |
| Pan / tilt | `/set_position`, `/present_position`, `/get_position`; public `SetPosition` / `GetPosition` types |
| Lift | `/lift_control` (`LiftControl`); legacy wire field `hight`, absolute height in mm, 0–180 |
| Head RGB-D | `/d455/camera/rgbd` (`realsense2_camera_msgs/msg/RGBD`) plus separate image/info topics |
| Wrist RGB-D | `/camera/d415_link/rgbd` plus separate image/info topics |
| Face display input | `/face` image relayed to `/sim/face` for viewers |
| Sensors | `/scan`, `/imu`, `/clock` |

This lets existing client APIs use standard motion and image transports without
exposing their implementation. To use your own Robot implementation, install its
normal message/runtime dependencies in your own workspace, disable physical drivers,
and source the simulation overlay. Simulation nodes all use simulated time;
pass `--ros-args -p use_sim_time:=true` to your client process.

This is **transport compatibility**, not a claim that every existing Robot method
has been validated. There are no fake success servers for speech recognition,
TTS, VLM/LLM planning, person finding/following, or motor-current push detection.
Those require real external software and are outside this simulation release.
There is no simulated Livox point cloud yet; `livox_frame` is available as TF only.

### Public Python example

A small independently implemented facade is supplied for common methods:

```python
import rclpy
from runa_sim.robot import Robot

rclpy.init()
robot = Robot()
try:
    print(robot.get_arm_pose(timeout=10))
    assert robot.set_arm_joints({'joint1': 0.2}, duration=2.0)
    assert robot.set_gripper(0.015)  # metres, not servo ticks
    assert robot.set_pantilt(pan=2100, tilt=1100)
    image = robot.get_latest_rgb_image(camera='d455')
    depth_mm = robot.get_latest_depth_image(camera='d415')
    # After SLAM has a map and Nav2 is active:
    pose = robot.wait(lambda: robot.robot_position, timeout=30)
    if pose:
        robot.go_to_abs([pose[0] + 0.35, pose[1], pose[2]])
finally:
    robot.destroy_node()
    rclpy.shutdown()
```

Run the example with `--ros-args -p use_sim_time:=true`. This facade is
`runa_sim.robot.Robot`; it does not install or shadow another package's Robot class.
Its arm/gripper setters wait for controller action completion; legacy topic-based
setters may return after publication. Pan/tilt publication and lift acceptance do
not imply completed movement—read feedback to confirm arrival.

## Model and sensor fidelity

- Primitive geometry, approximate dynamics; not a mesh/CAD-accurate digital twin.
- Wheel radius 0.05 m, track 0.39 m; base command cap 0.4 m/s and 1 rad/s,
  with a wall-clock velocity watchdog at 0.5 s.
- Head ticks: tilt ID 1, horizontal 1200; pan ID 2, forward 1900; 4096 ticks/rev.
  Tilt is clamped to 0–1200; pan respects the model's ±2.9 rad joint limits.
- Gripper stroke is 0–0.025 m per finger with a mimic joint. Object grasp stability
  and real servo current/contact behavior are not established.
- Cameras share RGB/depth geometry: RGB image plus `16UC1` millimetre depth,
  invalid depth 0. Native Gazebo `32FC1` metre depth is available under `/sim/*`.
- Arm, head and lift use real `gz_ros2_control` / trajectory controllers; joint
  feedback is measured simulated state, not an echo of requested targets.

## Validation

```bash
cd ~/runa_ws
python3 -m pytest -q src/Runa_description/test src/Runa_sim/test
# With sim.launch.py running (navigation enabled):
ros2 run runa_sim smoke_test --navigation --output smoke-results.json \
  --ros-args -p use_sim_time:=true
```

The smoke test checks actual base displacement/stopping, joint feedback, arm and
gripper actions, pan/tilt service feedback, lift feedback, both RGB-D streams,
Nav2 goal completion and Spin. It exits on failure and writes a JSON result.
The CI workflow builds in Jazzy and runs these checks with headless rendering.
See Actions for the latest actual result; static checks alone do not prove physics.

## License

[Apache-2.0](LICENSE) for simulation code. Nav2 configuration is adapted from
[Navigation2 (Jazzy)](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup/params)
under Apache-2.0. External worlds/models/messages retain their own licenses;
no world mesh or texture files are redistributed here.

## Acknowledgements

The field is provided by the
[PyLoT-Robotics fork of sobits_gazebo_worlds](https://github.com/PyLoT-Robotics/sobits_gazebo_worlds),
originally developed by [TeamSOBITS](https://github.com/TeamSOBITS/sobits_gazebo_worlds)
under **BSD-3-Clause**. Thank you to TeamSOBITS and the contributors for making
these RoboCup simulation environments available. Full-field furniture additionally
uses [tmc_wrs_gz](https://github.com/TeamSOBITS/tmc_wrs_gz) and
[aws_small_house_world](https://github.com/TeamSOBITS/aws_small_house_world);
consult their original asset licenses. Thanks also to the ROS 2, Gazebo,
ros2_control, SLAM Toolbox, Nav2 and Intel RealSense communities.
