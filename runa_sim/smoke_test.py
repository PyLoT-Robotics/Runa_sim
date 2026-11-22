"""Exercise real Gazebo feedback, controllers, cameras and optional Nav2."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from runa_sim.robot import Robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--output', default='smoke-results.json')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    robot = Robot()
    results = {}
    odom = []
    scans = []
    robot.create_subscription(Odometry, '/odom', lambda msg: odom.append(msg), 10)
    robot.create_subscription(LaserScan, '/scan', lambda msg: scans.append(msg), 10)
    velocity = robot.create_publisher(Twist, '/cmd_vel', 10)
    try:
        assert robot.wait(lambda: odom and scans and robot.get_arm_pose(timeout=0.01), 120), 'No odometry/scan/joint feedback'
        results['feedback'] = True
        x, y = odom[-1].pose.pose.position.x, odom[-1].pose.pose.position.y
        command = Twist()
        command.linear.x = 0.1
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            velocity.publish(command)
            time.sleep(0.05)
        # Stop publishing: the compatibility watchdog must stop the base.
        time.sleep(1)
        distance = math.hypot(odom[-1].pose.pose.position.x-x, odom[-1].pose.pose.position.y-y)
        assert distance > 0.04, f'Base did not move: {distance}'
        results['base_distance_m'] = distance
        before = (odom[-1].pose.pose.position.x, odom[-1].pose.pose.position.y)
        time.sleep(1)
        drift = math.hypot(odom[-1].pose.pose.position.x-before[0], odom[-1].pose.pose.position.y-before[1])
        assert drift < 0.03, f'Base failed to stop: {drift}'
        results['base_stop_drift_m'] = drift
        assert robot.set_arm_joints({'joint1': 0.2}, duration=1.0), 'Arm action failed'
        assert abs(robot.get_arm_joint('joint1')-0.2) < 0.05, 'Arm feedback mismatch'
        results['arm'] = True
        assert robot.set_gripper(0.015), 'Gripper action failed'
        assert abs(robot.get_gripper_position()-0.015) < 0.005, 'Gripper feedback mismatch'
        results['gripper'] = True
        assert robot.set_pantilt(pan=2100, tilt=1100), 'Pan/tilt publish failed'
        assert robot.wait(lambda: robot.get_pantilt_position(2) is not None and abs(robot.get_pantilt_position(2)-2100) < 30, 10), 'Pan did not move'
        results['pantilt'] = True
        assert robot.lift_control(30), 'Lift command rejected'
        assert robot.wait(lambda: abs(robot.joints.get('lift_joint', -1)-0.03) < 0.005, 10), 'Lift did not move'
        results['lift'] = True
        for camera in ['d455', 'd415']:
            image = robot.get_latest_rgb_image(timeout=30, camera=camera)
            depth = robot.get_latest_depth_image(timeout=30, camera=camera)
            assert image is not None and depth is not None and image.shape[:2] == depth.shape, f'No aligned RGBD: {camera}'
            results[camera] = {'rgb_shape': list(image.shape), 'depth_dtype': str(depth.dtype)}
        if args.navigation:
            assert robot.wait(lambda: robot.robot_position, 90), 'No map -> base transform'
            pose = robot.robot_position
            assert robot.go_to_abs([pose[0] + 0.35, pose[1], pose[2]], timeout=90), 'Nav2 goal failed'
            assert robot.rotate_in_place(0.25, timeout=30), 'Nav2 spin failed'
            results['navigation'] = True
        results['success'] = True
    finally:
        velocity.publish(Twist())
        Path(args.output).write_text(json.dumps(results, indent=2))
        robot.destroy_node()
        rclpy.shutdown()
    print(json.dumps(results, indent=2))
