"""Small public Robot facade for simulation; no private task-code dependency."""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition, LiftControl
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


class Robot(Node):
    """Blocking API with its own executor; call destroy_node() before shutdown."""
    def __init__(self):
        super().__init__('runa_sim_robot')
        self.joints = {}
        self.images = {}
        self.bridge = CvBridge()
        self.group = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(RGBD, '/d455/camera/rgbd', lambda msg: self.images.update(d455=msg), 10)
        self.create_subscription(RGBD, '/camera/d415_link/rgbd', lambda msg: self.images.update(d415=msg), 10)
        self.head_pub = self.create_publisher(SetPosition, '/set_position', 10)
        self.face_pub = self.create_publisher(Image, '/face', 10)
        self.controller_client = self.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=self.group)
        self.head_client = self.create_client(GetPosition, '/get_position', callback_group=self.group)
        self.lift_client = self.create_client(LiftControl, '/lift_control', callback_group=self.group)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.spin = ActionClient(self, Spin, '/spin', callback_group=self.group)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.group)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.group)
        self._spin_executor = MultiThreadedExecutor(num_threads=2)
        self._spin_executor.add_node(self)
        self.worker = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self.worker.start()

    def _joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        return None

    def _future(self, future, timeout):
        if not self.wait(future.done, timeout):
            return None
        return future.result()

    def _action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=min(timeout, 10.0)):
            return False
        handle = self._future(client.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            return False
        result = self._future(handle.get_result_async(), timeout)
        if result is None:
            self._future(handle.cancel_goal_async(), 2.0)
            return False
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def wait_ready(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        expected = {'arm_controller','gripper_controller','head_controller','lift_controller'}
        if not self.controller_client.wait_for_service(timeout_sec=min(timeout,5.0)):
            return False
        while time.monotonic() < deadline:
            result = self._future(self.controller_client.call_async(ListControllers.Request()), 2.0)
            if result is not None and expected <= {c.name for c in result.controller if c.state == 'active'}:
                return True
            time.sleep(0.1)
        return False

    @property
    def robot_position(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        except TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [t.translation.x, t.translation.y, yaw]

    def go_to_abs(self, goal, timeout=120.0):
        if len(goal) != 3 or not all(math.isfinite(float(v)) for v in goal):
            return False
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = map(float, goal[:2])
        request.pose.pose.orientation.z = math.sin(float(goal[2])/2)
        request.pose.pose.orientation.w = math.cos(float(goal[2])/2)
        return self._action(self.nav, request, timeout)

    def rotate_in_place(self, angle, timeout=30.0):
        if not math.isfinite(float(angle)):
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(seconds=timeout).to_msg()
        return self._action(self.spin, goal, timeout)

    def get_arm_pose(self, as_list=False, timeout=1.0):
        if not self.wait(lambda: all(n in self.joints for n in ARM_JOINTS), timeout):
            return None
        values = {n: self.joints[n] for n in ARM_JOINTS}
        return list(values.values()) if as_list else values

    def get_arm_joint(self, joint_name, timeout=1.0):
        if not self.wait(lambda: joint_name in self.joints, timeout):
            return None
        return self.joints[joint_name]

    def get_gripper_position(self, timeout=1.0):
        return self.get_arm_joint('gripper_left_joint', timeout)

    def _trajectory(self, client, names, positions, duration):
        if not names or len(names) != len(positions) or not math.isfinite(duration) or duration <= 0 or not all(math.isfinite(float(p)) for p in positions):
            return False
        if not self.wait_ready(timeout=30.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(map(float, positions))
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        return self._action(client, goal, duration + 10)

    def set_arm_pose(self, positions, duration=2.0):
        return self._trajectory(self.arm_client, ARM_JOINTS, positions, duration)

    def set_arm_joints(self, joint_angles, duration=2.0, fill_with_current=True):
        if not joint_angles or any(n not in ARM_JOINTS for n in joint_angles):
            return False
        target = self.get_arm_pose() if fill_with_current else {}
        if target is None:
            return False
        target.update(joint_angles)
        return self._trajectory(self.arm_client, list(target), list(target.values()), duration)

    def set_gripper(self, position, duration=1.0):
        if not 0 <= position <= 0.015:
            return False
        return self._trajectory(self.gripper_client, ['gripper_left_joint'], [position], duration)

    def set_pantilt(self, tilt=None, pan=None):
        if not self.wait(lambda: self.head_pub.get_subscription_count() > 0):
            return False
        for servo_id, value in [(1, tilt), (2, pan)]:
            if value is not None:
                msg = SetPosition()
                msg.id, msg.position = servo_id, int(value)
                self.head_pub.publish(msg)
        return True

    def pantilt_home(self):
        return self.set_pantilt(tilt=1200, pan=1900)

    def get_pantilt_position(self, servo_id):
        if not self.head_client.wait_for_service(timeout_sec=5.0):
            return None
        request = GetPosition.Request()
        request.id = servo_id
        result = self._future(self.head_client.call_async(request), 5.0)
        return result.position if result is not None and result.position >= 0 else None

    def lift_control(self, height):
        if not 0 <= height <= 180 or not self.lift_client.wait_for_service(timeout_sec=5.0):
            return False
        request = LiftControl.Request()
        request.hight = int(height)
        result = self._future(self.lift_client.call_async(request), 5.0)
        return result is not None and result.success

    def get_latest_rgb_image(self, timeout=5.0, camera='d455'):
        if camera not in ('d455', 'd415'):
            raise ValueError('camera must be d455 or d415')
        if not self.wait(lambda: camera in self.images, timeout):
            return None
        return self.bridge.imgmsg_to_cv2(self.images[camera].rgb, desired_encoding='bgr8')

    def get_latest_depth_image(self, timeout=5.0, camera='d455'):
        if camera not in ('d455', 'd415'):
            raise ValueError('camera must be d455 or d415')
        if not self.wait(lambda: camera in self.images, timeout):
            return None
        return self.bridge.imgmsg_to_cv2(self.images[camera].depth, desired_encoding='passthrough')

    def display_img(self, image):
        self.face_pub.publish(self.bridge.cv2_to_imgmsg(np.asarray(image), encoding='bgr8'))
        return True

    def destroy_node(self):
        self._spin_executor.shutdown(timeout_sec=2.0)
        self.worker.join(timeout=2.0)
        return super().destroy_node()
