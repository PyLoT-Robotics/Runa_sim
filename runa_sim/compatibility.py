"""Hardware-shaped ROS endpoints backed by simulated controllers and feedback."""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition, LiftControl


from .kinematics import tick_to_angle, angle_to_tick


class Compatibility(Node):
    def __init__(self):
        super().__init__('runa_compatibility')
        self.positions = {}
        self.last_velocity = Twist()
        self.velocity_received = None
        self.head_targets = {'base_to_pan_joint': 0.0, 'pan_to_tilt_joint': 0.0}
        self.velocity_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.lift_pub = self.create_publisher(JointTrajectory, '/lift_controller/joint_trajectory', 10)
        self.present_pub = self.create_publisher(SetPosition, '/present_position', 10)
        self.face_pub = self.create_publisher(Image, '/sim/face', 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(SetPosition, '/set_position', self.head, 10)
        self.create_subscription(Image, '/face', self.face_pub.publish, 10)
        self.create_service(GetPosition, '/get_position', self.get_position)
        self.create_service(LiftControl, '/lift_control', self.lift)
        self.create_timer(0.05, self.tick)

    def velocity(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            msg = Twist()
        out = Twist()
        out.linear.x = max(-0.4, min(0.4, msg.linear.x))
        out.angular.z = max(-1.0, min(1.0, msg.angular.z))
        self.last_velocity = out
        self.velocity_received = time.monotonic()

    def joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))
        for servo_id, name in [(1, 'pan_to_tilt_joint'), (2, 'base_to_pan_joint')]:
            if name in self.positions:
                out = SetPosition()
                out.id = servo_id
                out.position = angle_to_tick(servo_id, self.positions[name])
                self.present_pub.publish(out)

    def send_trajectory(self, publisher, names, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points = [p]
        publisher.publish(msg)

    def head(self, msg):
        if msg.id not in (1, 2):
            return
        name = 'pan_to_tilt_joint' if msg.id == 1 else 'base_to_pan_joint'
        self.head_targets[name] = tick_to_angle(msg.id, msg.position)
        self.send_trajectory(self.head_pub, list(self.head_targets), list(self.head_targets.values()), 0.5)

    def get_position(self, request, response):
        name = {1: 'pan_to_tilt_joint', 2: 'base_to_pan_joint'}.get(request.id)
        response.position = angle_to_tick(request.id, self.positions[name]) if name in self.positions else -1
        return response

    def lift(self, request, response):
        # Public wire contract uses the legacy spelling 'hight', in millimetres.
        target = request.hight / 1000.0
        response.success = target is not None and math.isfinite(target) and 0 <= target <= 0.18
        if response.success:
            self.send_trajectory(self.lift_pub, ['lift_joint'], [target], 2.0)
        return response

    def tick(self):
        stale = self.velocity_received is None or time.monotonic() - self.velocity_received > 0.5
        self.velocity_pub.publish(Twist() if stale else self.last_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = Compatibility()
    try:
        rclpy.spin(node)
    finally:
        node.velocity_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
