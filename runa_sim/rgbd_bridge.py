"""Gazebo RGB-D streams -> RealSense-shaped topics, with explicit depth units."""
import sys
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD


def depth_mm(msg):
    if msg.encoding != '32FC1':
        raise ValueError('Gazebo depth must be 32FC1 metres')
    dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 4)
    values = rows[:, :msg.width]
    good = np.isfinite(values) & (values > 0) & (values < 8)
    mm = np.zeros(values.shape, dtype='<u2')
    mm[good] = np.clip(values[good] * 1000, 0, 65535).astype('<u2')
    out = Image()
    out.header = msg.header
    out.height, out.width = msg.height, msg.width
    out.encoding, out.step, out.is_bigendian = '16UC1', msg.width * 2, 0
    out.data = mm.tobytes()
    return out


class RgbdBridge(Node):
    def __init__(self):
        super().__init__('runa_rgbd_bridge')
        self.declare_parameter('camera', 'd455')
        cam = self.get_parameter('camera').value
        self.frame = f'{cam}_color_optical_frame'
        base = '/d455/camera' if cam == 'd455' else '/camera/d415_link'
        self._output_publishers = {name: self.create_publisher(kind, base + '/' + name, 10) for name, kind in [
            ('color/image_raw', Image), ('depth/image_rect_raw', Image),
            ('aligned_depth_to_color/image_raw', Image), ('color/camera_info', CameraInfo),
            ('depth/camera_info', CameraInfo), ('aligned_depth_to_color/camera_info', CameraInfo), ('rgbd', RGBD)]}
        self._input_subscribers = [message_filters.Subscriber(self, kind, f'/sim/{cam}/{suffix}', qos_profile=qos_profile_sensor_data)
                            for kind, suffix in [(Image, 'image'), (Image, 'depth_image'), (CameraInfo, 'camera_info')]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self._input_subscribers, 10, 0.05)
        self.sync.registerCallback(self.frames)

    def frames(self, color, depth, info):
        try:
            converted = depth_mm(depth)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=10)
            return
        color.header.frame_id = self.frame
        converted.header = color.header
        info.header = color.header
        for name, value in [('color/image_raw', color), ('depth/image_rect_raw', converted),
                            ('aligned_depth_to_color/image_raw', converted), ('color/camera_info', info),
                            ('depth/camera_info', info), ('aligned_depth_to_color/camera_info', info)]:
            self._output_publishers[name].publish(value)
        out = RGBD()
        out.header, out.rgb, out.depth = color.header, color, converted
        out.rgb_camera_info = info
        out.depth_camera_info = info
        self._output_publishers['rgbd'].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
