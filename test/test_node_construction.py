"""Catch ROS Node property collisions before running expensive physics tests."""
import rclpy
from runa_sim.compatibility import Compatibility
from runa_sim.rgbd_bridge import RgbdBridge
from runa_sim.robot import Robot


def test_nodes_construct_without_servers_or_hardware():
    rclpy.init(args=[])
    nodes = []
    try:
        nodes.append(Compatibility())
        nodes.append(RgbdBridge())
        robot = Robot()
        nodes.append(robot)
        assert robot.worker.is_alive()
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
