import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Pose2D:
    def __init__(self, x, y, theta):
        self.position = {"x": x, "y": y}
        self.orientation = theta


class Robot:
    def __init__(self, node: Node):
        self._node = node
        self._cmd_publisher = self._node.create_publisher(Twist, "cmd_vel", 10)

    def move(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_publisher.publish(msg)

    def get_pose(self):
        try:
            # Try to lookup transform from 'odom' to 'base_link'
            trans = self._node.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time(),
            )
            return Pose2D(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.rotation.z,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self._node.get_logger().warn(f"TF lookup failed: {e}")
            return None


def test_robot(navigator):
    robot = navigator.robot

    while rclpy.ok():
        v = 0.2  # Linear velocity in m/s
        r = 0.5  # Radius in m
        omega = v / r  # Angular velocity in rad/s
        robot.move(v, omega)
        pose = robot.get_pose()
        if pose:
            navigator.get_logger().info(
                f"Robot Pose - x: {pose.position['x']:.2f}, y: {pose.position['y']:.2f}, theta: {pose.orientation:.2f}"
            )
        rclpy.spin_once(navigator)
