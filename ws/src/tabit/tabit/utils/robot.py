import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from time import sleep

from tabit.utils.helpers import odom_to_pose_stamped


class Robot:
    def __init__(
        self,
        node: Node,
        group,
        pull_distance: float = 0.2,
        max_speed: float = 0.5,
    ):
        self.node = node
        self._cmd_publisher = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.pull_distance = pull_distance
        self._max_speed = max_speed

        # Odometry subscriber and storage variables
        self.odom = None
        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=group,
        )

    def wait_until_ready(self):
        self.logger.info("Waiting for initial odometry...")
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self.node)
        self.logger.info("Initial odometry received.")

    def move(self, linear: float, angular: float):
        max_reached = False
        msg = Twist()
        msg.linear.x = (
            min(linear, self._max_speed)
            if linear >= 0
            else max(linear, -self._max_speed)
        )
        if abs(msg.linear.x) >= self._max_speed:
            max_reached = True
        msg.angular.z = min(angular, 0.3) if angular >= 0 else max(angular, -0.3)
        self._cmd_publisher.publish(msg)
        return max_reached

    def move_from_pull_point(self, v_pull_x: float, v_pull_y: float):
        vx = v_pull_x
        omega = v_pull_y / self.pull_distance
        return self.move(vx, omega)

    def get_pose(self):
        if self.odom is None:
            return None
        return odom_to_pose_stamped(self.odom)

    def get_pull_point(self):
        if self.odom is None:
            return None
        return odom_to_pose_stamped(self.odom)

    @property
    def odom_R_base(self):
        """
        2x2 rotation matrix that transforms a 2D position vector from base to odom.
        """
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        # Rotation matrix from base to odom
        return [
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta],
        ]

    @property
    def base_R_odom(self):
        """
        2x2 rotation matrix that transforms a 2D position vector from base to odom.
        """
        return np.transpose(self.odom_R_base)

    @property
    def x(self):
        if self.odom is None:
            return None
        return self.odom.pose.pose.position.x

    @property
    def y(self):
        if self.odom is None:
            return None
        return self.odom.pose.pose.position.y

    @property
    def theta(self):
        if self.odom is None:
            return None
        orientation_q = self.odom.pose.pose.orientation
        # Convert quaternion to yaw angle (rotation around z-axis)
        siny_cosp = 2 * (
            orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y
        )
        cosy_cosp = 1 - 2 * (
            orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z
        )
        return np.arctan2(siny_cosp, cosy_cosp)

    @property
    def logger(self):
        return self.node.get_logger()

    def odom_callback(self, msg):
        self.odom = msg

    def __repr__(self):
        return f"Robot Pose: x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}"


def test_robot(navigator):
    robot = navigator.robot

    while rclpy.ok():
        v = 0.2  # Linear velocity in m/s
        r = 0.5  # Radius in m
        omega = v / r  # Angular velocity in rad/s
        robot.move(v, omega)
        navigator.node.get_logger().info(str(robot))
        navigator.spin_once()
