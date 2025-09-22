import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tabit.utils.helpers import transform_stamped_to_pose_stamped


class Pose2D:
    def __init__(self, x, y, theta):
        self.position = {"x": x, "y": y}
        self.orientation = theta


class Robot:
    def __init__(self, node: Node, pull_distance: float = 0.2):
        self._node = node
        self._cmd_publisher = self._node.create_publisher(Twist, "cmd_vel", 10)
        self._pull_distance = pull_distance
        self.tf_broadcaster = TransformBroadcaster(self._node)
        self.timer = self._node.create_timer(0.01, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "pull_point"
        t.transform.translation.x = self._pull_distance
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def move(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_publisher.publish(msg)

    def move_from_pull_point(self, v_pull_x: float, v_pull_y: float):
        vx = v_pull_x
        omega = v_pull_y / self._pull_distance
        self.move(vx, omega)

    def get_pose(self):
        try:
            # Try to lookup transform from 'odom' to 'base_link'
            trans = self._node.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time(),
            )
            return transform_stamped_to_pose_stamped(trans)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self._node.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def get_pull_point(self):
        try:
            # Lookup transform from 'odom' to 'pull_point'
            trans = self._node.tf_buffer.lookup_transform(
                "odom",
                "pull_point",
                rclpy.time.Time(),
            )
            return transform_stamped_to_pose_stamped(trans)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self._node.get_logger().warn(f"TF lookup failed for pull_point: {e}")
            return None

    def __repr__(self):
        # return info about x,y location and vx and omega
        pose = self.get_pose()
        if pose:
            return f"Robot Pose - x: {pose.position['x']:.2f}, y: {pose.position['y']:.2f}, theta: {pose.orientation:.2f}"
        else:
            return "Robot Pose - Unknown"


def test_robot(navigator):
    robot = navigator.robot

    while rclpy.ok():
        v = 0.2  # Linear velocity in m/s
        r = 0.5  # Radius in m
        omega = v / r  # Angular velocity in rad/s
        robot.move(v, omega)
        navigator.get_logger().info(str(robot))
        rclpy.spin_once(navigator)
