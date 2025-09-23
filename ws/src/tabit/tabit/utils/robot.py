import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer

from tabit.utils.helpers import transform_stamped_to_pose_stamped


class Pose2D:
    def __init__(self, x, y, theta):
        self.position = {"x": x, "y": y}
        self.orientation = theta


class Robot:
    def __init__(
        self,
        node: Node,
        group,
        tf_buffer: Buffer,
        pull_distance: float = 0.2,
        max_speed: float = 0.5,
    ):
        self._node = node
        self._cmd_publisher = self._node.create_publisher(Twist, "cmd_vel", 10)
        self._pull_distance = pull_distance
        self._max_speed = max_speed
        self.tf_broadcaster = TransformBroadcaster(self._node)
        self.timer = self._node.create_timer(
            0.01, self.broadcast_timer_callback, callback_group=group
        )
        self.tf_buffer = tf_buffer

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
        omega = v_pull_y / self._pull_distance
        return self.move(vx, omega)

    def get_pose(self):
        try:
            # Try to lookup transform from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform(
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
            trans = self.tf_buffer.lookup_transform(
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
