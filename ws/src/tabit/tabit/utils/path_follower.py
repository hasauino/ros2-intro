import numpy as np
import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration


class PathFollower:
    def __init__(self, node, group, robot, map, tf_buffer):
        self.node = node
        self.robot = robot
        self.map = map
        self.tf_buffer = tf_buffer
        self.node.create_subscription(
            PointStamped,
            "/clicked_point",
            self.callback,
            10,
            callback_group=group,
        )
        self.map_to_odom = None
        self.proportional_gain = 1.0
        self.tolerance = 0.05
        self.logger = self.node.get_logger()

    def callback(self, msg):
        start = self.robot.get_pull_point()
        start.header.stamp = rclpy.time.Time()
        start = self.tf_buffer.transform(start, "map", timeout=Duration(seconds=5.0))
        start = [start.pose.position.x, start.pose.position.y]
        goal = [msg.point.x, msg.point.y]
        path = self.map.make_plan(start, goal)
        if path is None:
            self.logger.info("No path found 😞")
            return
        path = self.map.to_path_msg(path)
        path = self.transform_path_to(path, "odom")
        self.map.publish_path(path)
        self.follow(path)

    def follow(self, path: Path):
        def is_last_pose(index):
            return index == len(path.poses) - 1

        goal = path.poses[-1]
        pose_index = 0
        has_reached_goal = False
        while not has_reached_goal and rclpy.ok():
            rclpy.spin_once(self.node)
            pose = path.poses[pose_index]
            ex, ey = self.get_error(pose)
            if np.linalg.norm(self.get_error(goal)) < self.tolerance or is_last_pose(
                pose_index
            ):
                has_reached_goal = True
                self.logger.info("Goal reached 🎉")
                break
            self.robot.move_from_pull_point(
                self.proportional_gain * ex, self.proportional_gain * ey
            )
            if np.linalg.norm([ex, ey]) < self.robot.pull_distance:
                pose_index += 1
        self.robot.move(0.0, 0.0)

    def is_goal_reached(self, goal: PoseStamped):
        return np.linalg.norm(self.get_error(goal)) < self.tolerance

    @staticmethod
    def norm(point_2d):
        x, y = point_2d
        return (x**2 + y**2) ** 0.5

    def get_error(self, setpoint_odom: PoseStamped):
        robot_position = np.array([self.robot.x, self.robot.y])
        setpoint_position = np.array(
            [setpoint_odom.pose.position.x, setpoint_odom.pose.position.y]
        )

        # error vector represented in odom frame
        error_odom = setpoint_position - robot_position

        # error vector in robot base frame
        error_base = self.robot.base_R_odom.dot(error_odom)
        return error_base

    def transform_path_to(self, path: Path, target_frame: str) -> Path:
        path_odom = Path()
        path_odom.header.stamp = path.header.stamp
        path_odom.header.frame_id = target_frame
        for pose in path.poses:
            pose_odom = self.tf_buffer.transform(
                pose,
                target_frame,
                timeout=Duration(seconds=1.0),
            )
            path_odom.poses.append(pose_odom)
        return path_odom
