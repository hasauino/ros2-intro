import rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path


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
        start = self.tf_buffer.transform(start, "map")
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
        goal = path.poses[-1]
        pose_index = 0
        while not self.is_goal_reached(goal) or pose_index < len(path.poses):
            pose = path.poses[pose_index]
            ex, ey = self.get_error(pose)
            self.logger.info(f"error [{ex}, {ey}]")
            self.robot.move_from_pull_point(
                self.proportional_gain * ex, self.proportional_gain * ey
            )
            if self.norm([ex, ey]) < 0.5:
                pose_index += 1
        self.robot.move(0.0, 0.0)

    def is_goal_reached(self, goal: PoseStamped):
        return self.norm(self.get_error(goal)) < self.tolerance

    @staticmethod
    def norm(point_2d):
        x, y = point_2d
        return (x**2 + y**2) ** 0.5

    def get_error(self, setpoint: PoseStamped):
        setpoint.header.stamp = rclpy.time.Time()
        trans = self.tf_buffer.transform(setpoint, "pull_point")
        return trans.pose.position.x, trans.pose.position.y

    def transform_path_to(self, path: Path, target_frame: str) -> Path:
        path_odom = Path()
        path_odom.header.stamp = path.header.stamp
        path_odom.header.frame_id = target_frame
        for pose in path.poses:
            pose_odom = self.tf_buffer.transform(pose, target_frame)
            path_odom.poses.append(pose_odom)
        return path_odom
