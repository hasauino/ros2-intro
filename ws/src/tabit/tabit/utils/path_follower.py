from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path


class PathFollower:
    def __init__(self, node, robot, map, tf_buffer):
        self.node = node
        self.robot = robot
        self.map = map
        self.tf_buffer = tf_buffer
        self.node.create_subscription(PointStamped, "/clicked_point", self.callback, 10)
        self.map_to_odom = None

    def callback(self, msg):
        start = self.robot.get_pull_point()
        start = self.tf_buffer.transform(start, "map")
        start = [start.pose.position.x, start.pose.position.y]
        goal = [msg.point.x, msg.point.y]
        path = self.map.make_plan(start, goal)
        if path is None:
            self.node.get_logger().info("No path found 😞")
            return
        path = self.map.to_path_msg(path)
        self.follow(path)

    def follow(self, path: Path):
        pass
