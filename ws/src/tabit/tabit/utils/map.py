from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class Map:
    def __init__(
        self,
        node: Node,
        group=ReentrantCallbackGroup(),
        costmap_topic: str = "/global_costmap/costmap",
    ):
        self._node = node
        self._map_msg = None
        self._subscription = node.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.update,
            10,
            callback_group=group,
        )
        self._plan_pub = node.create_publisher(Path, "/our_plan", 10)

    def update(self, msg: OccupancyGrid):
        self._map_msg = msg

    def is_occupied(self, x: float, y: float) -> bool | None:
        if self._map_msg is None:
            return None

        # Convert map coordinates to grid indices
        mx, my = self.to_grid((x, y))

        # TODO [3]: convert x,y indices to a single index in the 1D data array
        # and return True if occupied (value > 50), False if free (value < 50),
        # and None if unknown (value == -1) or out of bounds
        return None

    def is_free(self, mx: int, my: int) -> bool:
        result = not self.is_occupied(mx, my)

        # consider unknown cells as free cells for path planning
        return True if result is None else result

    def to_grid(self, pt):
        """
        Converts world coordinates (x, y) in meters to grid indices (mx, my)."""
        map_info = self._map_msg.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        x, y = pt
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        return mx, my

    def to_world(self, mx, my):
        """
        Converts grid indices (mx, my) to world coordinates (x, y) in meters
        """
        map_info = self._map_msg.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        x = mx * resolution + origin_x
        y = my * resolution + origin_y
        return [x, y]

    def make_plan(
        self, start: list[float], goal: list[float]
    ) -> list[list[float]] | None:
        """
        Finds a path from start to goal using BFS. Returns a list of [x, y] points.
        start, goal: [x, y] in map coordinates (meters)
        """
        if self._map_msg is None:
            return None
        map_info = self._map_msg.info
        width, height = map_info.width, map_info.height

        start_idx = self.to_grid(start)  # (mx, my) of start point
        goal_idx = self.to_grid(goal)  # (mx, my) of goal point

        if not (0 <= start_idx[0] < width and 0 <= start_idx[1] < height):
            return None
        if not (0 <= goal_idx[0] < width and 0 <= goal_idx[1] < height):
            return None

        # TODO [4]: Implement BFS to find the shortest path from start_idx to goal_idx
        # You can use a queue (collections.deque) to explore the grid
        # and a set to keep track of visited nodes.
        # Return the path as a list of [x, y] points.
        return None

    def publish_path(self, path: Path):
        """
        Publishes the path on "our_plan" topic. Path is of type nav_msgs/Path.
        You can use the to_path_msg() function to convert a list of [x, y] points
        to a Path message.
        """
        self._plan_pub.publish(path)

    def to_path_msg(self, path):
        """
        Converts a list of [x, y] points to a nav_msgs/Path message.
        """
        if not path or self._map_msg is None:
            return None
        map_info = self._map_msg.info
        path_msg = Path()
        path_msg.header.stamp = self._node.get_clock().now().to_msg()
        path_msg.header.frame_id = (
            map_info.map_frame if hasattr(map_info, "map_frame") else "map"
        )
        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = self._node.get_clock().now().to_msg()
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg
