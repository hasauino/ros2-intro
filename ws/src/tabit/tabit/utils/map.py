import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node


class Map:
    def __init__(self, node: Node, costmap_topic: str = "/global_costmap/costmap"):
        self._node = node
        self._map_msg = None
        self._subscription = node.create_subscription(
            OccupancyGrid, costmap_topic, self.update, 10
        )
        self._plan_pub = node.create_publisher(Path, "/plan", 10)

    def update(self, msg: OccupancyGrid):
        self._map_msg = msg

    def is_occupied(self, x: float, y: float, timeout=1.0):
        """
        Checks occupancy at (x, y) in odom frame.
        Transforms to map frame and checks occupancy value.
        Returns True if occupied, False if free, None if unknown.
        """
        if self._map_msg is None:
            return None

        # Convert map coordinates to grid indices
        mx, my = self.to_grid((x, y))

        if (
            mx < 0
            or my < 0
            or mx >= self._map_msg.info.width
            or my >= self._map_msg.info.height
        ):
            return None

        idx = my * self._map_msg.info.width + mx
        value = self._map_msg.data[idx]
        if value == -1:
            return None
        return value > 50  # threshold for occupancy

    def to_grid(self, pt):
        map_info = self._map_msg.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        x, y = pt
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        return mx, my

    def to_world(self, mx, my):
        map_info = self._map_msg.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        x = mx * resolution + origin_x
        y = my * resolution + origin_y
        return [x, y]

    def make_plan(self, start, goal):
        """
        Finds a path from start to goal using BFS. Returns a list of [x, y] points.
        start, goal: [x, y] in map coordinates (meters)
        """
        if self._map_msg is None:
            return None
        map_info = self._map_msg.info
        width, height = map_info.width, map_info.height

        start_idx = self.to_grid(start)
        goal_idx = self.to_grid(goal)

        if not (0 <= start_idx[0] < width and 0 <= start_idx[1] < height):
            return None
        if not (0 <= goal_idx[0] < width and 0 <= goal_idx[1] < height):
            return None

        def is_free(mx, my):
            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False
            idx = my * width + mx
            value = self._map_msg.data[idx]
            return value >= 0 and value < 50

        from collections import deque

        queue = deque()
        queue.append((start_idx, [start_idx]))
        visited = set()
        visited.add(start_idx)

        directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

        while queue:
            (mx, my), path = queue.popleft()
            if (mx, my) == goal_idx:
                return [self.to_world(x, y) for x, y in path]
            for dx, dy in directions:
                nx, ny = mx + dx, my + dy
                if (nx, ny) not in visited and is_free(nx, ny):
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
        return None

    def publish_path(self, path):
        """
        Publishes a nav_msgs/Path message from a list of [x, y] points.
        """
        if not path or self._map_msg is None:
            return
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
        self._plan_pub.publish(path_msg)


def test_map(navigator):
    navigator.get_logger().info("ready ..")

    def goal_callback(msg):
        x = msg.point.x
        y = msg.point.y
        navigator.get_logger().info(f"got goal {x:.2f}, {y:.2f} ")
        occupied = navigator.map.is_occupied(x, y, navigator.tf_buffer)
        if occupied is None:
            navigator.get_logger().info(f"Goal ({x:.2f}, {y:.2f}) occupancy unknown.")
        elif occupied:
            navigator.get_logger().info(f"Goal ({x:.2f}, {y:.2f}) is OCCUPIED.")
        else:
            navigator.get_logger().info(f"Goal ({x:.2f}, {y:.2f}) is FREE.")

    navigator.create_subscription(PointStamped, "/clicked_point", goal_callback, 10)
    rclpy.spin(navigator)
