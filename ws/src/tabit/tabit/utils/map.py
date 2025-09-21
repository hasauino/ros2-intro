import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class Map:
    def __init__(self, node: Node, costmap_topic: str = "/global_costmap/costmap"):
        self._node = node
        self._map_msg = None
        self._subscription = node.create_subscription(
            OccupancyGrid, costmap_topic, self.update, 10
        )

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
        map_info = self._map_msg.info
        mx = int((x - map_info.origin.position.x) / map_info.resolution)
        my = int((y - map_info.origin.position.y) / map_info.resolution)

        if mx < 0 or my < 0 or mx >= map_info.width or my >= map_info.height:
            return None

        idx = my * map_info.width + mx
        value = self._map_msg.data[idx]
        if value == -1:
            return None
        return value > 50  # threshold for occupancy


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
