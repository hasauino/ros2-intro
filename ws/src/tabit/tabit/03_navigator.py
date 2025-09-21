import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from tabit.utils.map import Map, test_map
from tabit.utils.robot import Robot, test_robot


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)
        self.robot = Robot(self)
        self.map = Map(self)
        self.wait_for_transform("odom", "base_link")
        self.wait_for_transform("map", "base_link")

    def wait_for_transform(self, target_frame, source_frame):
        self.get_logger().info(
            f"Waiting for transform {target_frame} -> {source_frame}"
        )
        while not self.tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
        ):
            rclpy.spin_once(self)


def main():
    rclpy.init(args=None)
    navigator = Navigator()
    test_map(navigator)
    # test_robot(navigator)


if __name__ == "__main__":
    main()
