import rclpy
from tf2_ros import Buffer, TransformListener

from tabit.utils.map import Map
from tabit.utils.path_follower import PathFollower
from tabit.utils.robot import Robot


class Navigator:
    def __init__(self):
        self.node = rclpy.create_node("navigator")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self.node)
        self.robot = Robot(self.node)
        self.map = Map(self.node)
        self.wait_for_transform("odom", "base_link")
        self.wait_for_transform("map", "base_link")
        self.path_follower = PathFollower(self.node, self.robot, self.map, self.tf_buffer)
        self.node.get_logger().info("Navigator initialized 🚀")

    def wait_for_transform(self, target_frame, source_frame):
        self.node.get_logger().info(
            f"Waiting for transform {target_frame} -> {source_frame}"
        )
        while not self.tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
        ):
            rclpy.spin_once(self.node)

    def spin(self):
        rclpy.spin(self.node)

    def destroy(self):
        self.node.destroy_node()


def main():
    rclpy.init(args=None)
    navigator = Navigator()
    navigator.spin()
    navigator.destroy()


if __name__ == "__main__":
    main()
