import rclpy
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener

from tabit.utils.map import Map
from tabit.utils.path_follower import PathFollower
from tabit.utils.robot import Robot
from rclpy.callback_groups import ReentrantCallbackGroup


class Navigator:
    def __init__(self):
        self.node = rclpy.create_node("navigator")
        self.group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self.node)
        self.robot = Robot(self.node, self.group, self.tf_buffer)
        self.map = Map(self.node, group=self.group)
        self.wait_for_transform("odom", "base_link")
        self.wait_for_transform("map", "base_link")
        self.path_follower = PathFollower(
            self.node, self.group, self.robot, self.map, self.tf_buffer
        )
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
    executor = MultiThreadedExecutor()
    executor.add_node(navigator.node)
    executor.spin()
    navigator.destroy()


if __name__ == "__main__":
    main()
