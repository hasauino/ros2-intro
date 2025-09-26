import rclpy
from geometry_msgs.msg import PointStamped


class Exercise:
    def __init__(self):
        self.node = rclpy.create_node("exercise")
        # TODO [1]: create a subscriber to the /clicked_point topic

    def callback(self, msg: PointStamped):
        # TODO [2]: log the coordinates of the clicked point
        # self.get_logger().info(f"Clicked point: x={...}, y={...}")
        pass

    @property
    def logger(self):
        return self.node.get_logger()

    def destroy(self):
        self.node.destroy_node()

    def spin_once(self):
        rclpy.spin_once(self.node)

    def spin(self):
        rclpy.spin(self.node)


def main():
    rclpy.init()
    exercise = Exercise()
    exercise.logger.info("Exercise started")
    exercise.spin()
    exercise.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
