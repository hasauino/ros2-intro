import rclpy
from geometry_msgs.msg import Twist, TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node("twist_stamped_to_twist")
    publisher = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    node.create_subscription(
        Twist,
        "/cmd_vel_twist_only",
        lambda msg: publisher.publish(TwistStamped(twist=msg)),
        10,
    )
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
