from time import sleep

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("my_node")
    publisher = node.create_publisher(String, "my_topic", 10)
    msg = String(data="hello there")

    while rclpy.ok():
        publisher.publish(msg)
        sleep(1.0)


if __name__ == "__main__":
    main()
