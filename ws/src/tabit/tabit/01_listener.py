import rclpy
from std_msgs.msg import String


def callabck(msg):
    print(f"I got this msg: {msg.data}")


def main():
    rclpy.init()
    node = rclpy.create_node("listener")
    node.create_subscription(String, "my_topic", callabck, 10)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
