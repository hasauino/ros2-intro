import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotCommander(Node):
    def __init__(self):
        super().__init__("robot_commander")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.msg = Twist()

    def timer_callback(self):
        # let the robot move in a circle with a radius of 50 cm, at a speed of 30 cm/sec
        radius = 0.5  # m
        velocity = 0.3  # m/s
        self.msg.linear.x = velocity
        self.msg.angular.z = velocity / radius
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    rclpy.spin(RobotCommander())


if __name__ == "__main__":
    main()
