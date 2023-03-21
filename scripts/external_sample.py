import rclpy
from rclpy.node import Node
import random

from crazyswarm_application.msg import UserCommand
from geometry_msgs.msg import Point

class ExternalPublisher(Node):

    def __init__(self):
        super().__init__('external_publisher')
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)
        # self.publisher_ = self.create_publisher(UserCommand, '/user', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.max = 5.0
        self.height = 1.0
    
    def random_generator(self):
        return ((random.random()*2) - 1) * self.max

    def timer_callback(self):
        msg = UserCommand()
        # string cmd
        # string[] uav_id
        # geometry_msgs/Point goal
        # float32 yaw
        msg.uav_id.append("cf1")
        # msg.cmd = "goto_velocity"
        msg.goal = Point()
        msg.goal.x = self.random_generator()
        msg.goal.y = self.random_generator()
        msg.goal.z = self.height

        print(msg.goal.x, msg.goal.y, msg.goal.z)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    external_publisher = ExternalPublisher()

    rclpy.spin(external_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    external_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()