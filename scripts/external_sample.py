import rclpy
from rclpy.node import Node
import sys
import random

from crazyswarm_application.msg import UserCommand
from geometry_msgs.msg import Point
from std_srvs.srv import Empty

class ExternalPublisher(Node):

    def __init__(self, agent_list):
        super().__init__('external_publisher')
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)
        self.timer_period = 3.0  # seconds
        self.create_service(Empty, "/external/receive", self.external_callback)
        self.i = 0
        self.max = 5.0
        self.height = 1.0
        self.agents = agent_list
    
    def external_callback(self, request, response):
        self.get_logger().info("start timer")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        return response
    
    def random_generator(self):
        return ((random.random()*2) - 1) * self.max

    def timer_callback(self):
        idx = random.randint(0, len(self.agents)-1)
        # string cmd
        # string[] uav_id
        # geometry_msgs/Point goal
        # float32 yaw
        msg = self.agents.get(idx)
        msg.goal.x = self.random_generator()
        msg.goal.y = self.random_generator()
        msg.goal.z = self.height

        print(msg.uav_id, msg.goal.x, msg.goal.y, msg.goal.z)
        self.publisher_.publish(msg)


def main():
    
    if (len(sys.argv) == 1):
        print('invalid number of arguments', len(sys.argv))
        exit()
    agent_list = {}
    count = 0
    for arg in sys.argv[1:]:
        cmd = UserCommand()
        cmd.uav_id.append("cf" + str(arg))
        agent_list[count] = cmd
        print('... append: ', "cf" + str(arg))
        count += 1
    rclpy.init(args=None)

    external_publisher = ExternalPublisher(agent_list)

    rclpy.spin(external_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    external_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()