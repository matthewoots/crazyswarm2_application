import rclpy
from rclpy.node import Node
import sys
import random
from functools import partial

from crazyswarm_application.msg import UserCommand
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from crazyswarm_application.srv import Agents

class ExternalPublisher(Node):

    def __init__(self):
        super().__init__('external_publisher')
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)
        self.timer_period = 3.0  # seconds
        self.create_service(Agents, "/external/receive", self.external_callback)
        self.subscriber = []
        self.max = 2.0
        self.height = 1.0
        self.agents = {}
    
    def external_callback(self, request, response):
        self.get_logger().info("[external] set subscribers")
        for name in request.names:
            self.agents.update({name: PoseStamped()})
            print('creating: ', name)
            self.subscriber.append(
                self.create_subscription(
                PoseStamped, 
                name + "/pose",
                partial(self.listener_callback, agent_id=name), 
                10))
        self.get_logger().info("[external] start timer")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        return response
    
    def listener_callback(self, msg, agent_id):
        self.agents[agent_id] = msg
        return
    
    def random_generator(self):
        return ((random.random()*2) - 1) * self.max

    def timer_callback(self):
        key, value = random.choice(list(self.agents.items()))
        # string cmd
        # string[] uav_id
        # geometry_msgs/Point goal
        # float32 yaw
        msg = UserCommand()
        msg.uav_id.append(key)
        msg.goal.x = value.pose.position.x + self.random_generator()
        msg.goal.y = value.pose.position.y + self.random_generator()
        msg.goal.z = self.height

        print(msg.uav_id, msg.goal.x, msg.goal.y, msg.goal.z)
        self.publisher_.publish(msg)


def main():
    if (len(sys.argv) > 1):
        print('too number of arguments', len(sys.argv))
        exit()
        
    rclpy.init(args=None)

    external_publisher = ExternalPublisher()

    rclpy.spin(external_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    external_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()