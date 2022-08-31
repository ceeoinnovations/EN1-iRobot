'''
this is the minimal subscriber from the ROS2 tutorial
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MySubscriber(Node):

    def __init__(self):
        super().__init__('myNode')
        self.subscription = self.create_subscription(String,'chatter',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    chatter = MySubscriber()

    try:
        print('spinning up chatter')
        rclpy.spin(chatter)
        # stay here forever, publishing strings
    except Exception as e:
        print(e)
        chatter.destroy_node()
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()
