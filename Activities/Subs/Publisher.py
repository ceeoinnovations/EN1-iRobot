'''
this is the minimal publisher from the ROS2 tutorial
'''

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):  
        # initialize the topic (name it, create the publisher and publishing rate
        super().__init__('MyNode')
        queue_size = 10
        self.publisher_ = self.create_publisher(String, 'chatter', queue_size)  
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # every interval, create and publish a string
        msg = String()
        msg.data = 'Hi: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)  # prints to console / log
        self.i += 1
        #if self.i == 5:
        #    chatter.destroy_node()

def main():
    rclpy.init()
    chatter = MyPublisher()

    try:
        rclpy.spin(chatter)
        # stay here forever, publishing strings
    except Exception as e:
        print(e)
        chatter.destroy_node()
        rclpy.shutdown()
        print('Done')

if __name__ == '__main__':
    main()