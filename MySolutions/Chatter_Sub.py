# cd /media/psf/Home/Documents/Work/Classes/2022\ 2Fall/EN1-iRobot/ClassNotebooks/EN1-iRobot
# ros2 topic pub /chatter std_msgs/String "data: Hello world" 
exit()
python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('SubscriberNode')
        self.subscription = self.create_subscription(String,'chatter',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    chatter = MinimalSubscriber()

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