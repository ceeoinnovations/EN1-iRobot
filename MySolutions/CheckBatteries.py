# cd /media/psf/Home/Documents/Work/Classes/2022\ 2Fall/EN1-iRobot/ClassNotebooks/EN1-iRobot
exit()
python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


creates = ['10.247.234.38','10.247.231.226','10.247.225.173','10.247.224.139','10.247.233.108','10.247.228.95','10.247.230.51','10.247.230.58','10.247.227.121','10.247.228.58','10.247.235.176','10.247.232.178','10.247.227.176','10.247.238.97','10.247.231.168','192.168.86.233','192.168.86.225']
creates = ['Syndrome','Dory','Nemo','Remy','EdnaMode','Woddy','Buzz','MikeWazowski','LightningMcQueen','Mater','Dash','ElastGirl','Pig','PotatoHead']

def get_topic_list():
    node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
    topic_list = node_dummy.get_topic_names_and_types()
    node_dummy.destroy_node()
    return topic_list

class BatterySubscriber(Node):
    def __init__(self, namespace: str = ""):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryState, namespace + '/battery_state', self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg: BatteryState):
        self.get_logger().info('I heard: "%s"' % msg)
        self.printBattery(msg)

    def printBattery(self, msg):
        print(namespace, " battery:", msg.percentage)

def main():
    rclpy.init()
    topic_list = get_topic_list()
    Creates_Cn = []
    for info in topic_list:
        Creates_Cn.append(info[0])
    print(Creates_Cn)

    for create in Creates_Cn:
        if create not in creates:
            break
        try:
            print('Checking ' + create)
            battery_subscriber = BatterySubscriber(create)
            print('spinning')
            rclpy.spin_once(battery_subscriber)
        except Exception as e:
            print(e)
        finally:
            print("Done")
            battery_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
