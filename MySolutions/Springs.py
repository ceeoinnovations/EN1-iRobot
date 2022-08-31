# https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Projects/Invisible_Springs/invisible_springs.py

import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import IrIntensity

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

namespace = '/rogers'
max_speed = 4
threshold = 1000

class Springs(Node):  # trigger an update ever time you read the IR sensor
    def __init__(self):
        super().__init__('springs')
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.listener_callback, qos_profile_sensor_data)
        self.ir = IrIntensity()
        
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

    def listener_callback(self, msg:IrIntensityVector):
        val = []
        message = msg.readings
        for i in range(len(message)):
            val.append(message[i].value)

        if 0 < max(val) < threshold:
            speed = max_speed/max(val)
        elif max(val) >= threshold:
            speed = 0.0
        else: 
            speed = max_speed 
        
        self.linear.x = self.linear.y = self.linear.z = float(speed)
        self.angular.x = self.angular.y = self.angular.z = 0.0
        
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.wheels_publisher.publish(self.wheels)

def main(args=None):
    rclpy.init(args=args)
    springs = Springs()
    try:
        rclpy.spin(springs)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
        rclpy.shutdown()
    finally:
        print("Done")
        
if __name__ == '__main__':
    main()
