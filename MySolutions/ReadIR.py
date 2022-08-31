# https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Code/Jupyter_Notebook/sub_ir.ipynb

import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector

class IRSubscriber(Node):
    def __init__(self):
        super().__init__('IR_subscriber')
        self.subscription = self.create_subscription(IrIntensityVector, 'rogers/ir_intensity', self.listener_callback, qos_profile_sensor_data)

    def listener_callback(self, msg:IrIntensityVector):
        print('Printing IR sensor readings:')
        for reading in msg.readings: 
             val = reading.value
             print("IR Sensor:", val)


def main(args=None):
    rclpy.init(args=args)
    IR_subscriber = IRSubscriber()
    
    try:
        rclpy.spin(IR_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
         IR_subscriber.destroy_node()
         rclpy.shutdown()
         print("Done")

if __name__ == '__main__':
    main()
    