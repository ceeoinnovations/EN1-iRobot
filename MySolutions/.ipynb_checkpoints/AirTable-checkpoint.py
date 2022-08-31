'''
airtable_telerobot.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

https://github.com/tuftsceeo/Tufts_Create3_Examples/tree/main/Projects/Airtable

In this example we will use airtable to remotely control the Create®3.
'''
import sys
import rclpy
from rclpy.node import Node

import requests
import json 

from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

namespace = '/happy'
URL = 'https://api.airtable.com/v0/appyojjNhdfdHU8u7/tblzZDLFCzlk7fOqP?api_key=XXX'


class WheelVel(Node):
    def __init__(self):    
        super().__init__('wheel_vel')  # topic name
        self.wheels = Twist()
        self.wheels_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        timer_period = 2  #check every 2 sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
         
    def get_values(self, name, data):
        vector = Vector3()
        for record in data['records']:
            if record['fields']['Name']==name:
                vector.x = float(record['fields']['X'])
                vector.y = float(record['fields']['Y'])
                vector.z = float(record['fields']['Z'])
                return vector

    def timer_callback(self):
        try:
            r = requests.get(url = URL, params = {})
            data = r.json()
            self.wheels.linear = self.get_values('Linear', data)
            self.wheels.angular = self.get_values('Angular', data)
            
            self.wheels_publisher.publish(self.wheels)  # Send it out
        except:
            print('failed')
            return

def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
    
    '''
    Create the node
    '''
    wheel_vel = WheelVel()
    
    try:
        rclpy.spin(wheel_vel)
        
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
        rclpy.shutdown()
        
    finally:
        print("Done")  # Destroy the node explicitly
        
if __name__ == '__main__':
    main()
