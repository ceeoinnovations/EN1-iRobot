#cd /media/psf/Home/Documents/Work/Classes/2022\ Summer//Summer\ 2022/ROS
#exit()
#python3

'''
This library talks to the ROS library, setting up some key behaviors
'''

import rclpy
from ROS2Lib import Drive, Rotate, Lights
import time

class Create():
    def __init__ (self, namespace = ''):
        rclpy.init(args = None)
        self.namespace = namespace
        self.drive_client = Drive(namespace)
        self.rotate_client = Rotate(namespace)
        self.led_publisher = Lights(namespace)
        time.sleep(1)
        
    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            #time.sleep(0.1)
            print('...')
            rclpy.spin_once(client)

    def forward(self,dist = 0.5):
        '''
        goes the distance and then stops the ROS2 connection
        '''
        speed = 0.25
        self.drive_client.set_goal(dist,speed)
        self.wait(self.drive_client)
    
    def turn(self,angle):
        '''
        rotates a given angle
        '''
        speed = 0.5   
        self.rotate_client.send_goal(angle, speed)
        self.wait(self.rotate_client)
    
    def LED(self,color):
        '''
        changes the color of the LED
        '''
        led_colors = color
        self.led_publisher.set_color(led_colors)

    def close(self):
        self.drive_client.destroy_node()
        self.rotate_client.destroy_node()
        self.led_publisher.destroy_node()
        rclpy.shutdown()
