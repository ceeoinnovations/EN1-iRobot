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
            print('...', end = '')
            rclpy.spin_once(client)

    def forward(self,dist = 0.5):
        '''
        goes the distance and then stops the ROS2 connection
        '''
        speed = 0.25
        print('forward %f: goal' % dist, end = '')
        self.drive_client.set_goal(dist,speed)
        print(' set ', end = '')
        self.wait(self.drive_client)
        print('done')
    
    def turn(self,angle = 1.57):
        '''
        rotates a given angle
        '''
        speed = 0.5   
        print('turn %f: goal' % angle, end = '')
        self.rotate_client.set_goal(angle, speed)
        print(' set ', end = '')
        self.wait(self.rotate_client)
        print('done')
    
    def LED(self,color):
        '''
        changes the color of the LED
        '''
        led_colors = color
        print('publish LED ', end = ''        )
        self.led_publisher.set_color(led_colors)
        print('done')

    def close(self):
        self.drive_client.destroy_node()
        self.rotate_client.destroy_node()
        self.led_publisher.destroy_node()
        rclpy.shutdown()
