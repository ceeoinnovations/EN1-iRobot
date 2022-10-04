'''
This library talks to the ROS library, setting up some key behaviors
'''

import rclpy, os, sys
from Subs.ROS2Lib import Drive, Rotate, Lights, Audio, TwistIt
from Subs.TCPLib import TCPServer
import time

class Create():
    def __init__ (self, namespace = ''):
        rclpy.init(args = None)
        self.namespace = namespace
        self.drive_client = Drive(namespace)
        self.rotate_client = Rotate(namespace)
        self.led_publisher = Lights(namespace)
        self.audio_publisher = Audio(namespace)
        self.twist_publisher = TwistIt(namespace)
        self.serial = None
        
        print('ros domain: ' + str(os.environ['ROS_DOMAIN_ID']))
        print('middleware: ' + str(os.environ['RMW_IMPLEMENTATION']))
        reply = sys.version.split(' ')[0]
        print('python version: %s' % reply, end='')
        print ('- good' if  ('3.8' in reply) else '- BAD')
        time.sleep(1)

    def LED(self,color):
        '''
        changes the color of the LED
        '''
        led_colors = color
        print('publish LED ', end = '')
        self.led_publisher.set_color(led_colors)
        time.sleep(1)
        print('done')

    def beep(self, frequency = 440):
        '''
        Beeps
        '''
        print('publish beep ', end = '')
        self.audio_publisher.beep(frequency)
        time.sleep(1)
        print('done')
        
    def twist(self, x, y, z, th, speed, turn):
        '''
        twists the Create - move in x,y,z and rotate theta
        '''
        print('publish twist ', end = '')
        self.twist_publisher.move(x,y,z,th, speed, turn)
        print('done')
            
    def turn(self,angle = 90, speed = 0.5):
        '''
        rotates a given angle
        '''
        
        angle = angle/180*3.1415
        print('turn %0.2f: goal' % angle, end = '')
        self.rotate_client.set_goal(float(angle), speed)
        print(' set ', end = '')
        self.wait(self.rotate_client)
        print('done')

    def forward(self,dist = 0.5):
        '''
        goes the distance and then stops the ROS2 connection
        '''
        speed = 0.25
        print('forward %0.2f: goal' % dist, end = '')
        self.drive_client.set_goal(float(dist),speed)
        print(' set ', end = '')
        self.wait(self.drive_client)
        print('done')

    def wait(self, client):
        rclpy.spin_once(client)
        while not client.done:
            #time.sleep(0.1)
            print('...', end = '')
            rclpy.spin_once(client)
            
    def close(self):
        print('closing ', end = '')
        self.drive_client.destroy_node()
        self.rotate_client.destroy_node()
        self.led_publisher.destroy_node()
        self.audio_publisher.destroy_node()
        rclpy.shutdown()
        print('done')

# ----------------------------------------serial calls using serial over TCP------------------------- 

    def serial_init(self, IP, PORT, timeout = 0):
        self.serial = TCPServer (IP, PORT, timeout)
        
    def serial_write(self, string):
        if self.serial:
            self.serial.write(string)
        else:
            print('serial not initialized')
            
    def serial_write_binary(self, string):
        if self.serial:
            self.serial.write_binary(string)
        else:
            print('serial not initialized')
            
    def serial_abort(self):
        self.serial.write_binary(b'\x03')
            
    def serial_run(self, code):
        code = code.replace('\n','\r\n')
        code = code.replace('\t','    ')
        if self.serial:
            self.serial.write_binary(b'\x05') # Ctrl E
            self.serial.write(code)
            self.serial.write_binary(b'\x04')  #Ctrl D
        else:
            print('serial not initialized')
            
    def serial_read(self):
        if self.serial:
            return self.serial.read()
        else:
            print('serial not initialized')
        return None
        
    def serial_close(self):
        if self.serial:
            return self.serial.close()
        else:
            print('serial not initialized')
