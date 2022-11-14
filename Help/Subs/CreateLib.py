'''
This library talks to the ROS library, setting up some key behaviors
'''

import rclpy, os, sys
from Subs.ROS2Lib import Drive, Rotate, Lights, Audio, TwistIt, Battery, Dock, unDock
from Subs.TCPLib import TCPServer
import time

def getID(name):
    creates = [{"Name":"Syndrome","ID":"1","IP":"10.247.137.242"},{"Name":"Dory","ID":"2","IP":"10.247.137.243"},{"Name":"Nemo","ID":"3","IP":"10.247.137.244"},{"Name":"Remy","ID":"4","IP":"10.247.137.245"},{"Name":"EdnaMode ","ID":"5","IP":"10.247.137.246"},{"Name":"Woody ","ID":"6","IP":"10.247.137.247"},{"Name":"Buzz","ID":"7","IP":"10.247.137.241"},{"Name":"MikeWazowski","ID":"8","IP":"10.247.137.248"},{"Name":"LightningMcQueen ","ID":"9","IP":"10.247.137.249"},{"Name":"Mater","ID":"10","IP":"10.247.137.250"},{"Name":"Dash","ID":"11","IP":"10.247.137.251"},{"Name":"ElastaGirl","ID":"12","IP":"10.247.137.252"},{"Name":"Pig","ID":"13","IP":"10.247.137.240"},{"Name":"PotatoHead","ID":"14","IP":"10.247.137.239"},{"Name":"Dot","ID":"15","IP":"10.247.137.238"},{"Name":"Sully","ID":"16","IP":"10.247.137.237"},{"Name":"Carl","ID":"17","IP":"10.247.137.236"},{"Name":"Merida","ID":"18","IP":"10.247.137.235"}]    
    for create in creates:
        if create['Name'] == name:
            return int(create['ID'])
    return None

class Create():
    def __init__ (self, namespace = '', ID = -1):
        rclpy.init(args = None)
        self.namespace = namespace
        self.drive_client = Drive(namespace)
        self.rotate_client = Rotate(namespace)
        self.led_publisher = Lights(namespace)
        self.audio_publisher = Audio(namespace)
        self.twist_publisher = TwistIt(namespace)
        self.serial = None
        
        self.battery_sub = Battery(namespace)
        self.dock_client = Dock(namespace)
        self.undock_client = unDock(namespace)
        
        if ID == -1:
            ID = getID(namespace[1:])
            ID = ID if ID else 0  # if found use it, otherwise user 0
        os.environ['ROS_DOMAIN_ID'] = str(ID)
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

    def dockIt(self):
        '''
        docks the robot
        '''
        print('docking goal', end = '')
        self.dock_client.set_goal()
        print(' set ', end = '')
        self.wait(self.dock_client)
        print('done')

    def undockIt(self):
        '''
        undocks the robot
        '''
        print('undocking goal', end = '')
        future = self.undock_client.set_goal()
        print(' set ', end = '')
        self.wait(self.undock_client)
        #rclpy.spin_until_future_complete(self.undock_client, future)
        print('done')

    def battery(self):
        print('ask battery ', end = '')
        self.wait(self.battery_sub)
        print('done')
        return(' %0.1f ' % self.battery_sub.charge)
        
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
