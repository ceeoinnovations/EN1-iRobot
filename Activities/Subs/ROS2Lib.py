
'''
Stolen from  Kate and Maddie

This library allows you to control the create with a number of simple, synchronous calls.
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import AudioNoteVector 
from irobot_create_msgs.msg import AudioNote 
from builtin_interfaces.msg import Duration  
import geometry_msgs.msg

class Audio(Node):
    '''
    Set up a node that lets you publish notes to the speaker
    '''
    def __init__(self, namespace = '/Picard'):   
        '''
        define the node and set up the publisher
        '''
        super().__init__('audio_publisher')
        
        self.audio_publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio', 10)
        self.audio = AudioNoteVector()
        self.append = False
        
    def beep(self, frequency = 440):
        '''
        publish the requested frequency for 1 second
        '''
        self.audio.notes = [AudioNote(frequency = frequency, max_runtime = Duration(sec = 1, nanosec = 0))]
        self.audio_publisher.publish(self.audio)

class Lights(Node):
    '''
    Create a publisher that will update the LED color on the Create
    '''
    def __init__(self, namespace = '/Picard'):  
        '''
        Create a node, nitialize possible colors and create the publisher as part of this node.
        '''
        super().__init__('led_publisher')

        red = LedColor(red=255, green=0, blue=0)
        green = LedColor(red=0, green=255, blue=0)
        blue = LedColor(red=0, green=0, blue=255)
        yellow = LedColor(red=255, green=255, blue=0)
        pink = LedColor(red=255, green=0, blue=255)
        cyan = LedColor(red=0, green=255, blue=255)
        purple = LedColor(red=127, green=0, blue=255)
        white = LedColor(red=255, green=255, blue=255)
        grey = LedColor(red=189, green=189, blue=189)
        tufts_blue = LedColor(red=98, green=166, blue=10)
        tufts_brown = LedColor(red=94, green=75, blue=60)
        self.colors = [white, red, green, blue, yellow, pink, cyan, purple, grey,tufts_blue,tufts_brown]

        self.lights = self.create_publisher(LightringLeds, namespace + '/cmd_lightring', 10)
        self.lightring = LightringLeds()
        self.lightring.override_system = True  # override the Create error lighting

    def set_color(self, led_colors):
        '''
        set up the proper message type and publish it
        '''
        current_time = self.get_clock().now()
        self.lightring.leds = [self.colors[led_colors]]*6 
        self.lightring.header.stamp = current_time.to_msg()
        self.lights.publish(self.lightring)

    def reset(self):
        '''
        Release control of the lights and "gives" it back to the robot. 
        '''
        print('Resetting color to white')
        self.lightring.override_system = False
        white = [self.colors[0]]*6
        self.lightring.leds = white

        self.lights.publish(self.lightring)

class TwistIt(Node):
    '''
    Control the speed of the Create - both linearly and rotationally to form a twist.
    '''
    def __init__(self, namespace = '/Picard'):   
        '''
        define the node and the publisher
        '''
        super().__init__('twist_publisher')
        self.twist_publisher = self.create_publisher(geometry_msgs.msg.Twist, namespace + '/cmd_vel', 10)

    def move(self, x,y,z,th, speed, turn):
        '''
        publish the desired twist
        '''
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.linear.z = z * speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * turn
        self.twist_publisher.publish(twist)

class Drive(Node):
    '''
    Set up a node, 'drive_distance_action_client', that is an action 
    client and will drive the Create across a desired distance.  The done flag
    tells the wait function in CreateLib that you are done
    '''
    def __init__(self, namespace = '/Picard'):
        #  define an action client for driving the Create
        self.done = False
        super().__init__('drive_distance_action_client')
        self._action = ActionClient(self, DriveDistance, namespace + '/drive_distance')
        # note that the action client uses message type DriveDistance and we defined the namespace
        
    def set_goal(self, distance = 0.5, speed = 0.15):
        '''
        Set the goal of speed and distance and sets the callback to know when the goal is accepted
        '''
        self.done = False
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = speed

        self._action.wait_for_server()  # Wait for the server to be available and then send the goal.
        self._send_goal_future = self._action.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_request_callback)

    def goal_request_callback(self, future): # A callback that is executed when the future is complete.
        '''
        run this when the action server responds with either go or no-go and set up another callback for when the action is done
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.done = True
            return
        self.get_logger().info('Goal accepted :)')
        # goal accepted - now wait for it to be executed
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        run this when the action is done
        '''        
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.done = True

class Rotate(Node):
    '''
    Set up a node, 'rotate_action_client', that is an action client and 
    will rotate the Create through a desired angle. The methods are very
    similar to those in Drive
    '''
    def __init__(self, namespace = '/Picard'):
        #  define an action client for turning the Create
        self.done = False
        super().__init__('rotate_action_client')
        self._action = ActionClient(self, RotateAngle, namespace + '/rotate_angle')
        
    def set_goal(self, angle=1.57, max_rotation_speed=0.5):
        self.done = False
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action.wait_for_server() # wait for server
        self._send_goal_future = self._action.send_goal_async(goal_msg)  
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):  # execute when action server says go or no-go.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.done = True
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  #run when goal is completed
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.done = True
