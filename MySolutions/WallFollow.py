'''
taken from
https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Projects/Joust/airtable_joust.py  
'''

from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.action import ActionClient
from irobot_create_msgs.action import WallFollow

class WallFollowActionClient(Node):
    def __init__(self):
        super().__init__('wall_follow_action_client')
        self.subscription = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection', self.listener_callback, qos_profile_sensor_data)
        self._action_client = ActionClient(self, WallFollow, namespace + '/wall_follow')

    def listener_callback(self, msg):
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    self.send_goal(follow_side=-1)
                elif det == "bump_left":
                    self.send_goal(follow_side=1)
                elif det == "bump_front_right":
                    self.send_goal(follow_side=-1)
                elif det == "bump_front_left":
                    self.send_goal(follow_side=1)
                elif det == "bump_front_center":
                    pass
    
    def send_goal(self, follow_side=-1, max_runtime=10):
        print('Ready for Action')
        
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
            
        goal_msg.max_runtime = Duration(sec=10, nanosec=0)
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init(None)
     
    wall_client = WallFollowActionClient()
    wall_client.send_goal(follow_side=-1, max_runtime=10)
    rclpy.spin(wall_client)

if __name__ == '__main__':
    main()