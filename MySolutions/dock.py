# https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Code/Jupyter_Notebook/action_dock.ipynb
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DockServo

class DockActionClient(Node):
    def __init__(self):
        super().__init__('dock_action_client')
        self._action_client = ActionClient(self, DockServo, '/rogers/dock')

    def send_goal(self):
        goal_msg = DockServo.Goal()
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    dock_client = DockActionClient()
    
    future = dock_client.send_goal()
    rclpy.spin_until_future_complete(dock_client, future)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
