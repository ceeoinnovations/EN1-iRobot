# https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Code/Jupyter_Notebook/action_undock.ipynb

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock

class UndockingActionClient(Node):

    def __init__(self):
        super().__init__('undocking_action_client')
        self._action_client = ActionClient(self, Undock, '/rogers/undock')

    def send_goal(self):
        goal_msg = Undock.Goal()
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    undock_client = UndockingActionClient()
    future = undock_client.send_goal()
    rclpy.spin_until_future_complete(undock_client, future)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()