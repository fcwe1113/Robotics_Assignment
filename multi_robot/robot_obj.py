import numpy as np
import rospy

from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class robot_obj:

    name: str
    position: Tuple[float, float]
    orientation: float # degrees

    def __init__(self, name):
        self.name = name
        rospy.Subscriber(f'{name}/odom', Odometry, self.update) # check if functioning
        self.movement_control = rospy.Publisher(f'{name}/cmd_vel', Twist, queue_size=10) # check if functioning
    
    def update(self, msg):
        position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        # line below taken from https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/turtlebot3/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
        self.orientation = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2))
        self.position = tuple(position.x, position.y)
    
    def __repr__(self): # similar to to_string()
        
        return f"robot name: {self.name}, position: {self.position}, angle: {self.orientation})"