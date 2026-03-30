import numpy as np
import rospy
import threading

from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class robot_obj:

    name: str
    position: Tuple[float, float]
    orientation: float # degrees
    init_odom_state: bool
    bot_thread: threading.Thread
    speed: float # check top speed of bot
    moving: bool

    def __init__(self, name):
        self.name = name
        self.bot_thread = threading.Thread(target=self.spin)
        self.bot_thread.start()
        self.init_odom_state = False
    
    def update_callback(self, msg):
        # print(f"{self.name} callback")
        position = msg.pose.pose.position
        # print(f"{self.name} pos")
        quat = msg.pose.pose.orientation
        # print(f"{self.name} quat")
        # line below taken from https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/turtlebot3/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
        self.orientation = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2))
        # print(f"{self.name} angle calc ed")
        self.position = [position.x, position.y] # tuples assign like mini arrays
        # print(f"{self.name} pos saved")
        self.init_odom_state = True
        # print(f"robot {self.name} callback, pos: {self.position}")
    
    def __repr__(self): # similar to to_string()

        return f"robot name: {self.name}, position: {self.position}, angle: {self.orientation}°)"
    
    def spin(self):
        # try:
        #     rospy.init_node(f'multibot_node', disable_signals=True)
        # except rospy.exceptions.ROSException as _:
        #     print("node started already, skipping to bot spawn")
        rospy.Subscriber(f'{self.name}/odom', Odometry, self.update_callback) # check if functioning
        self.movement_control = rospy.Publisher(f'{self.name}/cmd_vel', Twist, queue_size=10) # check if functioning
        rate = rospy.Rate(10)  # 10 Hz meaning it reads data every 100ms
        # rospy.spin()
        # print(f"{self.name} robot thread starting")
        while not rospy.is_shutdown():
            # do pid stuff here
            rate.sleep()
    
    def to_string_long(self):

        return f"robot name: {self.name}\nposition: {self.position}\nangle: {self.orientation}°)"
