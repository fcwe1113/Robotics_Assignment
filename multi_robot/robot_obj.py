import numpy as np
import rospy
import threading
import math

from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class robot_obj:

    name: str
    position: Tuple[float, float]
    orientation: float # in degrees
    init_odom_state: bool
    bot_thread: threading.Thread
    speed: float # front back axis, shows percent of max speed
    rotation: float # left right axis, shows percent of rotation
    moving: bool
    PID: bool # shows robot moving with PID or raw speed/rotation vars
    PID_queue: list[Tuple[float, float]] # list of tuples containing (dest_x, dest_y)
    PID_DIST_THRESHOLD: float # in meters
    PID_ORIENTATION_ROTATE_THRESHOLD: float # in degrees
    PID_ORIENTATION_MOVE_THRESHOLD: float # prevent linear movement before the robot is facing the right direction
    PID_MAX_SPEED: float # max speed percentage available to PID movement system
    PID_MAX_ROTATE: float # max rotation percentage available to PID movement system
    MAX_SPEED: float # max speed of the robot
    MAX_ROTATION: float # max rotation of the robot

    def __init__(self, name):
        self.name = name
        self.init_odom_state = False
        self.twist = Twist()
        self.speed = 0
        self.rotation = 0
        self.moving = False
        self.PID = False
        self.PID_queue = []
        self.PID_DIST_THRESHOLD = 0.001
        self.PID_ORIENTATION_ROTATE_THRESHOLD = 2
        self.PID_ORIENTATION_MOVE_THRESHOLD = 5
        self.PID_MAX_SPEED = 0.5
        self.PID_MAX_ROTATE = 0.3
        self.MAX_SPEED = 0.22
        self.MAX_ROTATION = 2.84

        # start the bot thread
        self.bot_thread = threading.Thread(target=self.spin)
        self.bot_thread.start()
    
    def get_movement_vars(self) -> Tuple[float, float]:
        return (self.speed, self.rotation)
    
    def set_movement_vars(self, speed, rotation):
        self.speed = speed if speed <= 1 and speed >= -1 else 1 if speed > 1 else -1
        self.rotation = rotation if rotation <= 1 and rotation >= -1 else 1 if rotation > 1 else -1

    def set_moving(self, moving):
        self.moving = moving
    
    def stop_moving(self):
        self.speed = 0
        self.rotation = 0
        movement_control.publish(self.twist)
        self.set_moving(False)
    
    def distance_to_point(self, x2, y2): # in meters
        (x1, y1) = self.position
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def angle_to_point(self, x2, y2):
        (x1, y1) = self.position
        return math.arctan2(y2 - y1, x2 - x1) if x1 != x2 or y1 != y2 else 0
    
    def angle_difference(self, x2, y2):
        angle = angle_to_point(x2, y2)
        if math.abs(self.orientation - angle) > 180:
            if self.orientation > angle:
                return angle + 360 - self.orientation
            else:
                return self.orientation + 360 - angle
        else:
            if self.orientation > angle:
                return self.orientation - angle
            else:
                return angle - self.orientation
    
    def update_callback(self, msg):
        position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        # line below modified from https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/turtlebot3/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
        self.orientation = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2)) * 180 / math.pi + 180
        self.position = [position.x, position.y] # tuples assign like mini arrays
        self.init_odom_state = True
        # print(f"robot {self.name} callback, pos: {self.position}")
    
    def __repr__(self): # similar to to_string()
        return f"robot name: {self.name}, position: {self.position}, angle: {self.orientation}°"
    
    def to_string_long(self):
        return f"robot name: {self.name}\nposition: {self.position}\nangle: {self.orientation}°\nspeed: {round(self.speed * 100, 2)}%\nrotation: {round(self.rotation * 100, 2)}%"
    
    def set_use_PID(self, value):
        self.PID = value
    
    def PID_enqueue(self, x, y):
        self.PID_queue.append((x, y))
    
    def PID_clear(self):
        self.PID_queue = []

    def PID_linear_movement(self):
        ...

    def PID_angular_movement(self):
        self.twist.angular.z = self.rotation * self.MAX_ROTATION
    
    def PID_debug_string(self):
        (x, y) = self.PID_queue[-1]
        return f"PID stats:\ncurrent destination: {self.PID_queue[-1]}\n\norientation:\ncurrent orientation: {self.orientation}°\ndestination orientation: {self.angle_to_point(x, y)}\ncurrent orientation difference: {self.angle_difference(x, y)}\ncurrent angular movement: {self.rotation}%\n\ndistance:\ncurrent distance to destination: {self.distance_to_point(x, y)}\ncurrent speed: {self.speed}%" if self.PID_queue else "PID has no queued waypoints"
    
    def spin(self):
        rospy.Subscriber(f'{self.name}/odom', Odometry, self.update_callback) # check if functioning
        self.movement_control = rospy.Publisher(f'{self.name}/cmd_vel', Twist, queue_size=10) # check if functioning
        rate = rospy.Rate(10)  # 10 Hz meaning it reads data every 100ms
        while not rospy.is_shutdown():
            # do pid stuff here
            if self.moving:
                if self.PID:
                    if not self.PID_queue:
                        (x, y) = self.PID_queue[-1]
                        arrived = True
                        if self.distance_to_point(x, y) > self.PID_DIST_THRESHOLD and self.angle_difference < self.PID_ORIENTATION_MOVE_THRESHOLD:
                            arrived = False
                            self.PID_linear_movement()
                        
                        if self.angle_to_point(x, y) > self.PID_ORIENTATION_ROTATE_THRESHOLD:
                            arrived = False
                            self.PID_angular_movement()
                        
                        if arrived:
                            PID_queue.pop()

                else:
                    self.twist.linear.x = self.speed * self.MAX_SPEED
                    self.twist.angular.z = self.rotation * self.MAX_ROTATION
                
                self.movement_control.publish(self.twist) # test when cutting this

            rate.sleep()
    
    