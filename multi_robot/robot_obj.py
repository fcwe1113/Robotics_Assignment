import numpy as np
import rospy
import threading
import math

from typing import Tuple, List
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_PID import robot_PID

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
    PID_queue: List[Tuple[float, float]] # list of tuples containing (dest_x, dest_y)
    PID_ongoing: bool # flag to identify ongoing PID operation
    PID_movement_to_go: float # number to input into movement_PID
    PID_DIST_THRESHOLD: float # in meters
    PID_ORIENTATION_ROTATE_THRESHOLD: float # in degrees, functions as the "good enough" point
    PID_MAX_SPEED: float # max speed percentage available to PID movement system
    PID_MAX_ROTATE: float # max rotation percentage available to PID movement system
    PID_MAX_SPEED_STEP: float # max acceleration available to PID movement system
    MAX_SPEED: float # max speed of the robot
    MAX_ROTATION: float # max rotation of the robot

    # maybe set pid min speed/rotation

    def __init__(self, name):
        self.name = name
        self.init_odom_state = False
        self.twist = Twist()
        self.speed = 0
        self.rotation = 0
        self.moving = False
        self.PID = False

        self.PID_queue = []
        self.PID_ongoing = False
        self.PID_movement_to_go = 0
        self.PID_DIST_THRESHOLD = 0.1
        self.PID_ORIENTATION_ROTATE_THRESHOLD = 5
        self.PID_MAX_SPEED = 50
        self.PID_MAX_ROTATE = 0.3
        self.PID_MAX_SPEED_STEP = 0.001

        self.movement_PID_output = 0
        self.rotation_PID_output = 0
        
        self.MAX_SPEED = 0.22
        self.MAX_ROTATION = 2.84
        self.rotation_PID = robot_PID()
        self.movement_PID = robot_PID()
        self.move_counter = 50

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
        self.movement_control.publish(self.twist)
        self.set_moving(False)
    
    def distance_to_point(self, x2, y2): # in meters
        (x1, y1) = self.position
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def angle_to_point(self, x2, y2): # required orientation to face the dest
        (x1, y1) = self.position
        return np.rad2deg(np.arctan2(y2 - y1, x2 - x1)) + 180 if x1 != x2 or y1 != y2 else 0
    
    def angle_difference(self, x2, y2): # the angle diff between current and required angle
        output = self.orientation - self.angle_to_point(x2, y2)
        return output if output < 180 else 180 - output
    
    def update_callback(self, msg):
        position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        # line below modified from https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/turtlebot3/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
        self.orientation = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2)) * 180 / math.pi + 180 # added 180 bc the original scale goes from -180 to 180
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
    
    def PID_debug_string(self):
        if self.PID_queue:
            (x, y) = self.PID_queue[0]
        return f"\n\nPID stats:\ncurrent destination: {self.PID_queue[0]}\nqueue: {self.PID_queue}\n\norientation:\ncurrent orientation: {self.orientation}°\ndestination orientation: {self.angle_to_point(x, y)}°\ncurrent orientation difference: {self.angle_difference(x, y)}°\ncurrent angular movement: {self.rotation}%\ncurrent rotational PID output: {self.rotation_PID_output}\n\ndistance:\ncurrent distance to destination: {self.distance_to_point(x, y)}\ncurrent speed: {self.speed}%\ncurrent movement PID output: {self.movement_PID_output}\nmove counter: {self.move_counter}" if self.PID_queue else "PID has no queued waypoints"
    
    def spin(self):
        rospy.Subscriber(f'{self.name}/odom', Odometry, self.update_callback) # check if functioning
        self.movement_control = rospy.Publisher(f'{self.name}/cmd_vel', Twist, queue_size=10) # check if functioning
        rate = rospy.Rate(10)  # 10 Hz meaning it reads data every 100ms
        while not rospy.is_shutdown():
            # do pid stuff here
            if self.moving:
                if self.PID:
                    if self.PID_queue: # does not run if queue is empty

                        (x, y) = self.PID_queue[0]

                        if not self.PID_ongoing: # update destination to PIDs when starting new operation
                            self.movement_PID.update_setpoint(self.distance_to_point(x, y))
                            self.PID_movement_to_go = self.distance_to_point(x, y)
                            self.move_counter = 50
                            self.PID_ongoing = True

                        arrived = False

                        # PID operational flowchart
                        # 1. if angle diff is higher than threshold, rotate till lower than threshold
                        # 2. if angle diff lower than threhold and distance higher than threhold, start moving linearly towards goal, rotation PID stays active to correct course if needed
                        # 3. if distance lower than threshold, stop both PIDs and mark arrived flag, assign next waypoint if available
                                                
                        if abs(self.angle_difference(x, y)) < self.PID_ORIENTATION_ROTATE_THRESHOLD or self.move_counter == 0: # check if move counter should stay
                            if self.move_counter > 0:
                                self.move_counter -= 1
                            else:
                                self.movement_PID_output = self.movement_PID.compute(self.distance_to_point(x, y) - self.PID_movement_to_go) * 10
                                self.speed = min(self.movement_PID_output, self.PID_MAX_SPEED)# if self.speed > 0 else max(self.PID_accel + self.speed, -self.PID_MAX_SPEED)
                        else:
                            self.speed = 0
                            self.move_counter = 50
                        
                        if self.distance_to_point(x, y) < self.PID_DIST_THRESHOLD:
                            arrived = True
                            self.move_counter = 50
                        
                        self.rotation_PID_output = self.rotation_PID.compute(self.angle_difference(x, y)) / 1
                        self.rotation = min(self.rotation_PID_output, self.PID_MAX_ROTATE) if self.rotation_PID_output > 0 else max(self.rotation_PID_output, -self.PID_MAX_ROTATE)

                        if arrived:
                            self.PID_queue.pop(0)
                            self.PID_ongoing = False

                    else:
                        self.speed = 0
                        self.rotation = 0
                
                self.twist.linear.x = self.speed * self.MAX_SPEED
                self.twist.angular.z = self.rotation * self.MAX_ROTATION
                self.movement_control.publish(self.twist) # test when cutting this

            rate.sleep()
    
    