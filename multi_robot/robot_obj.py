import numpy as np
import rospy
import threading
import math

from typing import Tuple, List
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_PID import robot_PID
from robot_states import Bot_State

class robot_obj:
    """
    Class for individual robot control, each object is "supposed" to correspond to an individual robot namespace (in this case its "tb3_num")

    :cvar MAX_SPEED: This is the highest speed this robot can physically achieve
    :cvar MAX_ROTATION: This is the highest rotation this robot can physically achieve
    :cvar PID_DIST_THRESHOLD: Robots will treat distances under this value as arrived
    :cvar PID_ORIENTATION_ROTATE_THRESHOLD: Robots will treat angle differences under this value as oriented
    :cvar PID_MAX_SPEED: Robots in PID mode will not exceed this speed percentage
    :cvar PID_MAX_ROTATE: Robots in PID mode will not exceed this rotation percentage

    :cvar green_light: Flag for outside controller to signify permission to continue to next state
    :cvar controlled: Flag to show if robot is being controlled by controller or directly by user
    :cvar state: Enum to show state of robot (i.e. what is it doing right now)

    :cvar bot_thread: Thread to operate robot in real time
    :cvar twist: Robot movement controller, see here for API: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    :cvar movement_control: Robot command publisher, publish changes to twist here for them to take effect
    :cvar movement_PID: Robot linear movement PID controller
    :cvar rotation_PID: Robot angular movement PID controller

    :cvar name: Name of the robot
    :cvar position: Position of robot, in the format of tuple[x, y]
    :cvar orientation: The angle the robot is facing, in degrees
    :cvar speed: Speed of robot
    :cvar rotation: Rotation of robot, in degrees
    :cvar PID_movement_to_go: Variable to track PID linear movement progress
    :cvar movement_PID_output: movement_PID output
    :cvar rotation_PID_output: rotation_PID output
    :cvar move_counter: counter for rotation to stabilise before enabling linear movement

    """

    name: str
    position: Tuple[float, float]
    orientation: float
    bot_thread: threading.Thread
    speed: float
    rotation: float
    PID_queue: List[Tuple[float, float]]
    PID_movement_to_go: float
    PID_DIST_THRESHOLD: float
    PID_ORIENTATION_ROTATE_THRESHOLD: float
    PID_MAX_SPEED: float
    PID_MAX_ROTATE: float
    MAX_SPEED: float
    MAX_ROTATION: float
    state: Bot_State
    green_light: bool
    controlled: bool

    # attributes (or constants)
    PID_DIST_THRESHOLD = 0.1
    PID_ORIENTATION_ROTATE_THRESHOLD = 5
    PID_MAX_SPEED = 3
    PID_MAX_ROTATE = 0.3
    MAX_SPEED = 0.22
    MAX_ROTATION = 2.84

    def __init__(self, name):
        """
        Class constructor for robot_obj

        :param str name: Name of the robot
        """
        self.name = name
        self.twist = Twist()
        self.speed = 0
        self.rotation = 0
        self.movement_control = None

        self.PID_queue = []
        self.PID_movement_to_go = 0

        self.movement_PID_output = 0
        self.rotation_PID_output = 0

        self.rotation_PID = robot_PID()
        self.movement_PID = robot_PID()
        self.move_counter = 10
        self.state = Bot_State.IDLE
        self.green_light = False
        self.controlled = False

        # start the bot thread
        self.bot_thread = threading.Thread(target=self.spin)
        self.bot_thread.start()
    
    def is_ready(self) -> bool:
        """
        Checks if the robot is fully spawned

        :return: True if robot is spawned, False otherwise
        """
        try:
            _ = self.position
        except AttributeError:
            return False
        return True

    def get_movement_vars(self) -> Tuple[float, float]:
        """
        Gets robot speed and rotation in the format of [speed, rotation]

        :rtype: Tuple[float, float]
        """
        return self.speed, self.rotation
    
    def set_movement_vars(self, speed, rotation):
        """
        Sets robot speed and rotation

        :param float speed: New robot speed
        :param float rotation: New robot rotation
        :rtype: None
        """
        self.speed = speed if 1 >= speed >= -1 else 1 if speed > 1 else -1
        self.rotation = rotation if 1 >= rotation >= -1 else 1 if rotation > 1 else -1

    def set_state(self, state):
        """
        Setter for robot state

        :param Bot_State state: New robot state
        :rtype: None
        """
        self.state = state

    def get_state(self) -> Bot_State:
        """
        Getter for robot state

        :rtype: Bot_State
        """
        return self.state

    def give_green_light(self):
        """
        Gives the robot green light to move on

        :rtype: None
        """
        self.green_light = True

    def set_controlled(self, controlled):
        """
        Setter for robot controlled flag

        :param bool controlled: New flag state
        :rtype:
        """
        self.controlled = controlled
    
    def stop_moving(self):
        """
        Halts the robot and set it to idle state

        :rtype:
        """
        self.speed = 0
        self.rotation = 0
        self.movement_control.publish(self.twist)
        self.set_state(Bot_State.IDLE)
    
    def distance_to_point(self, x2, y2) -> float: # in meters
        """
        Get the Euclidian distance between the robot and the provided coordinates

        :param float x2: x coordinate of the point
        :param float y2: y coordinate of the point
        :rtype: float
        """
        (x1, y1) = self.position
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def angle_to_point(self, x2, y2) -> float: # required orientation to face the dest
        """
        Get the angle towards the provided coordinates from the bot's perspective in degrees

        :param float x2: x coordinate of the point
        :param float y2: y coordinate of the point
        :rtype: float
        """
        (x1, y1) = self.position
        return np.rad2deg(np.arctan2(y2 - y1, x2 - x1)) + 180 if x1 != x2 or y1 != y2 else 0
    
    def angle_difference(self, x2, y2) -> float: # the angle diff between current and required angle
        """
        Get the angle difference of the bot's current orientation against the angle towards the provided coordinates from the bot's perspective in degrees

        :param float x2: x coordinate of the point
        :param float y2: y coordinate of the point
        :rtype: float
        """
        output = (self.orientation - self.angle_to_point(x2, y2)) % 360
        return output if output < 180 else -(180 - (output - 180))
    
    def angle_difference_percentage(self, x2, y2, cap=180) -> float:
        """
        Gets the percentage difference of the bot's current orientation against the angle towards the provided coordinates from the bot's perspective

        :param float x2: x coordinate of the point
        :param float y2: y coordinate of the point
        :param cap: When does this function starts to take effect, for example if 45 is provided then the scale would start from -45deg to 45deg, where it returns 1 if bot is facing 45deg
        :raise ValueError: If Value of cap is not >= 0 and <= 180
        :rtype: float
        """
        if not (0 >= cap >= 180):
            raise ValueError("Cap angle provided is over 180")
        return 1 - (min(abs(self.angle_difference(x2, y2)), cap) / cap)
    
    def update_callback(self, msg):
        """
        Callback function to update the robot state (position, orientation), will be run constantly

        :param msg: Robot information provided from robot API
        :rtype: None
        """
        position: Tuple[float, float] = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        # line below modified from https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/turtlebot3/turtlebot3/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
        self.orientation = np.arctan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2)) * 180 / math.pi + 180 # added 180 bc the original scale goes from -180 to 180
        self.position = [position.x, position.y] # tuples assign like mini arrays
        # print(f"robot {self.name} callback, pos: {self.position}")
    
    def __repr__(self): # similar to to_string()
        """
        to_string() method similar to Java, does not need to be explicitly evoked when concatting in string

        :rtype: str
        """
        return f"robot name: {self.name}, position: {self.position}, angle: {self.orientation}°"
    
    def to_string_long(self):
        """
        Long form string representation of the robot state

        :rtype: str
        """
        return f"robot name: {self.name}\nposition: {self.position}\nangle: {self.orientation}°\nspeed: {round(self.speed * 100, 2)}%\nrotation: {round(self.rotation * 100, 2)}%"
    
    def PID_info_string(self):
        """
        PID info string of the robot

        :rtype: str
        """
        (x, y) = self.PID_queue[0]
        return f"name: {self.name}\tcurrent position: {self.position}\tcurrent orientation: {self.orientation}\tdestination: ({x}, {y})\tcurrent state: {self.state.value}\n"

    def PID_debug_string(self):
        """
        PID debug string of the robot

        :rtype: str
        """
        if self.PID_queue:
            (x, y) = self.PID_queue[0]
        return f"\n\nPID stats:\ncurrent location: {self.position}\ncurrent destination: {self.PID_queue[0]}\nqueue: {self.PID_queue}\nstate: {self.state.value}\n\norientation:\ncurrent orientation: {self.orientation}°\ndestination orientation: {self.angle_to_point(x, y)}°\ncurrent orientation difference: {self.angle_difference(x, y)}°\ncurrent angular movement: {self.rotation}%\ncurrent rotational PID output: {self.rotation_PID_output}\n\ndistance:\ncurrent distance to destination: {self.distance_to_point(x, y)}\ncurrent speed: {self.speed}%\ncurrent movement PID output: {self.movement_PID_output}\nmove counter: {self.move_counter}\n" if self.PID_queue else "PID has no queued waypoints\n"
    
    def PID_enqueue(self, x, y):
        """
        Enqueues provided coordinates to the robot PID system

        :param x: x coordinate of the point
        :param y: y coordinate of the point
        :rtype: None
        """
        self.PID_queue.append((x, y))
        # print(f"{x}, {y} enqueued")
    
    def PID_clear(self):
        """
        Clears the PID system queue

        :rtype: None
        """
        self.PID_queue = []

    def PID_if_queue_empty(self):
        """
        checks of the PID queue is empty

        :rtype: bool
        """
        return False if self.PID_queue else True
    
    def spin(self):
        """
        Thread function for running the physical robot, to be ran in a thread, currently there is no way to halt except to ctrl-C the main program

        :rtype: None
        """
        rospy.Subscriber(f'{self.name}/odom', Odometry, self.update_callback) # check if functioning
        self.movement_control = rospy.Publisher(f'{self.name}/cmd_vel', Twist, queue_size=10) # check if functioning
        rate = rospy.Rate(10)  # 10 Hz meaning it reads data every 100ms
        while not self.is_ready(): # todo check if this prevents errors from accessing uninit bots
            pass

        while not rospy.is_shutdown():
            # do pid stuff here
            if not self.state == Bot_State.IDLE:
                if self.state in [Bot_State.ROTATING, Bot_State.READY, Bot_State.MOVING, Bot_State.WAITING]:
                    if self.PID_queue: # does not run if queue is empty

                        # PID operational flowchart
                        # 1. if angle diff is higher than threshold, rotate till lower than threshold
                        # 2. if angle diff lower than threhold and distance higher than threhold, start moving linearly towards goal, rotation PID stays active to correct course if needed
                        # 3. if distance lower than threshold, stop both PIDs and mark arrived flag, assign next waypoint if available

                        (x, y) = self.PID_queue[0]

                        if self.state == Bot_State.WAITING: # update destination to PIDs when starting new operation
                            self.movement_PID.update_setpoint(self.distance_to_point(x, y))
                            self.PID_movement_to_go = self.distance_to_point(x, y)
                            self.set_state(Bot_State.ROTATING)
                        elif self.state == Bot_State.ROTATING:
                            self.rotation_PID_output = self.rotation_PID.compute(self.angle_difference(x, y)) / 10
                            self.rotation = (min(self.rotation_PID_output, self.PID_MAX_ROTATE) if self.rotation_PID_output > 0 else max(self.rotation_PID_output, -self.PID_MAX_ROTATE)) * (1 if self.angle_difference(x, y) > 45 else (self.angle_difference(x, y) / 45))
                            if abs(self.angle_difference(x, y)) < self.PID_ORIENTATION_ROTATE_THRESHOLD or self.move_counter == 0:  # check if move counter should stay
                                if self.move_counter > 0:
                                    self.move_counter -= 1
                                    self.speed = 0.01 # pre roll the wheels to prevent spinning out
                                else:
                                    self.set_state(Bot_State.READY)
                            else:
                                self.speed = 0
                                self.move_counter = 50
                        elif self.state == Bot_State.READY:
                            # do formation checks here
                            if self.green_light and self.controlled:
                                self.set_state(Bot_State.MOVING)
                                self.green_light = False
                            elif not self.controlled:
                                self.set_state(Bot_State.MOVING)
                        elif self.state == Bot_State.MOVING:

                            # rotation PID
                            self.rotation_PID_output = self.rotation_PID.compute(self.angle_difference(x, y)) / 10
                            self.rotation = (min(self.rotation_PID_output, self.PID_MAX_ROTATE) if self.rotation_PID_output > 0 else max(self.rotation_PID_output, -self.PID_MAX_ROTATE)) * (1 if self.angle_difference(x, y) > 45 else max(self.angle_difference(x, y) / 45, 0.5))

                            #movement PID
                            self.movement_PID_output = self.movement_PID.compute(self.distance_to_point(x, y) - self.PID_movement_to_go) / 2
                            self.speed = (max(min(self.movement_PID_output, self.PID_MAX_SPEED) * self.angle_difference_percentage(x, y, 45), 0.001)) * min(1.0, self.distance_to_point(x, y)) # if self.speed > 0 else max(self.PID_accel + self.speed, -self.PID_MAX_SPEED), modified by how off course the robot is

                            if self.distance_to_point(x, y) < self.PID_DIST_THRESHOLD: # if destination reached
                                self.PID_queue.pop(0)
                                self.rotation_PID.reset_integral()
                                self.move_counter = 50
                                self.set_state(Bot_State.WAITING)

                    else:
                        self.speed = 0
                        self.rotation = 0
                
                self.twist.linear.x = self.speed * self.MAX_SPEED
                self.twist.angular.z = self.rotation * self.MAX_ROTATION
                self.movement_control.publish(self.twist) # test when cutting this

            rate.sleep()
    
    