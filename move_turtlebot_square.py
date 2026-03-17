#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

class MoveTurtleBot():

    def __init__(self):
        self.turtlebot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.turtlebot_vel_publisher.get_num_connections()
            if connections > 0:
                self.turtlebot_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_turtlebot()
        self.ctrl_c = True

    def stop_turtlebot(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_turtlebot(self, moving_time, linear_speed=0.2, angular_speed=0.2):

        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0

        rospy.loginfo("Moving TurtleBot!")
        while not self.ctrl_c and i <= moving_time:
            self.publish_once_in_cmd_vel()
            i = i+1
            self.rate.sleep()

        self.stop_turtlebot()
    
if __name__ == '__main__':
    rospy.init_node('move_turtlebot_test', anonymous=True)
    moveturtlebot_object = MoveTurtleBot()
    try:
        while(True):
            moveturtlebot_object.move_turtlebot(5, 0.5, 0)
            moveturtlebot_object.move_turtlebot(1, 0, math.pi/4)
        
    except rospy.ROSInterruptException:
        pass