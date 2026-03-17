#!/usr/bin/env python3

import rospy
import math
from move_turtlebot import MoveTurtleBot
    
if __name__ == '__main__':
    rospy.init_node('move_turtlebot_test', anonymous=True)
    moveturtlebot_object = MoveTurtleBot()
    try:
        while(True):
            moveturtlebot_object.move_turtlebot(5, 0.5, 0)
            moveturtlebot_object.move_turtlebot(1, 0, math.pi/4)
        
    except rospy.ROSInterruptException:
        pass