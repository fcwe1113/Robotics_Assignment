#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

def odom_callback(msg):
    global distance_moved, initial_position

    # Retrieve current position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    if initial_position is None:
        initial_position = (x, y)
    else:
        # Calculate distance moved
        distance_moved = math.sqrt((x - initial_position[0])**2 + (y - initial_position[1])**2)

if __name__=="__main__":
    while True: # user has to input valid distance to leave loop
        dist = input("please enter the distance to be travelled in meters: ")
        try:
            dist = float(dist)
        except ValueError:
            print("please enter a valid number")
            continue
        finally:
            if dist < 0:
                print("please enter a positive number")
                continue
            print(f"moving the robot by {dist} meters")
            break
    
    distance_moved = 0.0
    initial_position = None
    
    rospy.init_node('odom_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz meaning it reads data every 100ms

    move_cmd = Twist()
    move_cmd.linear.x = 0.2 # movement speed

    # moving forward one meter
    while not rospy.is_shutdown() and distance_moved < dist: # using < not == to account for the robot not stopping completely immediately
        pub.publish(move_cmd)
        rate.sleep()

    print("movement complete")
    # Stop the robot
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    initial_position = None

                
    