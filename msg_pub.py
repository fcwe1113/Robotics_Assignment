#!/usr/bin/env python3
import rospy
from rss_assignment.msg import date_cmd_vel
from geometry_msgs.msg import Twist
from datetime import datetime, timedelta

rospy.init_node("msg_pub")
now = datetime.now()
date_str = now.strftime("%m/%d/%y,%H:%M:%S")
pw_cmd_vel = date_cmd_vel()
pw_cmd_vel.pw_date = date_str
# pw_cmd_vel.pw_cmd_vel.linear.x = 0.5
# pw_cmd_vel.pw_cmd_vel.angular.z = 0.1
pub = rospy.Publisher("/pw_topic", date_cmd_vel, queue_size=1)
# twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(1)

movement = [(0.25, 0.3), (0, 0), (0.25, 0)]
id = 0

while not rospy.is_shutdown():
    if id == 0 and (datetime.now() - now >= timedelta(seconds=20)):
        id += 1
        print(f"id now {id}")

    if id == 1 and (datetime.now() - now >= timedelta(seconds=25)):
        id += 1
        print(f"id now {id}")

    if id == 2 and (datetime.now() - now >= timedelta(seconds=30)):
        pw_cmd_vel.pw_cmd_vel.linear.x = 0
        pw_cmd_vel.pw_cmd_vel.angular.z = 0
        pub.publish(pw_cmd_vel)
        break

    pw_cmd_vel.pw_cmd_vel.linear.x = movement[id][0]
    pw_cmd_vel.pw_cmd_vel.angular.z = movement[id][1]
    pub.publish(pw_cmd_vel)
    # twist.publish(twist)
    rate.sleep()