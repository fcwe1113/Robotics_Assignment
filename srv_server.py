#!/usr/bin/env python3
from __future__ import print_function
import rospy
# import the service message python classes generated from SetBool.srv
from std_srvs.srv import SetBool, SetBoolResponse

# callback function to process client requents
def pw_callback(request):
    rospy.loginfo("SetBool Server has been called")
    cur_response = SetBoolResponse()
    if request.data == True :
        cur_response.success = True
        cur_response.message = "OK"
        return cur_response
    else:
        cur_response.success = False
        cur_response.message = "Not OK"
        return cur_response

# create a server node
rospy.init_node("pw_srv_server")
# create the Service called pw_service with the defined callback
pw_service = rospy.Service("setBool_service", SetBool, pw_callback)
rospy.loginfo("Service/setBool_service is ready !")
# maintain the service open
rospy.spin()