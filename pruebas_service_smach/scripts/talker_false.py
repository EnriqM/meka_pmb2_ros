#!/usr/bin/env python
'''
Created on 15 jul. 2017

@author: Enrique Ortega
'''

import rospy
#from std_msgs.msg import String
#from rospy_tutorials.srv import *
from std_srvs.srv import SetBool#, SetBoolResponse, SetBoolRequest
 

rospy.wait_for_service('set_booleano')
set_flag = rospy.ServiceProxy('set_booleano', SetBool)
try:
    resp1 = set_flag(False)
    rospy.loginfo('Respuesta = %s'%resp1.message)
    print resp1.message
except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))
