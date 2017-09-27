#!/usr/bin/env python
'''
Created on 18 jul. 2017


Creator: Enrique Ortega, from Universidad de Jaen

El servicio que se usara es el siguiente:

bool interruptor
string modo
string brazo
string frame_objetivo
---
bool success
string message
'''


import rospy
#from std_msgs.msg import String
#from rospy_tutorials.srv import *
#from std_srvs.srv import SetBool#, SetBoolResponse, SetBoolRequest
from nav_to_people.srv import Permiso_Brazos, Permiso_BrazosResponse, Permiso_BrazosRequest


#flag = True
flag = False


rospy.wait_for_service('permiso_meka_arms')
set_meka = rospy.ServiceProxy('permiso_meka_arms', Permiso_Brazos)
try:
    resp1 = set_meka(flag, "mimica", "left", "left_hand_2")
    #resp1 = set_meka(flag, "mimica", "right", "right_hand_1")
    #rospy.loginfo('Respuesta = %s'%resp1.message)
    print resp1.message
except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))
