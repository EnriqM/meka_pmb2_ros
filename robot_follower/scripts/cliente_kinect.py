#!/usr/bin/env python
'''
Created on 29 oct. 2017


Creator: Enrique Ortega, from Universidad de Jaen

El servicio que se usara es el siguiente:


bool interruptor
string frame_objetivo #el frame que se sigue, sin la /.
string frame_referencia #el frame que hace referencia, normalmente robot_link
---
bool success
string message


El antiguo servicio
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
#from nav_to_people.srv import Permiso_Brazos, Permiso_BrazosResponse, Permiso_BrazosRequest
from robot_follower.srv import NumeroTF, NumeroTFResponse, NumeroTFRequest


#flag = True
flag = False


rospy.wait_for_service('permiso_kinect')
set_kinect = rospy.ServiceProxy('permiso_kinect', NumeroTF)
try:
    resp1 = set_kinect(flag, "turtle1", "world")
    #resp1 = set_meka(flag, "mimica", "right", "right_hand_1")
    #rospy.loginfo('Respuesta = %s'%resp1.message)
    print resp1.message
except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))
