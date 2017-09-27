#!/usr/bin/env python

"""


@author: Enrique Ortega

   

Script modificado de patrol.py (el que no usa smach), para dar metas 
respecto el mapa

Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal

Actual robo pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped

SACADO DE: answers.ros.org/question/185907/hel-with-some-code-send-a-navigation-goal/

"""
from __future__ import print_function

import rospy
import actionlib


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees


#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [( -0.845509290695, -1.00263929367, 0.0), (0.0, 0.0, -0.208937036828, 0.977929094895)],
    [( 0.519209861755, -1.67815709114, 0.0), (0.0, 0.0, 0.737550452222, 0.675292033439)],
    [( 0.227547883987, -0.037682056427, 0.0), (0.0, 0.0, 0.959776036849, -0.2807667343)]
]

#Funcion para convertir cada punto en un MoveBaseGoal 
#	Se podra poner frame_id 'map' o 'odom', 'base_link'...
def goal_pose(pose, yaw):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    
    #Orientacion del robot esta expresada en el valor yaw de angulos Euler
    angle = radians(yaw)
    quat = quaternion_from_euler(0.0, 0.0, angle)
    goal_pose.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return goal_pose


if __name__ == '__main__':
    try:
        #Inicia el nodo
        rospy.init_node('patrol')

        #Crea un cliente simple action, y luego espera a que el servidor,
        # es decir, navegacion, este disponible para recibirlo
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Conectado.")

        while not rospy.is_shutdown():
            #Dentro del loop, hace for entre los puntos y los va enviando y esperando
            # a que se ejecuten
            for pose in waypoints:
                goal = goal_pose(pose, 45.0)
                rospy.loginfo("Enviando meta.")
                client.send_goal(goal)
                client.wait_for_result()
        rospy.loginfo("Se ha cerrado el nodo.")
    except rospy.ROSInterruptException:
        #rospy.loginfo("Programa interrumpido antes de completarse.")
        print("Programa interrumpido antes de completarse", file=sys.stderr)
        pass
