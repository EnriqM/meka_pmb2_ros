#!/usr/bin/env python

"""
Package: nav_to_people


Este nodo escucha a 2 tf, una fija que es /robot_link del Meka. La movil es la de 
la extremidad, donde tambien se parametrizara el offset de seguridad
Envia directamente al topico de la transformada inversa del robot

Creator: Enrique Ortega, from Universidad de Jaen

"""
 
import rospy
import tf
import std_msgs.msg 
from navegacion_Funciones import transformaciones_tf


if __name__ == '__main__':
    rospy.init_node('meka_arm_directriz')
    movename = rospy.get_param('~movframe')
    which_arm = rospy.get_param('~arm') #Solo sera LEFT o RIGHT
    x = rospy.get_param('~x_offset')
    y = rospy.get_param('~y_offset')
    moveframe = "/" + movename
    newtopic = "/command_frame_arm_" + which_arm
    if which_arm == "left":
        vect_offset = [-x, y, 0.0]
    else:
        vect_offset = [-x, -y, 0.0]
    
    listener = tf.TransformListener()    
    arm_position = rospy.Publisher(newtopic, std_msgs.msg.Float64MultiArray, queue_size=5)
    rate = rospy.Rate(1.0)#20.0
    vector_multi = std_msgs.msg.Float64MultiArray()
    #listener.waitForTransform(moveframe, "/robot_link", rospy.Time(), rospy.Duration(10.0))#4.0

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            #listener.waitForTransform(moveframe, "/robot_link", now, rospy.Duration(10.0))#estaba a 4 segundos la espera maxima
            final_trans = transformaciones_tf(moveframe, "/robot_link", now, listener)
            final_trans += vect_offset
            print final_trans
            vector_multi.data = final_trans
            arm_position.publish(vector_multi)
            
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass
