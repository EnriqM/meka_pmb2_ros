#!/usr/bin/env python

"""
Package: nav_to_people

It will listen to 2 tf, one fixed and the other moving and it will broadcast 
a topic with the module of the speed.

Creator: Enrique Ortega, from Universidad de Jaen

"""
 
import rospy
import tf
import math
from std_msgs.msg import Float64

#import turtlesim.msg


if __name__ == '__main__':
    rospy.init_node('velocity_tf_module')
    fixedname = rospy.get_param('~fixframe')
    movename = rospy.get_param('~movframe')
    rospy.loginfo("Iniciando nodo de velocidad de humano.")
    fixedframe = "/" + fixedname
    moveframe = "/" + movename
    rospy.loginfo(fixedframe+moveframe+ " Ahora sin palito " + fixedname+ movename )
    
    
    listener = tf.TransformListener()
  
#lookupTwist(tracking_frame, observation_frame, time, averaging_interval) linear, angular
    
    human_vel = rospy.Publisher('human_speed', Float64, queue_size=10) #Cambiar para que sea parametrizable segun numero
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            
            (linear, angular)=listener.lookupTwist(moveframe, fixedframe, rospy.Time(0), rospy.Duration(0.2))
            
            resultado=math.sqrt(math.pow(linear[0], 2) + math.pow(linear[1], 2))
            #rospy.loginfo(str(resultado))
            human_vel.publish(Float64(resultado))
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass

