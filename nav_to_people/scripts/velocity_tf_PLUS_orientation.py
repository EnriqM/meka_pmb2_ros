#!/usr/bin/env python

"""
Package: nav_to_people

It will listen to 2 tf, one fixed and the other moving and it will broadcast 
a topic with the module of the speed.

@Author: Enrique Ortega, from Universidad de Jaen

"""
 
import rospy
import tf
import math
from std_msgs.msg import Float64
#import tf_conversions.posemath as pm
from geometry_msgs.msg import Pose
#from nav_to_people.msg import CoordXYZArray

#import turtlesim.msg


if __name__ == '__main__':
    rospy.init_node('velocity_tf_module')
    fixedname = rospy.get_param('~fixframe')
    movename = rospy.get_param('~movframe')
    rospy.loginfo("Iniciando nodo de velocidad de humano.")
    fixedframe = "/" + fixedname
    moveframe = "/" + movename
    newtopic = "human_" + movename[-1:] + "_speed"
    newTF= "carrot_" + movename[-1:]
    
    
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    msg = Pose()
#lookupTwist(tracking_frame, observation_frame, time, averaging_interval) linear, angular
    
    human_vel = rospy.Publisher(newtopic, Float64, queue_size=10) #Cambiar para que sea parametrizable segun numero
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (linear, angular)=listener.lookupTwist(moveframe, fixedframe, rospy.Time(0), rospy.Duration(0.2)) #para calcular la velocidad de la persona
            (trans,rot) = listener.lookupTransform(moveframe, fixedframe, rospy.Time(0))
            trans  = tf.transformations.translation_matrix(trans)
            rot =  tf.transformations.quaternion_matrix(rot)
            transform = tf.transformations.concatenate_matrices(trans, rot)
            inversed_transform = tf.transformations.inverse_matrix(transform)
            #rotation = tf.transformations.rotation_from_matrix(inversed_transform)
            final_trans = tf.transformations.translation_from_matrix(inversed_transform)
            
            angulo_rad= math.atan2(final_trans[1], final_trans[0]) 
            angulo_deg = math.degrees(angulo_rad)
            rotacion = tf.transformations.quaternion_from_euler(0.0, 0.0, (math.radians(180.0) + angulo_rad))
             
            
            br.sendTransform((final_trans[0], final_trans[1], 0.0),
                     (rotacion[0], rotacion[1], rotacion[2], rotacion[3]),
                     rospy.Time.now(),
                     newTF,
                     fixedname)

            resultado=math.sqrt(math.pow(linear[0], 2) + math.pow(linear[1], 2))
            #rospy.loginfo(str(resultado))
            human_vel.publish(Float64(resultado))
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass
        
