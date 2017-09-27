#!/usr/bin/env python

"""
Package: nav_to_people


Este nodo escucha a 2 tf, una fija y otra movil, que se parametrizan mediante
el launch. Creara un topico con el modulo de la velocidad.
Tambien crea un TF que se oriente hacia la camara, para enviar metas a ese topico

Creator: Enrique Ortega, from Universidad de Jaen


ANTES PONIA MOVENAME[10], ahora pone movename[-1:]
"""
 
import rospy
import tf
import math
import std_msgs.msg
from nav_to_people.msg import CoordXYZArray
#from navegacion_Funciones import calculo_angulo_tf
#from meka_arm_simple import transformaciones_tf
def calculo_angulo_tf(msg):
    num_per=int(movename[-1:]) #Esto es porque el vector de vectores se referencia al numero
    #La siguiente linea se puede descomentar para depurar
    #rospy.loginfo("msg in X is: %d" % (msg.coords[num_per].X_Coord))
    #rospy.loginfo("msg in Z is: %d" % (msg.coords[num_per].Z_Coord))
    angulo_rad= math.atan2(msg.coords[num_per].X_Coord, msg.coords[num_per].Z_Coord)
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, angulo_rad+3.14159265),
                     rospy.Time.now(),
                     "person_new_tf",
                     "person_tf_0")

#import turtlesim.msg


if __name__ == '__main__':
    rospy.init_node('velocity_tf_module')
    fixedname = rospy.get_param('~fixframe')
    movename = rospy.get_param('~movframe')
    #rospy.loginfo("Iniciando nodo de velocidad de humano.")
    fixedframe = "/" + fixedname
    moveframe = "/" + movename
    #rospy.loginfo(fixedframe+moveframe+ " Ahora sin palito " + fixedname+ movename )
    newname = "person_new_" + movename[-1:]
    newtopic = "human_" + movename[-1:] + "_speed"



    listener = tf.TransformListener()    
    human_vel = rospy.Publisher(newtopic, std_msgs.msg.Float64, queue_size=5) #Cambiar para que sea parametrizable segun numero
    rate = rospy.Rate(20.0)


    while not rospy.is_shutdown():
        try:
            #now = rospy.Time.now()
            #lookupTwist(tracking_frame, observation_frame, time, averaging_interval) linear, angular
            (linear, angular)=listener.lookupTwist(moveframe, fixedframe, rospy.Time(0), rospy.Duration(0.2)) #para calcular la velocidad de la persona          
            resultado=math.sqrt(math.pow(linear[0], 2) + math.pow(linear[1], 2))
            #rospy.loginfo(str(resultado))
            human_vel.publish(std_msgs.msg.Float64(resultado))
            rospy.Subscriber("SwissRanger/people_track/coords_centroids", CoordXYZArray, calculo_angulo_tf)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass
        
