#!/usr/bin/env python

"""
Package: nav_to_people


Este nodo escucha a 3 tf,un mapa (o world), una fija y otra movil, que se parametrizan mediante
el launch. Creara un topico con el modulo de la velocidad.
Tambien crea un TF que se oriente hacia la camara, adaptado para enviar los 'pose'
o destinos de la navegación correctamente.

Requiere especialmente de navegacion_Funciones, creado en el mismo paquete.

Creator: Enrique Ortega, from Universidad de Jaen


ANTES PONIA movename[10], ahora pone movename[-1:]
"""
 
import rospy
import tf
import std_msgs.msg
#import navegacion_Funciones as nF
from navegacion_Funciones import velocidad_humano
from navegacion_Funciones import transforaciones_tf_clase as tfh



if __name__ == '__main__':
    rospy.init_node('velocity_tf_module')
    fixedname = rospy.get_param('~fixframe')
    movename = rospy.get_param('~movframe')
    #rospy.loginfo("Iniciando nodo de velocidad de humano.")
    fixedframe = "/" + fixedname
    moveframe = "/" + movename
    #rospy.loginfo(fixedframe+moveframe+ " Ahora sin palito " + fixedname+ movename )
    newname = "person_tf_" + movename[-1:]
    newtopic = "human_" + movename[-1:] + "_speed"
    
    
    listener = tf.TransformListener()  
    br = tf.TransformBroadcaster()  
    human_vel = rospy.Publisher(newtopic, std_msgs.msg.Float64, queue_size=10) #Cambiar para que sea parametrizable segun numero
    rate = rospy.Rate(20.0)


    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
           
            velocidad = velocidad_humano(moveframe, fixedframe, now, 0.4, listener)
            human_vel.publish(std_msgs.msg.Float64(velocidad))
            
            
            yaw_rad = tfh.yaw(moveframe, fixedframe, now, True)
            (roll, pitch, Z) = tfh.trans_human_camera(moveframe, now, listener)
            
            rotacion = tf.transformations.quaternion_from_euler(-roll, -pitch, yaw_rad)
            br.sendTransform((0.0, 0.0, -Z),
                     (rotacion[0], rotacion[1], rotacion[2], rotacion[3]),
                     rospy.Time.now(),
                     newname,
                     movename)
            
            
            
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass
