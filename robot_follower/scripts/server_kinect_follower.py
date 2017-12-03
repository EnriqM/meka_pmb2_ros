#!/usr/bin/env python
"""
Package: robot_follower

Created on 26 oct. 2017

@author: Enrique

Creator: Enrique Ortega, from Universidad de Jaen



Este nodo escucha a 2 tf, una fija que es /robot_link del Meka. La movil es la del 
torso de la persona que sigue, cuyo frame es dado por la Kinect. Finalmente su salida es
un tercer TF de nombre invariable para que la navegacion la siga facilmente.



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


"""
 
import rospy
import tf
import math
#from nav_to_people.srv import Permiso_Brazos, Permiso_BrazosResponse, Permiso_BrazosRequest
from robot_follower.srv import NumeroTF, NumeroTFResponse, NumeroTFRequest


class kinect_server(object):
    def __init__(self):
        
        self.iniciacion()
        self.r = rospy.Rate(10.0) # 10hz
        #self.s = rospy.Service('permiso_meka_arms', Permiso_Brazos, self.server_brazo)
        self.s = rospy.Service('permiso_kinect', NumeroTF, self.server_kinect)
        self.flag = False#Inicialmente estara sin permiso, por lo que no se mueve.
        print "Nodo_servidor de Kinect iniciado"
        #self.run()
    
    def transformaciones_tf(self, move, fix, tiempo, listener):
        (trans,rot) = listener.lookupTransform(move, fix, tiempo)# En lugar de now usabamos: rospy.Time(0)
        trans  = tf.transformations.translation_matrix(trans)
        rot =  tf.transformations.quaternion_matrix(rot)
        transform = tf.transformations.concatenate_matrices(trans, rot)
        inversed_transform = tf.transformations.inverse_matrix(transform)
        #rotation = tf.transformations.rotation_from_matrix(inversed_transform)
        return tf.transformations.translation_from_matrix(inversed_transform)


    def iniciacion(self):
        rospy.init_node('server_kinect')
        #movename = rospy.get_param('~movframe')
        
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()    
        '''
        self.arm_right = rospy.Publisher("/command_frame_arm_right",
                                          std_msgs.msg.Float64MultiArray, queue_size=5)
        self.arm_left = rospy.Publisher("/command_frame_arm_left",
                                         std_msgs.msg.Float64MultiArray, queue_size=5)
        self.vector_multi = std_msgs.msg.Float64MultiArray()
        #listener.waitForTransform(moveframe, "/robot_link", rospy.Time(), rospy.Duration(10.0))#4.0
        '''
    def server_kinect(self, req):
        self.flag = req.interruptor
        if self.flag == True:
            #self.modo = req.modo
            #self.which_arm = req.brazo
            self.movename = "/" + req.frame_objetivo
            self.fixedname = "/" + req.frame_referencia
            self.fixedname_no = req.frame_referencia
            respuesta_mensaje = (" ACTIVADO con move: " + self.movename 
                                + " y fixed: " + self.fixedname)
            rospy.loginfo(rospy.get_caller_id() + respuesta_mensaje)
        else:
            respuesta_mensaje = " DESACTIVADO"
            rospy.loginfo(rospy.get_caller_id() + respuesta_mensaje)
        return NumeroTFResponse(self.flag, respuesta_mensaje)
    
    def tf_list_broad(self):
        now = rospy.Time(0)
        final_trans = self.transformaciones_tf(self.movename, self.fixedname, now, self.listener)
    
        angulo_rad= math.atan2(final_trans[1], final_trans[0]) 
        #angulo_deg = math.degrees(angulo_rad)
        #print "El angulo es: "
        #print angulo_deg
        rotacion = tf.transformations.quaternion_from_euler(0.0, 0.0, (math.radians(180.0) + angulo_rad))                   
        #print "El quaternion es"  
        #print rotacion
        self.br.sendTransform((final_trans[0], final_trans[1], 0.0),
                (rotacion[0], rotacion[1], rotacion[2], rotacion[3]),
                #0.0, 0.0, 0.0, 1.0),
                #quat,
                rospy.Time.now(),
                "carrot_kinect",
                self.fixedname_no)
        #rate.sleep()
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.flag:#Si fuera while no ve lo de fuera en el loop
                    self.tf_list_broad()
                    print "ENVIANDO TF"
                else:
                    print "Esperando peticion"
                #rospy.spin()
                self.r.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
                pass

if __name__ == "__main__":
    foobar = kinect_server()
    foobar.run() #Sustituido porque se debe ejecutar en la instanciacion de la clase

