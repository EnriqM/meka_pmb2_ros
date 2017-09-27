#!/usr/bin/env python
"""
Package: nav_to_people

Created on 18 jul. 2017

Este nodo escucha a 2 tf, una fija que es /robot_link del Meka. La movil es la de 
la extremidad, donde tambien se parametrizara el offset de seguridad
Envia directamente al topico de la transformada inversa del robot

Creator: Enrique Ortega, from Universidad de Jaen

El servicio que se usara es el siguiente:

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
import std_msgs.msg 
from navegacion_Funciones import transformaciones_tf
from nav_to_people.srv import Permiso_Brazos, Permiso_BrazosResponse, Permiso_BrazosRequest

class meka_server(object):
    def __init__(self):
        
        self.iniciacion_meka()
        self.r = rospy.Rate(4) # 10hz
        self.s = rospy.Service('permiso_meka_arms', Permiso_Brazos, self.server_brazo)
        self.flag = False#Inicialmente estara sin permiso, por lo que no se mueve.
        print "Nodo_servidor de brazos del Meka iniciado"
        #self.run()

    def iniciacion_meka(self):
        rospy.init_node('meka_arm_directriz')
        #movename = rospy.get_param('~movframe')
        self.x = rospy.get_param('~x_offset')
        self.y = rospy.get_param('~y_offset')
        
        self.y_mimic_min = 0.1
        self.y_mimic_max = 0.5
        self.z_mimic = 0.3
        #self.moveframe = "/" + movename #DEPRECATED: Ahora se introduce mediante el servicio
        #self.vect_offset_left = [-x, y, 0.0]
        #self.vect_offset_right = [-x, -y, 0.0]
        
        self.listener = tf.TransformListener()    
        self.arm_right = rospy.Publisher("/command_frame_arm_right",
                                          std_msgs.msg.Float64MultiArray, queue_size=5)
        self.arm_left = rospy.Publisher("/command_frame_arm_left",
                                         std_msgs.msg.Float64MultiArray, queue_size=5)
        self.vector_multi = std_msgs.msg.Float64MultiArray()
        #listener.waitForTransform(moveframe, "/robot_link", rospy.Time(), rospy.Duration(10.0))#4.0
  
    def server_brazo(self, req):
        self.flag = req.interruptor
        if self.flag == True:
            self.modo = req.modo
            self.which_arm = req.brazo
            self.movename = "/" + req.frame_objetivo
            respuesta_mensaje = (" ACTIVADO con modo " + self.modo 
                                + " en el brazo " + self.which_arm +
                                " dirigido al frame " + self.movename)
            rospy.loginfo(rospy.get_caller_id() + respuesta_mensaje)
        else:
            respuesta_mensaje = " DESACTIVADO"
            rospy.loginfo(rospy.get_caller_id() + respuesta_mensaje)
        return Permiso_BrazosResponse(self.flag, respuesta_mensaje)
    
    def offset_mimica(self, vector, flag):
        #Flag a TRUE si izquierdo, FALSE si derecho
        final=[0.287, 0.0, 0.0]
        
        #Primero se limita la Z, que es comun
        if vector[2] < -self.z_mimic:
            final[2] = -self.z_mimic
        elif vector[2] > self.z_mimic:
            final[2] = self.z_mimic
        else:
            final[2] = vector[2]
        
        if flag == True:
            #Izquierdo
            if vector[1] < self.y_mimic_min:
                final[1] = self.y_mimic_min
            elif vector[1] > self.y_mimic_max:
                final[1] = self.y_mimic_max
            else:
                final[1] = vector[1]
        else:
            #Derecho, va al contrario (numero negativo)
            if vector[1] > -self.y_mimic_min:
                final[1] = -self.y_mimic_min
            elif vector[1] < -self.y_mimic_max:
                final[1] = -self.y_mimic_max
            else:
                final[1] = vector[1]
            
        return final
        
    def dar_mano(self):
        now = rospy.Time.now()
        final_trans = transformaciones_tf(self.movename, "/robot_link", now, self.listener)
        if self.which_arm == "left":
            final_trans += [-self.x, self.y, 0.0]
            self.vector_multi.data = final_trans#Los otros apartados se mandan vacios
            self.arm_left.publish(self.vector_multi)
        elif self.which_arm == "right":
            final_trans += [-self.x, -self.y, 0.0]
            self.vector_multi.data = final_trans
            self.arm_right.publish(self.vector_multi)
        else:
            print "ERROR"
            return -1#Error, no hay brazo introducido
        #self.r.sleep()
    
    def mimica(self):
        #print "Haciendo mimica"
        #now = rospy.Time.now()
        now = rospy.Time(0)
        transi = transformaciones_tf(self.movename, "/robot_link", now, self.listener) 
        if self.which_arm == "left":
            final_trans = self.offset_mimica(transi, True)
            #Flag a TRUE si izquierdo, FALSE si derecho
            self.vector_multi.data = final_trans#Los otros apartados no se rellenan
            print final_trans
            self.arm_left.publish(self.vector_multi)
        elif self.which_arm == "right":
            final_trans = self.offset_mimica(transi, False)
            print final_trans
            self.vector_multi.data = final_trans
            self.arm_right.publish(self.vector_multi)
        else:
            print "ERROR"
            return -1#Error, no hay brazo introducido
        #self.r.sleep()
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.flag and (self.modo == "dar_mano"):#Si fuera while no ve lo de fuera en el loop
                    self.dar_mano()
                    print "DANDO MANO"
                    
                elif self.flag and (self.modo == "mimica"):
                    self.mimica()
                    print "HACIENDO MIMICA"
                else:
                    print "Esperando peticion"
                #rospy.spin()
                self.r.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
                pass

if __name__ == "__main__":
    foobar = meka_server()
    foobar.run() #Sustituido porque se debe ejecutar en la instanciacion de la clase
