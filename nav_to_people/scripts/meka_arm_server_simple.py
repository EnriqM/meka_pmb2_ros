#!/usr/bin/env python
"""
Package: nav_to_people

Created on 18 jul. 2017

Este nodo escucha a 2 tf, una fija que es /robot_link del Meka. La movil es la de 
la extremidad, donde tambien se parametrizara el offset de seguridad
Envia directamente al topico de la transformada inversa del robot

Creator: Enrique Ortega, from Universidad de Jaen

"""
 
import rospy
import tf
import std_msgs.msg 
from navegacion_Funciones import transformaciones_tf
from std_srvs.srv import SetBool, SetBoolResponse#, SetBoolRequest

class meka_server(object):
    def __init__(self):
        
        self.iniciacion_meka()
        #self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.r = rospy.Rate(4) # 10hz
        self.s = rospy.Service('set_booleano', SetBool, self.activacion)
        self.flag = False
        print "Nodo_servidor iniciado"
        self.run()

    def iniciacion_meka(self):
        rospy.init_node('meka_arm_directriz')
        movename = rospy.get_param('~movframe')
        which_arm = rospy.get_param('~arm') #Solo sera LEFT o RIGHT
        x = rospy.get_param('~x_offset')
        y = rospy.get_param('~y_offset')
        self.moveframe = "/" + movename
        newtopic = "/command_frame_arm_" + which_arm
        if which_arm == "left":
            self.vect_offset = [-x, y, 0.0]
        else:
            self.vect_offset = [-x, -y, 0.0]
    
        self.listener = tf.TransformListener()    
        self.arm_position = rospy.Publisher(newtopic, std_msgs.msg.Float64MultiArray, queue_size=5)
        self.vector_multi = std_msgs.msg.Float64MultiArray()
        #listener.waitForTransform(moveframe, "/robot_link", rospy.Time(), rospy.Duration(10.0))#4.0

    def activacion(self, req):
    
        if req.data == True:
            self.flag = True
            rospy.loginfo(rospy.get_caller_id() + "Recibida peticion de True")
            return SetBoolResponse(True, "Activado")
        else:
            self.flag = False
            rospy.loginfo(rospy.get_caller_id() + "Recibida peticion de False")
            return SetBoolResponse(False, "Desactivado")
    
    def move_1(self):
        now = rospy.Time.now()
        #listener.waitForTransform(moveframe, "/robot_link", now, rospy.Duration(10.0))#estaba a 4 segundos la espera maxima
        final_trans = transformaciones_tf(self.moveframe, "/robot_link", now, self.listener)
        final_trans += self.vect_offset
        print final_trans
        self.vector_multi.data = final_trans#Los otros apartados no se rellenan
        self.arm_position.publish(self.vector_multi)
        #self.r.sleep()
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.flag:#Si fuera while no ve lo de fuera en el loop
                    self.move_1()
                #rospy.spin()
                self.r.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
                pass

if __name__ == "__main__":
    foobar = meka_server()
    #foobar.run() #Sustituido porque se debe ejecutar en la instanciacion de la clase
    