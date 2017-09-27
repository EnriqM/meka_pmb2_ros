#!/usr/bin/env python
'''
Created on 15 jul. 2017

@author: Enrique Ortega
'''

import rospy
from std_msgs.msg import String
#from rospy_tutorials.srv import *
from std_srvs.srv import SetBool, SetBoolResponse#, SetBoolRequest
from time import sleep



class talker_server(object):
    def __init__(self):
        
        rospy.init_node('talker_server', anonymous=True)
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        self.s = rospy.Service('set_booleano', SetBool, self.activacion)
        self.flag = False
        print "Nodo_servidor iniciado"
        #self.run()

        

    def activacion(self, req):
    
        if req.data == True:
            self.flag = True
            rospy.loginfo(rospy.get_caller_id() + "Recibida peticion de True")
            return SetBoolResponse(True, "Activado")
        else:
            self.flag = False
            rospy.loginfo(rospy.get_caller_id() + "Recibida peticion de False")
            return SetBoolResponse(False, "Desactivado")
    
    def talker(self):
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
        #self.r.sleep()
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.flag:#Si fuera while no ve lo de fuera en el loop
                    self.talker()
                #rospy.spin()
                self.r.sleep()
            except rospy.ROSInterruptException:
                pass

if __name__ == "__main__":
    habla = talker_server()
    print 'Espera de 3 segundos'
    sleep(3)
    print 'A la espera de una peticion'
    habla.run()
    

