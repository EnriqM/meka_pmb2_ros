#!/usr/bin/env python
"""
Package: robot_follower

Created on 26 oct. 2017

@author: Enrique

Creator: Enrique Ortega, from Universidad de Jaen



 



El servicio que se usara es el siguiente:


bool interruptor
string frame_objetivo #el frame que se sigue, sin la /.
string frame_referencia #el frame que hace referencia, normalmente robot_link
---
bool success
string message


"""
import rospy
import smach
import smach_ros
import smach_funciones_kinect as sf


#from smach_funciones_demo1 import*
from time import sleep
from std_msgs.msg import Float64, Int32, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler#, euler_from_quaternion
from math import radians#, degrees


metros_r = 0.7
#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [(metros_r, 0.0, 0.0), (180.0, 0.0)], #Se coloca 
    [( 0.0, metros_r, 0.0), (270.0, 0.0)],
    [( 0.0, -metros_r, 0.0), (90.0, 0.0)], #Se coloca a su derecha (o izquierda?)
    [( -22.989, 31.568, 0.0), (10.0, 0.0)],#Posicion inicial en el mapa. Variable
    [( -17.502, 25.962, 0.0), (-27.0, 0.0)],#Posicion final en el mapa.
]
#Diccionario con los frames disponibles a los que navegar en TF:
frames = {'mapa':'map', 0:'carrot_0', 1:'carrot_1', 2:'carrot_2', 3:'carrot_3',
           'kinect':'carrot_kinect',  'relativo':'base_link'}

topic_num_tof = "people_track/found"
topic_num_kinect = "openni_tracker/kinect_tracked_users"
topic_names = "person_name" #Tipo String


class nodo_espera_tof(object):
    def __init__(self):
        self.value = 0
        self.numero_deteccion = 1
        #rospy.init_node('nodo_espera_tof') Comentado ya que el nodo lo inicia el propio SMACH.
        
        rospy.Subscriber(topic_num_tof, Int32, self.update_value)
    def update_value(self, msg):
        self.value = msg.data
        #print self.value

    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown() puesto para que se pueda salir del script adecuadamente.
        while self.value < self.numero_deteccion and not(rospy.is_shutdown()):
            #self.pub.publish(self.value)
            print "Esperando a que aparezca una persona en la escena."
            r.sleep()

class nodo_mide_velocidad(object):
    def __init__(self):
        self.value = 10 #Un numero alto al principio, para que no detecte una velocidad baja.
        self.tolerancia_velocidad = 0.15
        self.persona = 0 #Esta linea se comentaria en el futuro para que admita mas personas el nodo de espera de velocidad
        nombre_topic = "human_" + str(self.persona) + "_speed"
        rospy.Subscriber(nombre_topic, Float64, self.update_value)

    def update_value(self, msg):
        self.value = msg.data
        #print self.value

    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown() puesto para que se pueda salir del script adecuadamente.
        while self.value > self.tolerancia_velocidad and not(rospy.is_shutdown()):
            #self.pub.publish(self.value)
            print "Esperando a que la persona reduzca velocidad lineal debajo de la tolerancia indicada."
            r.sleep()


class nodo_espera_calibracion(object):
    def __init__(self):
        self.value = []
        self.valor_final = 0
        self.numero_deteccion = 1
        rospy.Subscriber(topic_num_kinect, String, self.update_value)
        

    def update_value(self, msg):
        if len(msg.data) == 0:
            self.value = []
        else:
            #Se pone el segundo caracter porque el primero es un espacio en blanco
            self.value = msg.data[1] 
            self.valor_final = int(self.value)
        #print self.value
        
    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown() puesto para que se pueda salir del script adecuadamente.
        while len(self.value) == 0 and not(rospy.is_shutdown()):
            #self.pub.publish(self.value)
            print "Esperando a la calibracion"
            r.sleep()

class nodo_obtiene_nombres(object):
    def __init__(self):
        self.nombre = ""
        rospy.Subscriber(topic_names, String, self.update_value)

    def update_value(self, msg):
        self.nombre = msg.data

    def run(self):
        r = rospy.Rate(10)
        while len(self.nombre) == 0 and not(rospy.is_shutdown()):
            r.sleep()
            
class espera_aparicion(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['detected', 'last_person'])
        self.clase = nodo_espera_tof()
        
    def execute(self, userdata):
               
        self.clase.value = 0
        self.clase.run()
        
        sleep(1) #Espera para que le de tiempo al medidor de velocidad
                
        return 'detected'
    
class medidor_velocidad(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stopped'])
        #self.clase = nodo_mide_velocidad()
        
    def execute(self, userdata):
        rospy.loginfo('Comenzando con la bienvenida')
        sf.habla("Hello everyone. I'm going to give you a welcome", 1.5)
                        
        return 'stopped'

class calibracion_kinect(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['calibrated', 'calibration_error'],
                             input_keys=['cal_counter_in'],
                             output_keys=['cal_counter_out'])
        self.clase = nodo_espera_calibracion()
        
        
        
    def execute(self, userdata):
        rospy.loginfo('Navegacion hasta la persona. Paso a la calibracion.')
        sf.habla("Please, put your hands up to recognize you", 1.5)
        
        self.clase.run()
        sleep(0.5)
        
        userdata.cal_counter_out = self.clase.valor_final
        
        return 'calibrated'

    
    '''
    Despues queda todo el tema de la interaccion con el Meka. 
    Gracias a que es un server externo, se puede hacer un sleep
    para que interactue durante un tiempo.
    '''

class interaccion_meka(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['welcomed', 'failed'],
                             input_keys=['inter_counter_in'])
        self.frase = "Recognized user. Welcome, "
        #self.nombre = "Enrique"
        self.clase = nodo_obtiene_nombres()
        self.flag = True
        
    def execute(self, userdata):
        rospy.loginfo('Persona calibrada. Pasando a interaccion humana.')
        numTF = userdata.inter_counter_in
        self.clase.run()
        sf.habla(self.frase + self.clase.nombre, 3.0)
        sf.habla("Lets say hello, " + self.clase.nombre, 0.1)
        print "El numero para el TF es " + str(numTF)
        
        if self.flag: #self.clase.nombre == "Enrique" or self.clase.nombre == "Jose":
            sf.cliente_meka(True, "mimica", "left", "left_hand_" + str(numTF))
        else:
            sf.cliente_meka(True, "dar_mano", "left", "right_hand_"+ str(numTF))
                   
        
      
        rospy.sleep(15.0)
        sf.cliente_meka(False, "mimica", "left", "left_hand_"+ str(numTF))
        
        sf.habla("Goodbye, " + self.clase.nombre, 1.0)
        return 'welcomed'
    
class interaccion_kinect(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['welcomed', 'failed'],
                             input_keys=['inter_counter_in'])
        self.frase = "Recognized user. Welcome, "
        self.clasenombre = "Enrique"
        #self.clase = nodo_obtiene_nombres()
        
    def execute(self, userdata):
        rospy.loginfo('Persona calibrada. Pasando a interaccion humana.')
        numTF = str(userdata.inter_counter_in)
        nombre_tf = "torso_" + numTF 
        #self.clase.run()
        sf.habla(self.frase + self.clasenombre, 3.0)
        sf.habla("I am going to follow you, " + self.clasenombre, 1.0)
        print "El numero para el TF es " + numTF
        
        sf.cliente_kinect(True, nombre_tf, "robot_link")
        '''
        if self.flag: #self.clase.nombre == "Enrique" or self.clase.nombre == "Jose":
            sf.cliente_meka(True, "mimica", "left", "left_hand_" + str(numTF))
        else:
            sf.cliente_meka(True, "dar_mano", "left", "right_hand_"+ str(numTF))
        '''      
        
      
        #rospy.sleep(15.0)
        #sf.cliente_meka(False, "mimica", "left", "left_hand_"+ str(numTF))
        
        #sf.habla("Goodbye, " + self.clase.nombre, 1.0)
        return 'welcomed'


def main():
    rospy.init_node('follow_and_navigate_smach')

    # Create a SMACH state machine
    '''
    sm = smach.StateMachine(outcomes=['detected', 'last_person', 'stopped', 'succeeded','aborted',
                                      'preempted', 'ended', 'failed','calibrated',
                                       'calibration_error', 'welcomed'])
    '''
    sm = smach.StateMachine(outcomes=['succeeded','aborted',
                                      'preempted', 'ended', 'failed'])
    # Inicializamos el contador a 0 en la propia maquina de estados.
    sm.userdata.sm_counter = 0#Cambiaremos este valor para admitir mas personas en el ciclo
    sm.userdata.calibracion_counter = 0#Inicializamos el valor a un numero

    # Open the container
    with sm:
        def nav_goal_cb(pose, frame_objetivo):
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = frame_objetivo
            goal_pose.target_pose.pose.position.x = pose[0][0]
            goal_pose.target_pose.pose.position.y = pose[0][1]
            goal_pose.target_pose.pose.position.z = pose[0][2]
    
            #Orientacion del robot esta expresada en el valor yaw de angulos Euler
            angle = radians(pose[1][0])
            quat = quaternion_from_euler(0.0, 0.0, angle)
            goal_pose.target_pose.pose.orientation = Quaternion(*quat.tolist())

            return goal_pose
            
        # Add states to the container
        
        #Tambien remapea de la state machine a cada una de las states.

        '''
        smach.StateMachine.add('ESPERA_APARICION', espera_aparicion(), 
                               transitions={'detected':'MEDIDOR_VELOCIDAD', 
                                            'last_person':'ended'})
        
        smach.StateMachine.add('MEDIDOR_VELOCIDAD', medidor_velocidad(), 
                               transitions={'stopped':'NAV_GOAL0'})
        
    
        smach.StateMachine.add('NAV_GOAL0',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[3], 
                                                         sf.frame_contador(frames, "mapa"))),
                      transitions={'succeeded':'NAV_GOAL1',
                                   'aborted':'failed'})
        smach.StateMachine.add('NAV_GOAL1',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[4], 
                                                         sf.frame_contador(frames, "mapa"))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'NAV_GOAL1'})
    
        
        smach.StateMachine.add('NAV_GOAL2',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[2], 
                                                         sf.frame_contador(frames, sm.userdata.sm_counter))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'failed'})
        '''
        
        #torso_
        smach.StateMachine.add('CALIBRACION', calibracion_kinect(), 
                               transitions={'calibrated':'INTERACCION', 
                                            'calibration_error':'failed'},
                               remapping={'cal_counter_in':'calibracion_counter', 
                                          'cal_counter_out':'calibracion_counter'})
        
        smach.StateMachine.add('INTERACCION', interaccion_kinect(), 
                               transitions={'welcomed':'NAV_GOAL0'},
                               remapping={'inter_counter_in':'calibracion_counter'})     
        
        #kinect
        '''
        smach.StateMachine.add('NAV_GOAL_MAP',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[3], 
                                                         sf.frame_contador(frames, "mapa"))),
                      transitions={'succeeded':'MEDIDOR_VELOCIDAD',
                                   'aborted':'failed'})
        '''
        smach.StateMachine.add('NAV_GOAL0',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[0], 
                                                         sf.frame_contador(frames, "kinect"))),
                      transitions={'succeeded':'NAV_GOAL0',
                                   'aborted':'NAV_GOAL1'})
        smach.StateMachine.add('NAV_GOAL1',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[1], 
                                                         sf.frame_contador(frames, "kinect"))),
                      transitions={'succeeded':'NAV_GOAL0',
                                   'aborted':'NAV_GOAL2'})
    
        
        smach.StateMachine.add('NAV_GOAL2',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[2], 
                                                         sf.frame_contador(frames, "kinect"))),
                      transitions={'succeeded':'NAV_GOAL0',
                                   'aborted':'NAV_GOAL0'})
      




    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    print "The app stopped. Stopping the kinect server too"
    sf.cliente_kinect(False, "whatever", "robot_link")


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            main()
        except rospy.ROSInterruptException:
            pass