#!/usr/bin/env python
'''
Created on 16 ago. 2017

@author: Enrique Ortega

BASADO EN: cliente_meka, wait_and_navigate y passingdata_smach_service1.


    
'''




import rospy
import smach
import smach_ros
import smach_funciones_demo1 as sf


#from smach_funciones_demo1 import*
from time import sleep
from std_msgs.msg import Float64, Int32, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler#, euler_from_quaternion
from math import radians#, degrees



#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [( 1.3, 0.0, 0.0), (180.0, 0.0)], #Se coloca 
    [( 0.0, 1.3, 0.0), (270.0, 0.0)],
    [( 0.0, -1.3, 0.0), (90.0, 0.0)], #Se coloca a su derecha (o izquierda?)
    [( -45.872, -10.063, 0.0), (10.0, 0.0)]#Posicion inicial en el mapa. Variable
]
#Diccionario con los frames disponibles a los que navegar en TF:
frames = {'mapa':'map', 0:'carrot_0', 1:'carrot_1', 2:'carrot_2', 3:'carrot_3', 'relativo':'base_link'}

topic_num_tof = "people_track/found"
topic_num_kinect = "openni_tracker/kinect_tracked_users"
topic_names = "person_name" #Tipo String


class nodo_espera_tof(object):
    def __init__(self):
        self.value = 0
        self.numero_deteccion = 1
        #rospy.init_node('nodo_espera_tof') Comentado ya que el nodo lo inicia el propio SMACH.
        #self.pub = rospy.Publisher('/value', Int32, latch=True)
        rospy.Subscriber(topic_num_tof, Int32, self.update_value)#CAMBIA!!!!!!!

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
        #rospy.init_node('nodo_espera_tof') Comentado ya que el nodo lo inicia el propio SMACH.
        #self.pub = rospy.Publisher('/value', Int32, latch=True)
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
        rospy.Subscriber(topic_names, String, self.update_value)#CAMBIA!!!!!!!

    def update_value(self, msg):
        self.nombre = msg.data

    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown() puesto para que se pueda salir del script adecuadamente.
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
        self.clase = nodo_mide_velocidad()
        
    def execute(self, userdata):
        rospy.loginfo('Persona 1 detectada. Midiendo velocidad de la persona 1.')
        self.clase.run()
        sleep(0.5)
        sf.habla("Me dirijo a saludarle. Porr favorr, espere.", 1.5)
                        
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
        sf.habla("Porr favorr, suba las manos para que pueda rreconocerle.", 1.5)
        
        self.clase.run()
        sleep(0.5)
        
        userdata.cal_counter_out = self.clase.valor_final
        
        return 'calibrated'
    
        #if False:#Nunca ocurre
        #    return 'calibration_error'
        


    
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
        #self.clase = nodo_mide_velocidad()
        self.frase = "Usuario reconocido. Bienvenido, "
        #self.nombre = "Enrique"
        self.clase = nodo_obtiene_nombres()
        
    def execute(self, userdata):
        rospy.loginfo('Persona calibrada. Pasando a interaccion humana.')
        numTF = userdata.inter_counter_in
        self.clase.run()
        sf.habla(self.frase + self.clase.nombre, 2.0)
        sf.habla("Vamos a saludarnos, " + self.clase.nombre, 0.1)
        print "El numero para el TF es " + str(numTF)
        
        if self.clase.nombre == "Enrique" or self.clase.nombre == "Jose":
            sf.cliente_meka(True, "mimica", "left", "left_hand_" + str(numTF))
        else:
            sf.cliente_meka(True, "dar_mano", "left", "right_hand_"+ str(numTF))
                   
        
      
        rospy.sleep(13.0)
        sf.cliente_meka(False, "mimica", "left", "left_hand_"+ str(numTF))
        
        sf.habla("Me voy. Adios, " + self.clase.nombre, 1.0)
        return 'welcomed'
    
    
    


def main():
    rospy.init_node('wait_and_navigate_smach')

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
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'NAV_GOAL0'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})
                               
                               class espera_aparicion(smach.State):
        '''
        
        
        smach.StateMachine.add('ESPERA_APARICION', espera_aparicion(), 
                               transitions={'detected':'MEDIDOR_VELOCIDAD', 
                                            'last_person':'ended'})
        smach.StateMachine.add('MEDIDOR_VELOCIDAD', medidor_velocidad(), 
                               transitions={'stopped':'NAV_GOAL0'})
        
        smach.StateMachine.add('NAV_GOAL0',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[0], 
                                                         sf.frame_contador(frames, sm.userdata.sm_counter))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'NAV_GOAL1'})
        smach.StateMachine.add('NAV_GOAL1',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[1], 
                                                         sf.frame_contador(frames, sm.userdata.sm_counter))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'NAV_GOAL2'})
        smach.StateMachine.add('NAV_GOAL2',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[2], 
                                                         sf.frame_contador(frames, sm.userdata.sm_counter))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'failed'})
        
        
        smach.StateMachine.add('CALIBRACION', calibracion_kinect(), 
                               transitions={'calibrated':'INTERACCION', 
                                            'calibration_error':'failed'},
                               remapping={'cal_counter_in':'calibracion_counter', 
                                          'cal_counter_out':'calibracion_counter'})
        
        smach.StateMachine.add('INTERACCION', interaccion_meka(), 
                               transitions={'welcomed':'NAV_GOAL_MAP'},
                               remapping={'inter_counter_in':'calibracion_counter'})     
        
        
        smach.StateMachine.add('NAV_GOAL_MAP',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[3], 
                                                         sf.frame_contador(frames, "mapa"))),
                      transitions={'succeeded':'ESPERA_APARICION',
                                   'aborted':'failed'})
        
        
      




    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            main()
        except rospy.ROSInterruptException:
            pass

aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa meka_server_duo aaaaaaaaa
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

aaaaaaaaaaaaaaaaaaa PLUS_ ORIENTATION aaaaaaaaaaaaaaaaaaaaaa

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
    #rospy.loginfo(fixedframe+moveframe+ " Ahora sin palito " + fixedname+ movename )
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
            #now = rospy.Time.now()
            #listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
            (linear, angular)=listener.lookupTwist(moveframe, fixedframe, rospy.Time(0), rospy.Duration(0.2)) #para calcular la velocidad de la persona
            #(linear, angular)=listener.lookupTwistFull(moveframe, fixedframe, fixedframe, [0, 0, 0], fixedframe, rospy.Time(0), rospy.Duration(0.2))
            #rospy.loginfo(str(linear[1]))
            #print "El linear es: "
            #print linear
            #print "El angular es: "
            #print angular
            (trans,rot) = listener.lookupTransform(moveframe, fixedframe, rospy.Time(0))
            trans  = tf.transformations.translation_matrix(trans)
            rot =  tf.transformations.quaternion_matrix(rot)
            transform = tf.transformations.concatenate_matrices(trans, rot)
            inversed_transform = tf.transformations.inverse_matrix(transform)
            #rotation = tf.transformations.rotation_from_matrix(inversed_transform)
            final_trans = tf.transformations.translation_from_matrix(inversed_transform)
            
            angulo_rad= math.atan2(final_trans[1], final_trans[0]) 
            angulo_deg = math.degrees(angulo_rad)
            #print "El angulo es: "
            #print angulo_deg
            rotacion = tf.transformations.quaternion_from_euler(0.0, 0.0, (math.radians(180.0) + angulo_rad))
             
                    
            #print "El quaternion es"
            
            #print rotacion
            br.sendTransform((final_trans[0], final_trans[1], 0.0),
                     (rotacion[0], rotacion[1], rotacion[2], rotacion[3]),
                     #0.0, 0.0, 0.0, 1.0),
                     #quat,
                     rospy.Time.now(),
                     newTF,
                     fixedname)

            resultado=math.sqrt(math.pow(linear[0], 2) + math.pow(linear[1], 2))
            #rospy.loginfo(str(resultado))
            human_vel.publish(Float64(resultado))
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException):
            pass
        
aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa heibus.py aaaaaaaaaaaaaaaaaaaaaaa


#!/usr/bin/env python
'''
Created on 16 ago. 2017

@author: Enrique Ortega

BASADO EN: cliente_meka, wait_and_navigate y passingdata_smach_service1.

Descripcion: SMACH que realiza todo el caso final
Requiere el funcionamiento del proyecto de fusion sensorial

    
'''




import rospy
import smach
import smach_ros
import smach_funciones_demo1 as sf


#from smach_funciones_demo1 import*
from time import sleep
from std_msgs.msg import Float64, Int32, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler#, euler_from_quaternion
from math import radians#, degrees



#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [( 1.3, 0.0, 0.0), (180.0, 0.0)], #Se coloca 
    [( 0.0, 1.3, 0.0), (270.0, 0.0)],
    [( 0.0, -1.3, 0.0), (90.0, 0.0)], #Se coloca a su derecha (o izquierda?)
    [( -22.989, 31.568, 0.0), (10.0, 0.0)],#Posicion inicial en el mapa. Variable
    [( -17.502, 25.962, 0.0), (-27.0, 0.0)],#Posicion final en el mapa.
]
#Diccionario con los frames disponibles a los que navegar en TF:
frames = {'mapa':'map', 0:'carrot_0', 1:'carrot_1', 2:'carrot_2', 3:'carrot_3', 'relativo':'base_link'}

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
    
    
    


def main():
    rospy.init_node('wait_and_navigate_smach2')

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
	'''
	
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
	
	'''
        smach.StateMachine.add('NAV_GOAL2',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[2], 
                                                         sf.frame_contador(frames, sm.userdata.sm_counter))),
                      transitions={'succeeded':'CALIBRACION',
                                   'aborted':'failed'})
	'''
        
        
        smach.StateMachine.add('CALIBRACION', calibracion_kinect(), 
                               transitions={'calibrated':'INTERACCION', 
                                            'calibration_error':'failed'},
                               remapping={'cal_counter_in':'calibracion_counter', 
                                          'cal_counter_out':'calibracion_counter'})
        
        smach.StateMachine.add('INTERACCION', interaccion_meka(), 
                               transitions={'welcomed':'NAV_GOAL_MAP'},
                               remapping={'inter_counter_in':'calibracion_counter'})     
        
        
        smach.StateMachine.add('NAV_GOAL_MAP',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[3], 
                                                         sf.frame_contador(frames, "mapa"))),
                      transitions={'succeeded':'MEDIDOR_VELOCIDAD',
                                   'aborted':'failed'})
        
        
      




    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            main()
        except rospy.ROSInterruptException:
            pass




