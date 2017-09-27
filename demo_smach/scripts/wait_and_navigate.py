#!/usr/bin/env python
'''
Created on 20 jul. 2017

@author: Enrique Ortega


Descripcion: script que usa SMACH para testear que espera 
a un valor determinado de un topico y luego manda una meta en la
navegacion autonoma
'''




import rospy
import smach
import smach_ros
from time import sleep

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler#, euler_from_quaternion
from math import radians#, degrees
from sound_play.libsoundplay import SoundClient

#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [( 0.5, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)],
    [( -1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 180.0)]
]
#from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


from std_msgs.msg import Int32

def habla(frase, espera):
    #Dice la frase mediante el nodo SoundPlay.
    #La espera se puede reconfigurar como parametro de entrada
    soundhandle = SoundClient()
    rospy.sleep(0.1)
    voice = 'voice_kal_diphone'

    print 'Saying: %s' % frase
    print 'Voice: %s' % voice

    soundhandle.say(frase, voice)
    rospy.sleep(espera)
    

class Nodo_suma(object):
    def __init__(self):
        self.value = 0

        #rospy.init_node('Nodo_sumaer')
        self.pub = rospy.Publisher('/value', Int32, latch=True)
        rospy.Subscriber('/value', Int32, self.update_value)

    def update_value(self, msg):
        self.value = msg.data + 1
        print self.value

    def run(self):
        r = rospy.Rate(10)
        #while not rospy.is_shutdown():
        while not self.value == 10:
            self.pub.publish(self.value)
            r.sleep()


# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO. Solo suma uno al counter que recibe')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            sleep(1)
            return 'outcome1'
        else:
            sleep(1)
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        self.clase = Nodo_suma()
        self.frase = "Usuario reconocido. Bienvenido, "
        self.nombre = "Enrique"
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR. Solo recibe el contador de FOO y ya, devuelve el outcome1')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)  #Aqui es donde se ve el contador para rospy.loginfo
        
        self.clase.value = 0
        self.clase.run()
        
        sleep(0.1)
        '''
        NUEVO: PROBANDO EL SAY.PY
        '''
        
        
        
        
        habla(self.frase + self.nombre, 2.0)
        
        
        
        
        return 'outcome1'
        




def main():
    rospy.init_node('wait_and_navigate_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'succeeded','aborted','preempted', 'ended'])
    # Inicializamos el contador a 0 en la propia maquina de estados.
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        def nav_goal_cb(pose):
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'base_link'
            goal_pose.target_pose.pose.position.x = pose[0][0]
            goal_pose.target_pose.pose.position.y = pose[0][1]
            goal_pose.target_pose.pose.position.z = pose[0][2]
    
            #Orientacion del robot esta expresada en el valor yaw de angulos Euler
            angle = radians(pose[1][3])
            quat = quaternion_from_euler(0.0, 0.0, angle)
            goal_pose.target_pose.pose.orientation = Quaternion(*quat.tolist())

            return goal_pose
            
        # Add states to the container
        
        #Tambien remapea de la state machine a cada una de las states.
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'NAV_GOAL'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})
        smach.StateMachine.add('NAV_GOAL',
                      smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=nav_goal_cb(waypoints[0])),
                      transitions={'succeeded':'ended'})
        


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
        
    





    
'''
#!/usr/bin/env python


Script modificado de patrol.py (el que no usa smach), para dar metas 
respecto el mapa

Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal

Actual robo pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped

SACADO DE: answers.ros.org/question/185907/hel-with-some-code-send-a-navigation-goal/


from __future__ import print_function
import rospy
import actionlib


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees


#Lista de puntos para que el robot "patrulle". Son dos, uno la posicion 
#   y otro es la orientacion
waypoints = [
    [( 1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)],
    [( -1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 180.0)]
]

#Funcion para convertir cada punto en un MoveBaseGoal 
#    Se podra poner frame_id 'map' o 'odom', 'base_link'...
def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'base_link'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    
    #Orientacion del robot esta expresada en el valor yaw de angulos Euler
    angle = radians(pose[1][3])
    quat = quaternion_from_euler(0.0, 0.0, angle)
    goal_pose.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return goal_pose


if __name__ == '__main__':
    try:
        #Inicia el nodo
        rospy.init_node('patrol')

        #Crea un cliente simple action, y luego espera a que el servidor,
        # es decir, navegacion, este disponible para recibirlo
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Conectado.")

        while not rospy.is_shutdown():
            #Dentro del loop, hace for entre los puntos y los va enviando y esperando
            # a que se ejecuten
            for pose in waypoints:
                goal = goal_pose(pose)
                rospy.loginfo("Enviando meta.")
                client.send_goal(goal)
                client.wait_for_result()
        rospy.loginfo("Se ha cerrado el nodo.")
    except rospy.ROSInterruptException:
        #rospy.loginfo("Programa interrumpido antes de completarse.")
        print("Programa interrumpido antes de completarse", file=sys.stderr)
        pass
'''
