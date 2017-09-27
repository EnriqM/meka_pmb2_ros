#!/usr/bin/env python

'''
Created on 15 jul. 2017

@author: Enrique Ortega
'''

import rospy
import smach
import smach_ros
from time import sleep
#from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


from std_msgs.msg import Int32

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
        while not self.value == 40:
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
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR. Solo recibe el contador de FOO y ya, devuelve el outcome1')
        #rospy.loginfo('Counter = %f'%userdata.bar_counter_in)  #Aqui es donde se ve el contador para rospy.loginfo
        
        self.clase.value = 0
        self.clase.run()
        
        sleep(1)
        
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    # Inicializamos el contador a 0 en la propia maquina de estados.
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        
        #Tambien remapea de la state machine a cada una de las states.
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


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
