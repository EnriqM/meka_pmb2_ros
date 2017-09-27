#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse
from smach_ros.service_state import ServiceState



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
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR. Solo recibe el contador de FOO y ya, devuelve el outcome1')
        #rospy.loginfo('Counter = %f'%userdata.bar_counter_in)  #Aqui es donde se ve el contador para rospy.loginfo
        '''
        rospy.wait_for_service('add_two_ints', timeout=3)
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        try:
            resp1 = add_two_ints(userdata.bar_counter_in, (userdata.bar_counter_in*2))
            rospy.loginfo('Respuesta = %f'%resp1.sum)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        '''
        sleep(1)
        
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4','preempted', 'aborted'])
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
        #smach.StateMachine.add('BAR', Bar(), 
        #                       transitions={'outcome1':'FOO'},
        #                       remapping={'bar_counter_in':'sm_counter'})
        @smach.cb_interface(input_keys=['bar_counter_in'])
        def two_ints_rquest_cb(userdata, request):
            peticion = AddTwoIntsRequest()
            peticion.a = userdata.sm_counter
            peticion.b = userdata.sm_counter*2
            
            print str(peticion.a + peticion.b)
            return peticion
        
        smach.StateMachine.add('BAR', remapping={'bar_counter_in':'sm_counter'}, ServiceState('add_two_ints', AddTwoInts, 
                               request_cb = two_ints_rquest_cb,
                               input_keys = ['sm_counter']),
                               transitions={'succeeded':'FOO'})


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
    