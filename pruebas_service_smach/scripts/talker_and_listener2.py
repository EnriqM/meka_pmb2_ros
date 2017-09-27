#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class Echo(object):
    def __init__(self):
        self.value = 0

        rospy.init_node('echoer')
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

if __name__ == '__main__':
    prueba = Echo()
    prueba.run()
    