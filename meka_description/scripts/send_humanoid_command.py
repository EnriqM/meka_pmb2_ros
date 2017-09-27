import time
import os
import roslib; roslib.load_manifest('meka_description')
import rospy
from m3ctrl_msgs.msg import M3JointCmd
from std_msgs.msg import Header

rospy.init_node("humanoid_command_publisher")
pub = rospy.Publisher("/humanoid_command", M3JointCmd)

chain=[]
chain_idx=[]
stiffness=[]
velocity=[]
position = []
effort= []
control_mode=[]
smoothing_mode=[]

chain.append(0)
chain_idx.append(0)
stiffness.append(0.5)
velocity.append(0.1)
position.append(2.0)
effort.append(100)
control_mode.append(3)
smoothing_mode.append(3)

header = Header(0,rospy.Time.now(),'0')
pub.publish(M3JointCmd(header, chain, chain_idx, stiffness, velocity, position, effort, control_mode, smoothing_mode))

try:
    while not rospy.is_shutdown():
	time.sleep(1.0)	
	header = Header(0,rospy.Time.now(),'0')
	pub.publish(JointState(header, joints, positions, [0]*len(positions), [0]*len(positions)))	    
except (KeyboardInterrupt,EOFError,rospy.ROSInterruptException):
    pass




