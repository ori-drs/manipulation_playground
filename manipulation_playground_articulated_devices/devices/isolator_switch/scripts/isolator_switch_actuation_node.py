#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


bool_pub = rospy.Publisher('activation_state', Bool, queue_size=1)
command_pub = rospy.Publisher('isolator_switch_controller/command', Float64, queue_size = 10)

activation_state = Bool(False)
joint_target = Float64()

def callback(joint_states):
    if (joint_states.position[0] > 1.3963) and (not activation_state.data):
        activation_state.data = True
        joint_target.data = 1.5707
        command_pub.publish(joint_target)
    elif (joint_states.position[0] < 0.1745) and (activation_state.data):
        activation_state.data = False
        joint_target.data = 0.0
        command_pub.publish(joint_target)
     
    bool_pub.publish(activation_state)
    

if __name__ == '__main__':

    rospy.init_node('actuation_node')
	
    rospy.Subscriber('joint_states', JointState, callback)	# Listen for joint state
        
    rospy.spin()
