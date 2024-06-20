#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from manipulation_playground_device_dynamics_toggler.srv import toggleDeviceDynamics, toggleDeviceDynamicsResponse

class toggleDeviceDynamicsServer:
    def __init__(self):

        self.device_database = {}

        self.srv = rospy.Service(
            "toggleDeviceDynamics",
            toggleDeviceDynamics,
            self.handle_toggleDeviceDynamics,
        )

    def handle_toggleDeviceDynamics(self, req):
        device_type = req.device_type
        device_id = req.device_id
        enable = req.enable

        if device_type not in self.device_database:
            self.device_database[device_type] = {}

        if device_id not in self.device_database[device_type]:
            self.device_database[device_type][device_id] = None     # unknown initial joint state

        pub = rospy.Publisher('/' + device_type + '/' + device_id + '/' + device_type + '_controller/command', Float64, queue_size=10)

        msg = rospy.wait_for_message('/' + device_type + '/' + device_id + '/joint_states', JointState, timeout=3)
        current_joint_state = msg.position[0]

        # Set initial joint state if toggling for the first time (assumes device is currently at its initial joint state)
        if self.device_database[device_type][device_id] == None:
            self.device_database[device_type][device_id] = current_joint_state

        if enable:
            # Toggle on by changing controller command to initial joint state
            pub.publish(self.device_database[device_type][device_id])

        elif not enable:
            # Toggle off by changing controller command to current joint state
            pub.publish(current_joint_state)

        else:
            return toggleDeviceDynamicsResponse(False)

        return toggleDeviceDynamicsResponse(True)



if __name__ == "__main__":
    rospy.init_node("device_dynamics_toggler")

    s = toggleDeviceDynamicsServer()

    rospy.loginfo("Device dynamics toggler ready")

    rospy.spin()