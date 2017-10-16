#!/usr/bin/env python
import sys
import rospy
from states_manager.msg import multiModal
from myo_driver.msg import emgState
from sensor_msgs.msg import Imu
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState


def callback_emg(data):
    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.emgState = data
    
def callback_imu(data):
    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.imuState = data
    
def callback_endpoint_state(data):
    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.endpointState = data
    
def callback_joint_state(data):
    global multiModal_states
    if any("head_pan" in s for s in data.name):
        multiModal_states.header.stamp = rospy.Time.now()
        multiModal_states.jointState = data


if __name__ == '__main__':

    # the publish rate of multiModal_states
    publish_rate = 50    
    
    global multiModal_states
    multiModal_states = multiModal()
    
    # ros node init
    rospy.init_node('states_pub_node', anonymous=True)
    
    # subscribe the states topics
    rospy.Subscriber("/myo_raw_emg_pub", emgState, callback_emg)
    rospy.Subscriber("/myo_raw_imu_pub", Imu, callback_imu)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, callback_endpoint_state)
    rospy.Subscriber("/robot/joint_states", JointState, callback_joint_state)    
    
    # publish topic
    pub = rospy.Publisher("/multiModal_states",multiModal, queue_size=20)
    
    r = rospy.Rate(publish_rate)
    
    while not rospy.is_shutdown():
        pub.publish(multiModal_states)
        print multiModal_states
        r.sleep()

