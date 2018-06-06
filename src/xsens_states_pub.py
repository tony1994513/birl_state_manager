#!/usr/bin/env python

import rospy
from states_manager.msg import multiModal
from myo_driver.msg import emgState
from sensor_msgs.msg import Imu
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import tf
from geometry_msgs.msg import TransformStamped
import ipdb
import numpy as np

def callback_emg(data):
    global flag_emg
    if not flag_emg:
        flag_emg = True
        print 'emg signals is OK!'

    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.emgStates = data

    
def callback_imu(data):
    global flag_imu
    if not flag_imu:
        flag_imu = True
        print 'imu signals is OK!'

    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.imuStates = data

    
def callback_endpoint_state(data):
    global flag_endpoint_state
    if not flag_endpoint_state:
        flag_endpoint_state = True
        print 'endpoint state signals is OK!'

    global multiModal_states
    multiModal_states.header.stamp = rospy.Time.now()
    multiModal_states.endpointStates = data

    
def callback_joint_states(data):
    global flag_joint_state
    if not flag_joint_state:
        flag_joint_state = True
        print 'joint states signals is OK!'

    global multiModal_states
    if any("head_pan" in s for s in data.name):
        multiModal_states.header.stamp = rospy.Time.now()
        multiModal_states.jointStates = data


def tf_compute(target, source):
    try:
        global listener
        (trans, rot) = listener.lookupTransform(target,source, rospy.Time(0))
        global flag_tf
        if flag_tf == False:
            flag_tf = True
            print 'tf of interest signals is OK!'
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    transforms_temp = TransformStamped()
    transforms_temp.transform.translation.x = trans[0]
    transforms_temp.transform.translation.y = trans[1]
    transforms_temp.transform.translation.z = trans[2]
    transforms_temp.transform.rotation.x = rot[0]
    transforms_temp.transform.rotation.y = rot[1]
    transforms_temp.transform.rotation.z = rot[2]
    transforms_temp.transform.rotation.w = rot[3]
    transforms_temp.header.frame_id = source
    transforms_temp.child_frame_id = target
    return transforms_temp

right_hand_list = []
def callback_tfoi(data):
    global tfoi,right_hand_list
    for idx, des in enumerate(tfoi):
        des_tf = tf_compute(des,"base")
        if des_tf!=None:
            global multiModal_states
            multiModal_states.header.stamp = rospy.Time.now()
            multiModal_states.tf_of_interest.transforms[idx] = des_tf


if __name__ == '__main__':

    # ros node init
    rospy.init_node('multi_modal_states_pub_node', anonymous=True)

    # the publish rate of multiModal_states
    publish_rate = 50

    # tf of interest
    tfoi = ["/left_hand_xsens_new","/right_hand_xsens_new","/left_hand","/right_hand"]

    # the multimodal states to include all info of interest
    multiModal_states = multiModal()
    multiModal_states.tf_of_interest.transforms = [TransformStamped() for i in range(len(tfoi))]

    # the listener for tf of interest
    listener = tf.TransformListener()

    # the flag
    flag_emg = False
    flag_imu = False
    flag_endpoint_state = False
    flag_joint_state = False
    flag_tf = False

    # subscribe the states topics
    rospy.Subscriber("/myo_raw_emg_pub", emgState, callback_emg)
    rospy.Subscriber("/myo_raw_imu_pub", Imu, callback_imu)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, callback_endpoint_state)
    rospy.Subscriber("/robot/joint_states", JointState, callback_joint_states)
    rospy.Subscriber("/tf", TFMessage, callback_tfoi)

    # publish topic
    pub = rospy.Publisher("/multiModal_states",multiModal, queue_size=100)
    
    r = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        pub.publish(multiModal_states)
        r.sleep()

