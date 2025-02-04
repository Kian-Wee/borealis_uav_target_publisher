#!/usr/bin/python

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import signal

gun_target_odom_sub = None
gun_target_pose_sub = None 
follow_me_target_odom_sub = None 
follow_me_target_pose_sub = None
human_target_odom_sub = None
human_target_pose_sub = None
target_odom_pub = None
target_pose_pub = None
command_sub = None
mux_state = "None"

def gun_target_odom_cb(odom):
    if (mux_state=="Go"):
        target_odom_pub.publish(odom)

def gun_target_pose_cb(pose):
    if (mux_state=="Go"):
        target_pose_pub.publish(pose)

def follow_me_target_odom_cb(odom):
    if (mux_state=="Follow"):
        target_odom_pub.publish(odom)

def follow_me_target_pose_cb(pose):
    if (mux_state=="Follow"):
        target_pose_pub.publish(pose)

def human_target_odom_cb(odom):
    if (mux_state=="Human"):
        target_odom_pub.publish(odom)

def human_target_pose_cb(pose):
    if (mux_state=="Human"):
        target_pose_pub.publish(pose)

def command_cb(command):
    global mux_state
    mux_state = command.data
    # if (mux_state == 'Circle'):
    if (mux_state == 'Go'):
        rospy.loginfo("Gun Target Enabled")
    elif (mux_state == 'Follow'):
        rospy.loginfo("Follow Me Target Enabled")
    elif (mux_state == 'Human'):
        rospy.loginfo("Human Target Enabled")
    else:
        rospy.logerr("Target Mux : Invalid Command - " + command.data)

def target_multiplexer():
    
    rospy.init_node('uav_target_mux')
    signal.signal(signal.SIGINT, terminate)

    gun_target_odom_topic = rospy.get_param(rospy.get_name()+'/gun_target_odom_topic') 
    gun_target_pose_topic = rospy.get_param(rospy.get_name()+'/gun_target_pose_topic')   
    follow_me_target_odom_topic = rospy.get_param(rospy.get_name()+'/follow_me_target_odom_topic') 
    follow_me_target_pose_topic = rospy.get_param(rospy.get_name()+'/follow_me_target_pose_topic')
    human_target_odom_topic = rospy.get_param(rospy.get_name()+'/human_target_odom_topic') 
    human_target_pose_topic = rospy.get_param(rospy.get_name()+'/human_target_pose_topic')
    target_odom_topic = rospy.get_param(rospy.get_name()+'/target_odom_topic') 
    target_pose_topic = rospy.get_param(rospy.get_name()+'/target_pose_topic')   
    command_topic = rospy.get_param(rospy.get_name()+'/command_topic') 

    global gun_target_odom_sub, gun_target_pose_sub, follow_me_target_odom_sub, follow_me_target_pose_sub, human_target_odom_sub, human_target_pose_sub, target_odom_pub, target_pose_pub, command_sub, mux_state

    gun_target_odom_sub = rospy.Subscriber(gun_target_odom_topic, Odometry, gun_target_odom_cb)
    gun_target_pose_sub = rospy.Subscriber(gun_target_pose_topic, PoseWithCovarianceStamped, gun_target_pose_cb)
    follow_me_target_odom_sub = rospy.Subscriber(follow_me_target_odom_topic, Odometry, follow_me_target_odom_cb)
    follow_me_target_pose_sub = rospy.Subscriber(follow_me_target_pose_topic, PoseWithCovarianceStamped, follow_me_target_pose_cb)
    human_target_odom_sub = rospy.Subscriber(human_target_odom_topic, Odometry, human_target_odom_cb)
    human_target_pose_sub = rospy.Subscriber(human_target_pose_topic, PoseWithCovarianceStamped, human_target_pose_cb)
    target_odom_pub = rospy.Publisher(target_odom_topic, Odometry, queue_size=1000)
    target_pose_pub = rospy.Publisher(target_pose_topic, PoseWithCovarianceStamped, queue_size=1000)
    command_sub = rospy.Subscriber(command_topic, String, command_cb)
    
    print ("UAV Target Mux: %s" %(rospy.get_name()))
    rospy.spin()

def terminate(*args):
    print "UAV Target Mux: User termination requested"
    gun_target_odom_sub.unregister()
    gun_target_pose_sub.unregister()
    follow_me_target_odom_sub.unregister()
    follow_me_target_pose_sub.unregister()
    human_target_odom_sub.unregister()
    human_target_pose_sub.unregister()
    command_sub.unregister()
    sys.exit()

if __name__ == '__main__':
    target_multiplexer()


    

