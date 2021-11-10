#!/usr/bin/python
"""
This node contains 2 Subscribers and 2 publishers.
The 2 subscribers subscribe to the human odom and gun pose to get the data
The other 2 publishers publishes to uav_target_odom and uav_target_pose
"""

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from tf import Transformer, transformations
import tf
import math
import numpy as np
import signal

human_odom_sub = None
gun_pose_sub = None
uav_target_pose_pub = None
uav_target_odom_pub = None
sequence_number = 0
human_pose = None
gun_pose = None

def human_odom_cb(odom):
    global human_pose
    human_pose = odom.pose.pose

def gun_pose_cb(pose):
    if(human_pose is None):
        rospy.logerr("Gun Pose Publisher: Human Odometry Not Received, but Gun Pose received")
    else:
        # Fetch Latest Human Pose
        current_human_pose = human_pose
        # Generate Human TF
        pos, qt = current_human_pose.position, current_human_pose.orientation
        human_tf = transformations.quaternion_matrix([qt.x, qt.y, qt.z, qt.w])
        human_tf[:3, 3] = pos.x, pos.y, pos.z

        # Generate Gun TF (Gun Local Frame)
        pos, qt = pose.pose.pose.position, pose.pose.pose.orientation
        gun_tf = transformations.quaternion_matrix([qt.x, qt.y, qt.z, qt.w])
        gun_tf[:3, 3] = pos.x, pos.y, pos.z

        # Calculate Gun TF in Odometry frame
        # Rotating gun pose by 90 degree yaw to match with human coordinate system
        #yaw_90 = transformations.euler_matrix(0.0, 0.0, -math.pi/2, axes='sxyz')
        #gun_tf_odom_frame = np.dot(yaw_90, gun_tf)
        # Removed rotation since Sam is publishing orientation in actual human frame
        gun_tf_odom_frame = gun_tf
        # Adding human position translation to gun pose 
        gun_tf_odom_frame[:3, 3] += human_tf[:3, 3] # Adding human translation to gun pose
      
        # Transfer back to odom
        global gun_pose
        gun_pose = Pose()
        gun_pose.position.x, gun_pose.position.y, gun_pose.position.z = gun_tf_odom_frame[:3,3]
        qt = transformations.quaternion_from_matrix(gun_tf_odom_frame)
        gun_pose.orientation.x = qt[0]
        gun_pose.orientation.y = qt[1]
        gun_pose.orientation.z = qt[2]
        gun_pose.orientation.w = qt[3]

def publish_uav_target_pose():
    if (gun_pose is None):
        rospy.logwarn_throttle(10.0, rospy.get_name() + " : Gun pose not yet received")
    else:
        target_pose = gun_pose
        targetOdom = Odometry()

        # Populating Odometry msg
        global sequence_number
        targetOdom.header.frame_id = "odom"
        targetOdom.header.stamp = rospy.get_rostime()
        targetOdom.header.seq = sequence_number
        sequence_number += 1
        targetOdom.pose.pose = target_pose

        uav_target_odom_pub.publish(targetOdom)

        targetPose = PoseWithCovarianceStamped()
        targetPose.header = targetOdom.header
        targetPose.pose.pose = target_pose

        uav_target_pose_pub.publish(targetPose)
        
    
def target_publisher():
    
    rospy.init_node('gun_target_publisher')
    signal.signal(signal.SIGINT, terminate)

    human_odom_topic = rospy.get_param(rospy.get_name()+'/human_odom_topic')            #, '/HumanPose'
    gun_pose_topic = rospy.get_param(rospy.get_name()+'/gun_pose_topic')                #, '/GunPose'
    uav_target_odom_topic = rospy.get_param(rospy.get_name()+'/target_odom_topic')      #, '/UAV1TargetOdom'
    uav_target_pose_topic = rospy.get_param(rospy.get_name()+'/target_pose_topic')      #, '/UAV1TargetPose'

    publish_rate = rospy.get_param(rospy.get_name()+'/publish_rate')		            #, 10

    global human_odom_sub, uav_target_pose_pub, uav_target_odom_pub, gun_pose_sub

    human_odom_sub = rospy.Subscriber(human_odom_topic, Odometry, human_odom_cb)
    gun_pose_sub = rospy.Subscriber(gun_pose_topic, PoseWithCovarianceStamped, gun_pose_cb)
    uav_target_pose_pub = rospy.Publisher(uav_target_pose_topic, PoseWithCovarianceStamped, queue_size=1000)
    uav_target_odom_pub = rospy.Publisher(uav_target_odom_topic, Odometry, queue_size=1000)
    
    print ("UAV Gun Pose Publisher: %s" %(rospy.get_name()))

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        publish_uav_target_pose()
        rate.sleep()

def terminate(*args):
    print "Gun Target Publisher: User termination requested"
    human_odom_sub.unregister()
    gun_pose_sub.unregister()
    sys.exit()


if __name__ == '__main__':
    target_publisher()


    

