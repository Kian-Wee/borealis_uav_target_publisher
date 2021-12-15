#!/usr/bin/python

import os
# Constrain OPENBLAS Multithreading (To solve Numpy Performance Issues)
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from threading import Timer
from target_finder import TargetFinder
from tf import Transformer, transformations
import signal

human_odom_sub = None
uav_target_pose_pub = None
uav_target_odom_pub = None
target_finder = None
timer = None
sequence_number = 0

class ClassTimer():
        def __init__(self, callback, interval):
            self.callback = callback
            self.interval = interval
            self.timer = Timer(self.interval, self.cb)
            self.timer.start()

        def cb(self):
            self.callback()
            if not(rospy.is_shutdown()):
                self.timer = Timer(self.interval, self.cb)
                self.timer.start()
    
        def stop(self, *args):
            self.timer.cancel()

def human_odom_cb(odom):
    pose = odom.pose.pose
    target_finder.updateLeaderPose(pose)

def uav_pose_cb(pose):
    target_finder.updatePose(pose)

def publish_uav_target_pose():
    try:
        current_pose, target_pose = target_finder.extract_poses()
        
    except ValueError as ex:
        rospy.logerr_throttle(5.0, rospy.get_name() + " : " + str(ex))

        # Publish hovering target if no follow_me target is not available
        target_pose = 0.0, 0.0, 1.3     # x, y, z

    targetOdom = Odometry()
    # Populating Odometry msg
    global sequence_number
    targetOdom.header.frame_id = "odom"
    targetOdom.header.stamp = rospy.get_rostime()
    targetOdom.header.seq = sequence_number
    sequence_number += 1

    x, y, theta = target_pose
    q = transformations.quaternion_from_euler(0.0, 0.0, theta, axes='sxyz')
    targetOdom.pose.pose.position.x = x
    targetOdom.pose.pose.position.y = y
    targetOdom.pose.pose.position.z = 1.3
    targetOdom.pose.pose.orientation.x = q[0]
    targetOdom.pose.pose.orientation.y = q[1]
    targetOdom.pose.pose.orientation.z = q[2]
    targetOdom.pose.pose.orientation.w = q[3]

    uav_target_odom_pub.publish(targetOdom)

    targetPose = PoseWithCovarianceStamped()
    targetPose.header = targetOdom.header
    targetPose.pose.pose = targetOdom.pose.pose
    uav_target_pose_pub.publish(targetPose)
    
def target_publisher():
    
    rospy.init_node('follow_me_target_publisher')
    signal.signal(signal.SIGINT, terminate)

    human_odom_topic = rospy.get_param(rospy.get_name()+'/human_odom_topic')            #, '/HumanPose'
    uav_target_odom_topic = rospy.get_param(rospy.get_name()+'/target_odom_topic')      #, '/UAV1TargetOdom'
    uav_target_pose_topic = rospy.get_param(rospy.get_name()+'/target_pose_topic')      #, '/UAV1TargetPose'

    follow_distance = rospy.get_param(rospy.get_name()+'/follow_distance')              #, 2
    publish_rate = rospy.get_param(rospy.get_name()+'/publish_rate')		            #, 10

    global human_odom_sub, uav_target_pose_pub, uav_target_odom_pub, target_finder, timer

    human_odom_sub = rospy.Subscriber(human_odom_topic, Odometry, human_odom_cb)
    uav_target_pose_pub = rospy.Publisher(uav_target_pose_topic, PoseWithCovarianceStamped, queue_size=1000)
    uav_target_odom_pub = rospy.Publisher(uav_target_odom_topic, Odometry, queue_size=1000)
    target_finder = TargetFinder(follow_distance, publish_rate)
    
    interval = 1.0 / publish_rate       # Seconds
    timer = ClassTimer(publish_uav_target_pose, interval)
   
    print ("UAV Following Pose Publisher: %s" %(rospy.get_name()))
    rospy.spin()

def terminate(*args):
    print "Follow Me Target Publisher: User termination requested"
    timer.stop()
    human_odom_sub.unregister()
    sys.exit()


if __name__ == '__main__':
    target_publisher()


    

