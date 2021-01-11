import rospy
from threading import Timer, Lock
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Twist
from tf import Transformer, transformations
import copy
import numpy as np
from collections import deque
import sys
import math

class TargetFinder():

    def __init__(self, distance, rate=10):
        """ Given a leader path, pose and self pose, follows the leader at a specific distance

            :distance: Following distance to leader
            :rate: (Optional) Publish rate
        """
        self.distance = distance
        self.thread_lock = Lock()

        self.path = deque(maxlen=10000)      # x, y, theta, distance    Note: Distance 0 is the position of leader when the node is started
        self.pose = [0,0,0]                 # x, y, theta

    def updateLeaderPose(self, pose):
        """
            Update Leader Pose
        """
        self.thread_lock.acquire()
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        distance = None
        if len(self.path) == 0:
            distance = 0
        else:
            x_last, y_last, theta, distance_last = self.path[-1]
            distance = distance_last + np.sqrt( np.power(x - x_last, 2) + np.power(y - y_last, 2) )
        self.path.append([x, y, yaw, distance])
        self.thread_lock.release()

    def updatePose(self, pose):
        """
            Update follower pose
        """
        q = pose.pose.pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.thread_lock.acquire()
        self.pose = [pose.pose.pose.position.x, pose.pose.pose.position.y, yaw]
        self.thread_lock.release()

    def readLeaderPath(self):
        self.thread_lock.acquire()
        path = copy.deepcopy(self.path)
        self.thread_lock.release()
        return path
    
    def readPose(self):
        self.thread_lock.acquire()
        pose = copy.deepcopy(self.pose)
        self.thread_lock.release()
        return pose

    def extract_poses(self):
        """
            Returns current pose of follower and target pose

            Returns:    current_pose, target_pose
                        (x, y, theta), (x, y, theta)

            Throws: ValueError exception 

        """
        if len(self.path) == 0:
            raise ValueError("Path Empty")
            return
    
        path = np.array(self.readLeaderPath())
        # Path distance at target position
        target_dist = path[-1,3] - self.distance
        # Filter by distance
        far_poses = path[path[:,3] < target_dist]

        if len(far_poses) != 0:
            target_pose = far_poses[-1,:3]
        else:
            # No target
            raise ValueError("No target")
            return
        current_pose = np.array(self.readPose())

        return current_pose, target_pose
        

    
        
