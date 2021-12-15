# UAV Target Publisher Package for Borealis

This package will publish UAV target for Human Follow me mode, and Gun Targeting mode. <br />  <br /> 
The package in includes 3 nodes.

* follow_me_pose_publisher
  > Subscribes to Human pose and publishes a target point which is 'x' meters behind the human, along the path traversed by human.
* gun_pose_publisher
  > Subscribes to Gun Pose (defined by orientation in global frame & position in human frame) and publishes the target point in global odometry frame continuously.
* target_multiplexer
  > Subscribes to follow_me target and gun_target both; and a command topic which specifies which target to be relayed to UAV. 

## Dependencies
* ROS
* Numpy
  

#### Subscribed Topics
- nav_msgs/Odometry : /imu_odometry (Human Pose)
- nav_msgs/PoseWithCovarianceStamped : /firefly/command/pose  (Gun Pose)
- std_msgs/String : /command (Command Topic)
  
#### Published Topics
- nav_msgs/Odometry: /uav1/follow_me_target_odom
- nav_msgs/PoseWithCovarianceStamped: /uav1/follow_me_target_pose

The topics can be remapped or changed in the launch files.

**Compile and Launch the package**
>       roslaunch borealis_uav_target_publisher uav_target_publisher.launch

#### Parameters
* **publish_rate** : Target Publish rate (follow_me_target_publisher.launch, gun_target_publisher.launch)
* **follow_distance** : Follow Me distance from human (uav_target_publisher.launch)

#### Command Topic
Currently the multiplexer only accepts the following strings <br />
* "Gun" : To select Gun target following mode
* "FollowMe" : To select human follow me mode
* "Human" : TBC

The multiplexer can be **Dactivated** by inputting any other string. 

#### Note
Execution permissions might be needed for launching the package. Navigate to /scripts folder inside the package and execute
> sudo chmod +x ./*

