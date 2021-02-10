# Moveit2-Ros1-Control-Franka
This is a repo for using Moveit2 and Ros1 Control to moving Franka Panda, due to lack of Ros2 Control of Franka.

## Pre-requests
 - Ubuntu 20.04
 - ROS Noetic
 - ROS Foxy and Moveit2

## Install
### action_bridge and ros1_bridge
These 2 packages are necessary for ROS2 <---> ROS1 inter-communication. 

Install ros1_bridge for bridging topics and services:
``````
sudo apt get install ros-foxy-ros1-bridge
``````
Install action_bridge for bridging action, before that, install some dependence first:
``````
sudo apt install ros-noetic-actionlib ros-noetic-actionlib-tutorials ros-noetic-control-msgs ros-noetic-roscpp ros-foxy-control-msgs ros-foxy-rclcpp ros-foxy-rclcpp-action ros-foxy-action-tutorials-interfaces
``````
Then, copy the folder 'action_bridge_ws' to your `Home` path, namely `~`. Finally, build it.
``````
cd action_bridge_ws/
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
colcon build
``````

### franka_ros1_control
Install franka ros1 control for connecting real robot, before that, install some dependence first:
``````
sudo apt get install ros-noetic-ros-control ros-noetic-ros-controllers
``````
Also, You have to follow the official [Franka Control Interface Documentation](https://frankaemika.github.io/docs/installation_linux.html#building-from-source), to build 'libfranka' from source and install linux 'real-time kernel'.

After you finished building libfranka, copy the folder 'franka_ros1_control_ws' to your `Home` path, namely `~`. Finally, build it.
``````
cd franka_ros1_control_ws/
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
``````

### franka_ros2_moveit2
Install franka ros2 moveit2 packages for motion planning, before that, make sure you have installed [moveit2](https://moveit.ros.org/install-moveit2/source/) already.

copy the folder 'franka_ros2_moveit2_ws' to your `Home` path, namely `~`. Finally, build it.
``````
cd franka_ros2_moveit2_ws/
source /opt/ros/foxy/setup.bash
source ~/ws_ros2/install/setup.bash
colcon build
``````

## Usage
1. ros1 master
````
source /opt/ros/noetic/setup.bash
roscore
````
2. action_bridge
````
source ~/action_bridge_ws/install/setup.bash
ros2 run action_bridge action_bridge_follow_joint_trajectory_2_1 position_joint_trajectory_controller
````
3. ros1_bridge
````
source ~/action_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge
````
4. franka_ros1_control  

Note: you have to unlock the brake of franka and make sure the TCP connection to robot is available. here the ip address of robot is set as 192.168.3.20 and make sure your computer's ip address is set as 192.168.3.X. (The X could be 2~19). Then, launch the franka ros control:
````
source ~/franka_ros1_control_ws/install/setup.bash
roslaunch franka_control franka_control.launch
````
5. franka_ros2_moveit2 

launch the franka ros2 move_group. Then, you can move franka to pre-defined poses, such as 'ready', 'extended', 'transport'. Also, you can use MoveGroupInterface to control the robot too.
````
source ~/franka_ros2_moveit2_ws/install/setup.bash
ros2 launch run_move_group run_move_group.launch.py
````
6. trigger gripper switch by ros2 service

you can trigger the gripper to grasp object by:
```` 
ros2 service call /franka_gripper/gripper_switch std_srvs/srv/SetBool "data: true"
````
you can trigger the gripper to drop object by:
```` 
ros2 service call /franka_gripper/gripper_switch std_srvs/srv/SetBool "data: false"
````

7. perception pipeline for dynamic collision avoidance  

After you launch ros2 move_group, there will be a ros2 topic called '/test_point_cloud'. This is a topic to receive the sensor_msgs::msg::PointCloud2 and transform PointCloud2 to Octomap into moveit_planning_scene for collision detection. Thus, you can use ros2 node to send PointCloud2 to this topic to achieve dynamic collision avoidance.  

If you want to clear Octomap generated, you can use ros2 service call like below to do so:
````
ros2 service call /clear_octomap std_srvs/srv/Empty
````

