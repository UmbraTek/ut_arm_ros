

# 1. Introduction
This repository contains the 3D models of utra series and demo packages for ROS development and simulations.Developing and testing environment: Ubuntu 18.04 + ROS melodic + Gazebo 9.  


# 2. Preparations before using this package

## 2.1 Install dependent package module
gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> (if use Gazebo)   
ros_control: <http://wiki.ros.org/ros_control> (remember to select your correct ROS distribution)  
moveit_core: <https://moveit.ros.org/install/>  
   
## 2.2 Go through the official tutorial documents
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit Tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 2.2 Install "mimic_joint_plugin"
if you use Gripper in Gazebo, you need to install the [mimic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) for make the mimic joints behave normally. if you use [new version](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), please change "libroboticsgroup_gazebo_mimic_joint_plugin.so" to "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so" in file: utra_ros/utra_gripper/urdf/gripper.transmission.xacro

# 3. Getting started with 'utra_ros'
   
## 3.1 Create a catkin workspace. 
If you already have a workspace, skip and move on to next part.
Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 3.2 Obtain the package
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/UmbraTek/utra_ros
   ```
## 3.3 Install other dependent packages:
   ```bash
   rosdep update
   rosdep check --from-paths . --ignore-src --rosdistro melodic
   ```
   Please change 'melodic' to the ROS distribution you use. If there are any missing dependencies listed. Run the following command to install:  
   ```bash
   rosdep install --from-paths . --ignore-src --rosdistro melodic -y
   ```
   And change 'melodic' to the ROS distribution you use.  

## 3.4 Build the code
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
## 3.5 Source the setup script
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Skip above operation if you already have that inside your ~/.bashrc. Then do:
```bash
source ~/.bashrc
```
# 4. Run in RViz
Launch utra 550 rviz :
```bash
roslaunch utra_description utra6_550_view.launch [gripper:=true] [vacuum_gripper:=true]
```
Launch utra 850 rviz :
```bash
roslaunch utra_description utra6_850_view.launch [gripper:=true] [vacuum_gripper:=true]
```
Launch utra 1000 rviz :
```bash
roslaunch utra_description utra6_1000_view.launch [gripper:=true] [vacuum_gripper:=true]
```

# 5. Run in RViz  and Gazebo simulator
You can launch Rviz and gazebo, and controll the arm in Rviz. When in first launch time, the arm is in vertical posture, it hard to plan trajectory, you can select the **init** posture in **MotionPlanning->Planning->Goal state** and click the **Plan & Execute** button to make arm go to the good posture.

Launch utra 850 rviz and gazebo:
1. Run gazebo first:
```bash
roslaunch utra6_850_gazebo gazebo.launch
```
2. Then in another terminal:
```bash
roslaunch utra6_850_moveit_config moveit_planning_execution.launch
```
If launch with gripper,
1. Run gazebo with gripper first:
```bash
roslaunch utra6_850_gazebo gazebo.launch gripper:=true
```
2. Then in another terminal:
```bash
roslaunch utra6_850_moveit_gripper_config moveit_planning_execution.launch
```

# 6. Run RViz and connect with utra
You can connect with utra and controll it in rviz.
**Pay Attention** you need to very be careful the **Trajectory planning** in rviz , you must **Play** before the **Excute** every time to make sure that the generated trajectory is not **Collision**. We suggest that make utra with a good posture by select the **init** posture in **MotionPlanning->Planning->Goal state**.

Launch the Rviz and connect the utra 550
```bash
roslaunch utra6_550_moveit_config run_with_utra.launch utra_ip:="utra_ip_address"
```
Launch the Rviz and connect the utra 850
```bash
roslaunch utra6_850_moveit_config run_with_utra.launch utra_ip:="utra_ip_address"
```

Launch the Rviz and connect the utra 1000
```bash
roslaunch utra6_1000_moveit_config run_with_utra.launch utra_ip:="utra_ip_address"
```

# 7. Excute the command to communicate with server 

## 7.1 Launch the server to connect the utra

```bash
roslaunch utra_controller utra_server.launch utra_ip:="utra_ip_address"   //utra_ip_address like 192.168.1.234
```
## 7.2 Communication with utra through the rosservice command

Server's communication interface is below:

```
string api_name
string[] args
---
string[] rets

```


There are the server call examle below:

```
rosservice call /utra_server "connect" "['192.168.1.234']" //arg: [IP]. first time you roslaunch the server,it will connect automatic, so you don't need to connec again.
rosservice call /utra_server "disconnect" "[]"  

rosservice call /utra_server "get_motion_enable" "[]"
rosservice call /utra_server "set_motion_enable" "['1']"     //set motion enable arg is '1' or '0'

rosservice call /utra_server "get_motion_status" "[]"
rosservice call /utra_server "set_motion_status" "['0']"  //0  :normal 1  : move  2  :sleep  3  :pause 4  : stop

rosservice call /utra_server "move_joints" "['1.2','1.2','1.2','0.2','0.2','0.2','0.4','60']"  //set utra move to joints [j1,j2,j3,j4,j5,j6,velicity,acceleration] joint's unit is rad, velicity is rad/s, acceleration is rad/s²

rosservice call /utra_server "move_line" "[x,y,z,roll,pitch,yaw,velicity,acceleration]" x,y,z unit is mm, roll, pitch, yaw unit is rad, velecity unit is mm/s, acceleration unit is mm/s²
```
