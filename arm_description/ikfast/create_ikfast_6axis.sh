#!/bin/bash

#set -eu

# ./create_ikfast.sh ../urdf utra6_550_robot
robot_dir=${1}
robot_name=${2}
rosrun xacro xacro --inorder -o ${robot_name}.urdf ${robot_dir}/${robot_name}.urdf.xacro
rosrun collada_urdf urdf_to_collada ${robot_name}.urdf ${robot_name}.dae
export IKFAST_PRECISION="5"
rosrun moveit_kinematics round_collada_numbers.py ${robot_name}.dae ${robot_name}.dae "$IKFAST_PRECISION"
openrave-robot.py ${robot_name}.dae --info links
openrave ${robot_name}.dae
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=${robot_name}.dae --iktype=transform6d --baselink=1 --eelink=7 --savefile=./ik_fast_${robot_name}.cpp
