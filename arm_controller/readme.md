# Communication with arm through the rosservice command
All servers are implemented in utarm_api_server.cpp of arm_controller package.

All msg are in ut_msg package.


## Status
refer to [SetInt16.srv](../ut_msg/srv/SetInt16.srv),[GetInt16.srv](../ut_msg/srv/GetInt16.srv)

```
rosservice call /utsrv/status_set 0      #status 0:nomal 3:pause 4:stop
rosservice call /utsrv/status_get        #status 0:nomal 1:moveing 2:sleep 3:pause 4:stop
```

## Get Error
get arm error and servos msg with this service
refer to [GetUInt16A.srv](../ut_msg/srv/GetUInt16A.srv)

```
rosservice call /utsrv/get_error_code 
rosservice call /utsrv/get_servo_msg  
```

## Mode 
refer to [SetInt16.srv](../ut_msg/srv/SetInt16.srv),[GetInt16.srv](../ut_msg/srv/GetInt16.srv)

```
rosservice call /utsrv/mode_set 0 
rosservice call /utsrv/mode_get  
```

## Enable 
refer to [SetEnable.srv](../ut_msg/srv/SetEnable.srv),[GetInt16.srv](../ut_msg/srv/GetInt16.srv)

```
rosservice call /utsrv/enable_set 10 1 # 10>6 axis, Joint axis, if it is greater than the maximum number of joints, set all joints
rosservice call /utsrv/enable_get  
```

## Gripper Velocity and Acceleration
refer to [SetFloat32.srv](../ut_msg/srv/SetFloat32.srv),[GetFloat32.srv](../ut_msg/srv/GetFloat32.srv)

```
rosservice call /utsrv/gripper_vel_set  100   # value is [0-300]
rosservice call /utsrv/gripper_vel_get  

rosservice call /utsrv/gripper_acc_set  1# value is [0-300]
rosservice call /utsrv/gripper_acc_get 
```

## Gripper other
refer to [GetInt16.srv](../ut_msg/srv/GetInt16.srv),[SetInt16.srv](../ut_msg/srv/SetInt16.srv)

```
rosservice call /utsrv/gripper_error_code 
rosservice call /utsrv/gripper_reset_err 

rosservice call /utsrv/gripper_unlock 1  # this method is used when gripper is locked.
```

## Move Joint
Service name: utsrv/moveto_joint_p2p  
request:   
-------joints (list): target joint positions [rad]   
-------speed (float): joint speed of leading axis [rad/s]   
-------acc (float): joint acceleration of leading axis [rad/sˆ2]   

refer to [MovetoJointP2p.srv](../ut_msg/srv/MovetoJointP2p.srv)

```
rosservice call /utsrv/moveto_joint_p2p [1.248,1.416,1.155,-0.252,-1.248,-0.003] 0.1 3.0
```

## Move Line
Service name: utsrv/moveto_cartesian_line  
Move to position (linear in tool-space)   
request:   
-------pose (list): cartesian position [mm mm mm rad rad rad]  
-------speed (float): tool speed [mm/s]  
-------acc (float): tool acceleration [mm/sˆ2]  

refer to [MovetoCartesianLine.srv](../ut_msg/srv/MovetoCartesianLine.srv)

```
rosservice call /utsrv/moveto_joint_p2p [0,0,0,0,0,0] 0.1 3.0   # maybe you need to move to home pose first

rosservice call /utsrv/moveto_cartesian_line [-0.0,-360.0,800.0,1.58,0.0,0.0] 50 100   
```

## Move Line with Blend 
Blend circular (in tool-space) and move linear (in tool-space) to position.Accelerates to and moves with constant tool speed v.  
request:   
-------pose (list): cartesian position [mm mm mm rad rad rad]  
-------speed (float): tool speed [mm/s]  
-------acc (float): tool acceleration [mm/sˆ2]  
-------radii (float): blend radius [mm]  

refer to [MovetoCartesianLineB.srv](../ut_msg/srv/MovetoCartesianLineB.srv)

```
rosservice call /utsrv/moveto_joint_p2p [0,0,0,0,0,0] 0.1 3.0   # maybe you need to move to home pose first

rosservice call /utsrv/plan_sleep 15    #wait 15s for you to call the follow commands 

rosservice call /utsrv/moveto_cartesian_lineb [-0.0,-360.0,800.0,1.58,0.0,0.0] 50 100 50 
rosservice call /utsrv/moveto_cartesian_lineb [-8.0,-560.0,600.0,1.58,0.0,0.0] 50 100 50 
```