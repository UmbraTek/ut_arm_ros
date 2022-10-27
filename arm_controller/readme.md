# Communication with utra through the rosservice command
All servers are implemented in utra_server.cpp of arm_controller package.

All msg are in utra_msg package.


## Status
refer to [SetInt16.srv](../utra_msg/srv/SetInt16.srv),[GetInt16.srv](../utra_msg/srv/GetInt16.srv)

```
rosservice call /utra/status_set 0      #status 0:nomal 3:pause 4:stop
rosservice call /utra/status_get        #status 0:nomal 1:moveing 2:sleep 3:pause 4:stop
```

## Get Error
get utra error and servos msg with this service
refer to [GetUInt16A.srv](../utra_msg/srv/GetUInt16A.srv)

```
rosservice call /utra/get_error_code 
rosservice call /utra/get_servo_msg  
```

## Mode 
refer to [SetInt16.srv](../utra_msg/srv/SetInt16.srv),[GetInt16.srv](../utra_msg/srv/GetInt16.srv)

```
rosservice call /utra/mode_set 0 
rosservice call /utra/mode_get  
```

## Enable 
refer to [EnableSet.srv](../utra_msg/srv/EnableSet.srv),[GetInt16.srv](../utra_msg/srv/GetInt16.srv)

```
rosservice call /utra/enable_set 10 1 # 10>6 axis, Joint axis, if it is greater than the maximum number of joints, set all joints
rosservice call /utra/enable_get  
```

## Gripper Velocity and Acceleration
refer to [SetFloat32.srv](../utra_msg/srv/SetFloat32.srv),[GetFloat32.srv](../utra_msg/srv/GetFloat32.srv)

```
rosservice call /utra/gripper_vel_set  100   # value is [0-300]
rosservice call /utra/gripper_vel_get  

rosservice call /utra/gripper_acc_set  1# value is [0-300]
rosservice call /utra/gripper_acc_get 
```

## Gripper other
refer to [GetInt16.srv](../utra_msg/srv/GetInt16.srv),[SetInt16.srv](../utra_msg/srv/SetInt16.srv)

```
rosservice call /utra/gripper_error_code 
rosservice call /utra/gripper_reset_err 

rosservice call /utra/gripper_unlock 1  # this method is used when gripper is locked.
```

## Move Joint
Service name: utra/moveto_joint_p2p  
request:   
-------joints (list): target joint positions [rad]   
-------speed (float): joint speed of leading axis [rad/s]   
-------acc (float): joint acceleration of leading axis [rad/sˆ2]   

refer to [MovetoJointP2p.srv](../utra_msg/srv/MovetoJointP2p.srv)

```
rosservice call /utra/moveto_joint_p2p [1.248,1.416,1.155,-0.252,-1.248,-0.003] 0.1 3.0
```

## Move Line
Service name: utra/moveto_cartesian_line  
Move to position (linear in tool-space)   
request:   
-------pose (list): cartesian position [mm mm mm rad rad rad]  
-------speed (float): tool speed [mm/s]  
-------acc (float): tool acceleration [mm/sˆ2]  

refer to [MovetoCartesianLine.srv](../utra_msg/srv/MovetoCartesianLine.srv)

```
rosservice call /utra/moveto_joint_p2p [0,0,0,0,0,0] 0.1 3.0   # maybe you need to move to home pose first

rosservice call /utra/moveto_cartesian_line [-0.0,-360.0,800.0,1.58,0.0,0.0] 50 100   
```

## Move Line with Blend 
Blend circular (in tool-space) and move linear (in tool-space) to position.Accelerates to and moves with constant tool speed v.  
request:   
-------pose (list): cartesian position [mm mm mm rad rad rad]  
-------speed (float): tool speed [mm/s]  
-------acc (float): tool acceleration [mm/sˆ2]  
-------radii (float): blend radius [mm]  

refer to [MovetoCartesianLineB.srv](../utra_msg/srv/MovetoCartesianLineB.srv)

```
rosservice call /utra/moveto_joint_p2p [0,0,0,0,0,0] 0.1 3.0   # maybe you need to move to home pose first

rosservice call /utra/plan_sleep 15    #wait 15s for you to call the follow commands 

rosservice call /utra/moveto_cartesian_lineb [-0.0,-360.0,800.0,1.58,0.0,0.0] 50 100 50 
rosservice call /utra/moveto_cartesian_lineb [-8.0,-560.0,600.0,1.58,0.0,0.0] 50 100 50 
```