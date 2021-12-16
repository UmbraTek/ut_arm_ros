# Communication with utra through the rosservice command
All servers are implemented in utra_server.cpp of utra_controller package.

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
rosservice call /utra/enable_set 0 
rosservice call /utra/enable_get  
```

## Gripper velocity 
refer to [SetFloat32.srv](../utra_msg/srv/SetFloat32.srv),[GetFloat32.srv](../utra_msg/srv/GetFloat32.srv)

```
rosservice call /utra/gripper_vel_set  100   # value is [0-300]
rosservice call /utra/gripper_vel_get  
```