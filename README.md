# Pionner_bci

ROS interface with bci system through sockets.

## Launch
1. In one terminal launch the robot's starting scripts from [pionner_bringup](https://github.com/amineHorseman/pioneer_bringup) package:

- If you have a laser use:

```
roslaunch pioneer_bringup laser_lms1xx.launch
```

- If you don't have a laser use:

```
roslaunch pioneer_bringup minimal.launch
```

For any issue in this step, check [pionner_bringup](https://github.com/amineHorseman/pioneer_bringup) readme file.

2. In another terminal launch the bci socket listener:

``` 
ros_launch pionner_bci turn.launch
```

## Commands
Send socket commands as integer from 0 to 9, the robot will turn to one of the following angles, then move forward:


- 0 => none
- 1 => turn to angle 45°
- 2 => turn to angle 0°
- 3 => turn to angle -45°
- 4 => turn to angle 90°
- 5 => none
- 6 => turn to angle -90°
- 7 => turn to angle 135°
- 8 => turn to angle 180°
- 9 => turn to angle -135°

## Configuration
For now the configuration is manual. You have to change the variables in the file `nodes/show_angle.py`, on lines 17-21:

- port: socket port
- ip: socket ip
- cmd_vel_topic: use '/cmd_vel' when using real robot, or 'cmd_vel_mux/input/teleop' when using a simulated robot.
- laser_topic: usually set to 'scan'
- odom_topic: usually set to 'odom'

## Test sockets

If you want to test the socket, launch the script `nodes/test_socket.py`.

## Experiments
Tested on ROS 1 Kinetic with P3DX. Should work on all other Pionner-like robots using ROSARIA or P2OS.
Tested also on Gazebo with turtlebot simulation.
