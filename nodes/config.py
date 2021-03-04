
class Config:
    ## socket params:
    port = 12347
    ip = 'localhost'
    bufferSize = 256
    queue = 5
    
    ## laser & odom:
    cmd_vel_topic = 'cmd_vel'  # 'cmd_vel_mux/input/teleop'
    laser_topic = 'scan'
    odom_topic = 'pose' # 'odom'
    laserscan_range = (160, 480)
    
    ## mover params:
    angle_error = 0.1
    move_distance = 1
    obstacle_min_distance = 0.6
    move_forward_at_each_move = True
    regulated_speeds = {0.03: 0,
                        0.1: 0.01,
                        1: 0.03,
                        3: 0.1,
                        20: 0.3,
                        45: 0.5,
                        360: 0.5}  # mapping angles with angular speeds
    speed_reduce_factor = 0.1  # reducing linear speed
    slow_down_distance = 0.2  # reduce speed to minimum (0.1) under this distance from target (in meters)
