# share
共享文件
 * /hexapod/control_frequency: 200
 * /hexapod/controller_type: pc
 * /hexapod/elspider3_ethercat_ifname: enp86s0
 * /hexapod/elspider3_ethercat_loop_frequency: 1000
 * /hexapod/elspider3_loop_frequency: 500
 * /hexapod/elspider_air_loop_frequency: 500
 * /hexapod/elspider_mini_loop_frequency: 400
 * /hexapod/enable_debug: False
 * /hexapod/enable_rosconsole: True
 * /hexapod/foot_lift_height: 0.05
 * /hexapod/foot_trajectory_type: cycloid
 * /hexapod/gait_type: quick_tripod1
 * /hexapod/gazebo_loop_frequency: 500
 * /hexapod/heel_motor_init_torque: -2.0
 * /hexapod/heel_motor_kd: 2
 * /hexapod/heel_motor_kp: 0.075
 * /hexapod/hip_motor_init_torque: 2.0
 * /hexapod/hip_motor_kd: 2
 * /hexapod/hip_motor_kp: 0.2
 * /hexapod/interface_type: hardware
 * /hexapod/joy_coe_w: 0.4
 * /hexapod/joy_coe_x: 0.25
 * /hexapod/joy_coe_y: 0.2
 * /hexapod/knee_motor_init_torque: -2.0
 * /hexapod/knee_motor_kd: 2
 * /hexapod/knee_motor_kp: 0.2
 * /hexapod/motor_init_frequency: 1
 * /hexapod/mujoco_loop_frequency: 1000
 * /hexapod/print_task_status: True
 * /hexapod/robot_name: elspider_air
 * /hexapod/sitdown_footstep_location_x: 0
 * /hexapod/sitdown_footstep_location_y: 0.2
 * /hexapod/sitdown_footstep_location_z: -0.11
 * /hexapod/standup_footstep_location_x: 0
 * /hexapod/standup_footstep_location_y: 0.28
 * /hexapod/standup_footstep_location_z: -0.16
 * /hexapod/state_publish_frequency: 500
 * /hexapod/unitree_a1_loop_frequency: 500
 * /hexapod/unitree_a1_power_level: 1
 * /hexapod/use_state_estimator: True
 * /hexapod/use_vrpn: False
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /vrpn_client_node/broadcast_tf: True
 * /vrpn_client_node/frame_id: robot
 * /vrpn_client_node/port: 3883
 * /vrpn_client_node/refresh_tracker_frequency: 1.0
 * /vrpn_client_node/server: 192.168.1.110
 * /vrpn_client_node/update_frequency: 500.0
 * /vrpn_client_node/use_server_time: False

NODES
  /
    joystick_sim (interface/joystick_sim)
    main_ctrl (user/main_ctrl)
    vrpn_client_node (vrpn_client_ros/vrpn_client_node)

ROS_MASTER_URI=http://localhost:11311

process[main_ctrl-1]: started with pid [18364]
process[vrpn_client_node-2]: started with pid [18365]
process[joystick_sim-3]: started with pid [18366]
[ INFO] [1747455075.735538434]: Connecting to VRPN server at 192.168.1.110:3883
[ INFO] [1747455075.931194126]: [hexapod software version]: 00.01.00.00
[ INFO] [1747455075.932611696]: [robot_name]: elspider_air
[ INFO] [1747455075.933396291]: [interface_type]: hardware_interface
[ INFO] [1747455075.946354032]: [controller_type]: position_controller
[ INFO] [1747455075.949029427]: [foot_trajectory_type]: cycloid
[PeriodicTask] Start InterfaceThread (0 s, 2000000 ns)
[PeriodicTask] Start ControllerThread1 (0 s, 4999999 ns)
[ INFO] [1747455075.963736917]: wait hardware to be ready
[ INFO] [1747455076.740283171]: Connection established
