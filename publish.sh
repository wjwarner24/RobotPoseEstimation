#!/bin/bash
# This script continuously publishes a JointState message to /actuator_setpoint at 10 Hz

ros2 topic pub -r 10 /actuator_setpoint sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['front-l-drive', 'front-r-drive', 'hind-l-drive', 'hind-r-drive', 'front-steer', 'hind-steer'],
  position: [0, 0, 0, 0, 0.3927, 0],
  velocity: [10, 10, 10, 10, 0, 0],
  effort: [0, 0, 0, 0, 0, 0]
}"


# a = 10
# phi = pi / 8
# v = 0.5 m/s
# w = 0.518 rad/s
# R = 0.965 m