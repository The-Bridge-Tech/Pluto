# Pluto

Ros2 Developement for autonomous lawnmower Phase 2


## Splunk Messages

| id             | type        | ROS topic        |
| -------------- | ----------- | ---------------- |
| startup        | message     | -                |
| log            | message     | rosout           |
| gps            | sensor      | /fix/filtered    |
| imu            | sensor      | /imu/data        |
| joystick       | sensor      | /joy             |
| odometry       | calculation | /odometry/global |
| line_detection | calculation | /line            |
| waypoint_ping  | calculation | /waypoint_ping   |