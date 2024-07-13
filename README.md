# Pluto

Ros2 Developement for autonomous lawnmower Phase 2


## Splunk Messages

| source         | type        | ROS topic        |
| -------------- | ----------- | ---------------- |
| log            | message     | rosout           |
| gps            | sensor      | /fix/filtered    |
| imu            | sensor      | /imu/data        |
| joystick       | sensor      | /joy             |
| odometry       | calculation | /odometry/global |
| line_detection | calculation | /line            |