# Pluto

Ros2 Developement for autonomous lawnmower Phase 2


## Splunk Messages

| id             | type        | ROS topic        |
| -------------- | ----------- | ---------------- |
| startup        | message     | -                |
| log            | message     | rosout           |
| gps            | sensor      | /fix/filtered    |
| joystick       | sensor      | /joy             |
| odometry       | calculation | /odometry/global |
| line_detection | calculation | /line            |
| waypoint_ping  | ping        | /waypoint_ping   |
| startup_ping   | ping        | -