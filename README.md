# Pluto

Ros2 Developement for autonomous lawnmower Phase 2


## Splunk Messages

| id             | type        | ROS topic          |
| -------------- | ----------- | ------------------ |
| startup        | message     | N/A                |
| log            | message     | `/rosout`          |
| gps            | sensor      | `/fix/filtered`    |
| joystick       | sensor      | `/joy`             |
| odometry       | calculation | `/odometry/global` |
| line           | calculation | `/line`            |
| local_planner  | calculation | `/analysis/all`    |