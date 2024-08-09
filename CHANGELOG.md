# Changelog for Pluto


## 2.6.2 (2024-08-09)
* Calculate current distance from waypoint in gps_plotter node using haversine
* Adjust first waypoint
* Increase waypoint radius
* Use gps instead of odom to calculate distance in phase_one_demo
* Fix: waypoint number not updating issue

## 2.6.1 (2024-08-03)
* Add waypoints to gps plotter
* Adjust gps plotter trail markers
* Refactor: customize_local_planner

## 2.6.0 (2024-08-03)
* Add: gps plotter node
* Add: new waypoints

## 2.5.7 (2024-08-01)
* Add: changelog
* Contributor: Matthew Lauriault

## 2.5.6 (2024-07-31)
* Refactor: add files to gitignore
* Remove: comment out message queue in splunk_logger
* Add: heading (yaw) log message
* Add: launch file to test sensors
* Add: heading (yaw) to waypoint ping
* Make yaml file for local_planner parameters
* Contributor: Matthew Lauriault

## 2.5.5 (2024-07-20)
* Add: compass yaw to waypoint ping
* Contributor: Matthew Lauriault

## 2.5.4 (2024-07-17)
* Add: waypoint ping for data collection during testing
* Contributor: Matthew Lauriault

## 2.5.3 (2024-07-17)
* Refactor: change 'source' field to 'id' in splunk logger
* Contributor: Matthew Lauriault

## 2.5.2 (2024-07-13)
* Modify splunk fields
* Refactor: document splunk messages
* Contributor: Matthew Lauriault

## 2.5.1 (2024-07-06)
* Remove: unused odometry-calculation node

## 2.5.0 (2024-07-04)
* Change to new joystick controller

## 2.4.5 (2024-06-29)
* Refactor: format local_planner code
* Refactor: add documentation and logs
* Fix: msg issue
* Contributor: Matthew Lauriault

## 2.4.4 (2024-06-26)
* Send line detection msg to splunk
* Contributor: Matthew Lauriault

## 2.4.3 (2024-06-22)
* Add: splunk_logger to pluto_launch_all
* Fix: update gps point
* Add: line detection msg
* Fix: resolve error in splunk_logger for joystick
* Contributors: Joel Du Shouyu, Matthew Lauriault

## 2.4.2 (2024-06-21)
* Refactor: improve code readability
* Refactor: separate AI Image Classification implementation
* Refactor: improve documentation
* Test: line detection -> successful
* Record line detection test
* Add: camera node to realsense launch file
* Contributor: Matthew Lauriault

## 2.4.1 (2024-06-20)
* Improve line detection accuracy
* Remove: comment out AI Image Classifier from logic
* Contributor: Matthew Lauriault

## 2.4.0 (2024-06-19)
* Add: line detetection
* Add: image division into regions based on detected line
* Add: region classification as cut/uncut using AI image classifier
* Add: test images for line detection
* Run line detection tests on the images
* Contributor: Matthew Lauriault

## 2.3.0 (2024-06-13)
* Add: camera logic node for realsense camera
* ADd: AI image classifier for camera node
* Add cut/uncut grass image data
* Contributor: Matthew Lauriault

## 2.2.2 (2024-06-08)
* Update: gps point
* Add: launch file for electric mower
* Ignore bag files
* Contributor: Joel Du Shouyu

## 2.2.1 (2024-06-05)
* Update: gps calculation
* Fix: gps calculation error
* Fix: debug message for phase1 demo
* Contributor: Joel Du Shouyu

## 2.2.0 (2024-06-05)
* Add: new splunk_logger node
* Add: launch file for splunk_logger node
* Add: imu and joystick data to splunk_logger node
* Add: log and odometry data to splunk_logger node
* Refactor: organize event attributes
* Print status code from HTTP request instead of logging it
* Change data format on splunk_logger node
* Contributor: Matthew Lauriault

## 2.1.1 (2024-06-01)
* Fix: remove realsense
* Fix: logic for tuning pid testing
* Fix: tuning logic
* Fix: logging for phase1 demo
* Add: camera launch in pluto_launch
* Contributor: Joel Du Shouyu

## 2.1.0 (2024-05-27)
* Add: phase1 demo
* Contributor: Joel Du Shouyu

## 2.0.6 (2024-05-19)
* Refactor/Fix: heading_error logic
* Contributor: Joel Du Shouyu

## 2.0.5 (2024-05-18)
* Add: pid tuning setup for turning
* Tune turning
* Contributor: Joel Du Shouyu

## 2.0.4 (2024-05-17)
* Refactor: customize_local_planner
* Contributor: Joel Du Shouyu

## 2.0.3 (2024-05-16)
* Add controller launch to pluto_launch_all file
* Contributor: Joel Du Shouyu

## 2.0.2 (2024-05-11)
* Fix: error in local_planner
* Fix: remove self.error reference in customize_local_planner
* Contributor: Joel Du Shouyu

## 2.0.1 (2024-05-08)
* Change transform of gps to base_link
* Contributor: Joel Du Shouyu

## 2.0.0 (2024-05-07)
* Change to customize_local_planner logic
* Contributor: Joel Du Shouyu

## 1.15.1 (2024-05-07)
* Refactor: remove unused node
* Add: fishbot_navigation to launch file
* Add: new pid tuning tool
* Add: test case for customize_local_planner
* Contributor: Joel Du Shouyu

## 1.15.0 (2024-04-30)
* Add: configuration for rviz2
* Add: launch file for running the robot
* Contributor: Joel Du Shouyu

## 1.14.11 (2024-01-26)
* Fix: error in splunk_logger
* Contributor: Joel Du Shouyu

## 1.14.10 (2024-01-11)
* Change baud rate to 9600 for arduino-controller
* Contributor: Joel Du Shouyu

## 1.14.9 (2024-01-04)
* Change baud rate for arduino-controller
* Flush serial in arduino-controller
* Add: "\n" in sending message
* Contributor: Joel Du Shouyu

## 1.14.8 (2023-12-22)
* Fix: turning logic
* Add: debug message
* Fix: syntax error
* Fix: change call
* Fix: bug in local_planner
* Contributor: Joel Du Shouyu

## 1.14.7 (2023-12-21)
* Fix: turn the compensate PID pwm value to be positive
* Decrease straight tolderance angle
* Decrease NAV2 and controller frequency
* Add: more debug info for printout
* Contributor: Joel Du Shouyu

## 1.14.6 (2023-12-20)
* Remove: controller from pluto_launch_all file
* Increase turning speed
* Contributor: Joel Du Shouyu

## 1.14.5 (2023-12-19)
* Add: debug mode for object-recognization
* Disable: odometry_calculation_launch in pluto_launch_all file
* Enable: controller_launch in pluto_launch_all file
* Round pwm value to integer
* Correct compensation logic
* Remove turning filter
* Correct logic for turning slow
* Contributor: Joel Du Shouyu

## 1.14.4 (2023-12-12)
* Try to minimize value for turning
* Allow more heading error and slower turning
* Add: debug log
* Change to debug
* Correct parameter
* Fix: error in creating simplePID
* Fix: turning
* Contributor: Joel Du Shouyu

## 1.14.3 (2023-12-11)
* Add: documentation for the filter parameter in turning pid
* Contributor: Joel Du Shouyu

## 1.14.2 (2023-11-30)
* Disable: controller
* Customize filter for tuning
* Increase threshold for moving straight pid
* Avoid tolerance distance check
* Contributor: Joel Du Shouyu

## 1.14.1 (2023-11-26)
* Make adjustment for electrical lawnmower
* Contributor: Joel Du Shouyu

## 1.14.0 (2023-11-25)
* Add: simulation tool
* Update: coverage area interface
* Update: vscode file for humble version
* Update: fishbot_navigation2 for changing ROS2 to humble version
* Update: autonomous node
* Update: coverageClient
* Modify coverage and customize_local_planner
* Contributor: Joel Du Shouyu

## 1.13.2 (2023-11-24)
* Fix: error in publish planner path
* Fix: error when goal pose is behind
* Refactor/Fix: bug in joystick-controller node
* Enable /is_autonomous_mode to publish
* Contributor: Joel Du Shouyu

## 1.13.1 (2023-11-22)
* Upgrade ROS to humble version
* Contributor: Joel Du Shouyu

## 1.13.0 (2023-11-21)
* Add: initial draw of path planning framework
* Add: buildScript bash script for docker development
* Add: vs-config for development in dockerfile
* Add: launch file for arduino-controller node
* Contributor: Joel Du Shouyu

## 1.12.0 (2023-11-09)
* Add arduino-controller node for electrical lawnmower
* Contributor: Joel Du Shouyu

## 1.11.0 (2023-11-02)
* Add splunk logger
* Modify compass printout message
* Add: pid tuning node
* Modify robot_localization frequency
* Modify NAV2 parameter
* Contributor: Joel Du Shouyu

## 1.10.4 (2023-08-02)
* Set autostart to False
* Modify nav2 config file
* Contributor: Joel Du Shouyu

## 1.10.3 (2023-07-28)
* Fix: update the nav2 configuration
* Fix: increase frequency for dual-ekf
* Contributor: Joel Du Shouyu

## 1.10.2 (2023-07-27)
* Test: modify robot to move 5 meters south
* Modify local costmap setup
* Configure local costmap
* Contributor: Joel Du Shouyu

## 1.10.1 (2023-07-15)
* Fix: change heading direction from east to south in virtual waypoint
* Modify local costmap
* Contributor: Joel Du Shouyu

## 1.10.0 (2023-07-14)
* Update: logic for calculating velocity base on gps
* Contributor: Joel Du Shouyu

## 1.9.3 (2023-07-10)
* Fix: launch error
* Use gps velocity
* Fix: config for dual-ekf
* Fix: correct filter logic for gps_vel
* Contributor: Joel Du Shouyu

## 1.9.2 (2023-07-06)
* Fix: use geopy to calculate distance between 2 gps points 
* Contributor: Joel Du Shouyu

## 1.9.1 (2023-06-19)
* Add: gps_velocity to launch file
* Contributor: Joel Du Shouyu

## 1.9.0 (2023-06-17)
* Add: gps_velocity node
* Change ekf to use gps_velocity node
* Fix: frame for gps_velocity node
* Contributor: Joel Du Shouyu

## 1.8.1 (2023-06-16)
* Disable: unconfigure message from OdometryCalculation
* Fix: bug for multi-point navigate
* Contributor: Joel Du Shouyu

## 1.8.0 (2023-06-13)
* Update: logic for gps waypoint follower
* Add: gps_filter launch to pluto_launch_all.launch
* Update: add ekf_filter launch to pluto_launch_all.launch
* Contributor: Joel Du Shouyu

## 1.7.1 (2023-06-13)
* Refactor: modify uncertainty in imu message during launch
* Modify map size and nav2 parameters
* Revert back to initial estimate in ServoAndSpeed
* Add: more uncertainty in yaw result output
* Change compass to ENU mode
* Modify gps configuration for new gps
* Change topic to result from gps_filter
* Form temporary static tree
* Remove: x-acceleration from imu
* Contributor: Joel Du Shouyu

## 1.7.0 (2023-06-08)
* Configure fishbot
* Add gps_filter node
* Contributor: Joel Du Shouyu

## 1.6.4 (2023-05-22)
* Update: image_yolo_filter algorithm to run faster
* Remove: unused comment in image_yolo_filter
* Contributor: Joel Du Shouyu

## 1.6.3 (2023-05-20)
* Add faster implementation of image_yolo_filter
* Use cache
* Contributor: Joel Du Shouyu

## 1.6.2 (2023-05-18)
* Add: message log to show time delay in collecting data for image_yolo_filter
* Fix: syntax error in image_yolo_filter
* Contributor: Joel Du Shouyu

## 1.6.1 (2023-05-16)
* Update: .gitmodules
* Add: python version of pointcloud filter
* Fix: bug in determining the width and height of image
* Use new YOLO model from different git repository
* Remove: compass_heading node
* Contributor: Joel Du Shouyu

## 1.6.0 (2023-05-14)
* Use YOLO for object detection
* Add: simulate_gps_node
* Contributor: Joel Du Shouyu

## 1.5.1 (2023-05-09)
* Tune robot_localization
* Contributor: Joel Du Shouyu

## 1.5.0 (2023-01-27)
* Add YOLOX-ROS
* Contributor: Joel Du Shouyu

## 1.4.1 (2023-01-27)
* Add: static transform for 'camera'
* Fix: remove robot_localization
* Contributor: Joel Du Shouyu

## 1.4.0 (2023-01-20)
* Introduce localization package
* Add: static transform for 'gps' and 'imu_link'
* Fix: modify launch order
* Fix: add static_transform_launch.py
* Refactor: add explanation in differential_raw_twist_callback()
* Remove: robot_localization dependency
* Contributor: Joel Du Shouyu

## 1.3.1 (2023-01-20)
* Refactor: rename topic /differential_raw_odometry to odometry/wheel
* Fix: bug in differential_odometry launch file
* Refactor: update comment in xbox yaml file
* Fix: imu config use
* Add: separate launch file for imu_filter package
* Contributor: Joel Du Shouyu

## 1.3.0 (2023-01-12)
* First draft of raw odometry calculation
* Refactor: controller node and joystick interpreter node
* Use sim_time
* Contributor: Joel Du Shouyu

## 1.2.0 (2023-01-12)
* Configure publish speed for Controller and Odometry
* Update: max linear and angular speed from the joystick
* Contributor: Joel Du Shouyu

## 1.1.0 (2022-12-28)
* Add: use_sim_time=true for all nodes
* Add: links in comments on 'use_sim_time' for all nodes
* Contributor: Joel Du Shouyu

## 1.0.1 (2022-12-28)
* Remove: py_pubsub example
* Add: comment in config file
* Contributor: Joel Du Shouyu

## 1.0.0 (2022-12-17)
* Change velocity interpretation
* Refactor: rename parameter in calculate_pwm_from_velocity2() and calculate_velocity_from_pwm2()
* Contributor: Joel Du Shouyu

## 0.2.3 (2022-12-14)
* Update: launch files
* Lower speed limit in joystick-mode-2
* Lower RIGHT_MAX and LEFT_MAX
* Disable: imu_filter publish transform
* Contributor: Joel Du Shouyu

## 0.2.2 (2022-12-01)
* Upgrade ROS to galactic version
* Update: launch file for imu_filter
* Contributor: Joel Du Shouyu

## 0.2.1 (2022-11-12)
* Change imu to ENU
* Fix: bug in controller
* Fix: maestro_controller by converting pwm to microseconds
* Fix: bug in joystick-mode-1
* Contributor: Joel Du Shouyu

## 0.2.0 (2022-11-10)
* Update: launch file
* Update: controller node
* Add: compass interpreter node
* Fix: bug in imu launch file
* Change mode of imu_filter
* Contributor: Joel Du Shouyu

## 0.1.0 (2022-11-03)
* Configure .gitignore folder
* Add: gps node
* Add: launch file for gps node
* Add: imu_tool
* Add: imu node
* Add: .gitmodules
* Add: camera node
* Add: compass node
* Add: imu converter
* Add: controller node
* Add: maestro_controller
* Add: launch files
* Contributor: Joel Du Shouyu