cmake_minimum_required(VERSION 3.8)
project(image_yolo_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_filter src/publish_filter_pointcloud.cpp)
ament_target_dependencies(image_filter rclcpp std_msgs sensor_msgs bboxes_ex_msgs message_filters pcl_conversions PCL OpenCV cv_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  image_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
