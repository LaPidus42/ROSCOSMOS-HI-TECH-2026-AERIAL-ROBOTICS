cmake_minimum_required(VERSION 3.0.2)
project(fire_detection_ros_rknn_slop2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/fire_detector_rknn_node.py
  scripts/thermal_camera_node.py
  scripts/mavlink_telemetry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
