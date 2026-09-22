cmake_minimum_required(VERSION 3.0.2)
project(camera_semantic_perception)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  diagnostic_msgs
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_inference_core.py)
  catkin_add_nosetests(test/test_visualization.py)
endif()

catkin_install_python(PROGRAMS
  scripts/semantic_inference_node.py
  scripts/semantic_visualizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
