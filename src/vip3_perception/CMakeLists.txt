cmake_minimum_required(VERSION 3.0.2)
project(vip3_perception)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs tf2_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs tf2_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_projection.py)
  catkin_add_nosetests(test/test_frames.py)
  catkin_add_nosetests(test/test_pose.py)
  catkin_add_nosetests(test/test_calibration.py)
  catkin_add_nosetests(test/test_coordinates.py)
endif()

catkin_install_python(PROGRAMS scripts/svm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
