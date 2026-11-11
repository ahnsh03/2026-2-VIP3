cmake_minimum_required(VERSION 3.0.2)
project(vip3_parking)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  std_msgs
  vip3_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs morai_msgs nav_msgs rospy std_msgs vip3_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_ego_state.py)
  catkin_add_nosetests(test/test_freshness.py)
  catkin_add_nosetests(test/test_evaluation.py)
  catkin_add_nosetests(test/test_frames.py)
  catkin_add_nosetests(test/test_gt.py)
endif()

catkin_install_python(PROGRAMS
  scripts/state_node.py
  scripts/gear_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
