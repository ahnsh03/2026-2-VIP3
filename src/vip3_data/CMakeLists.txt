cmake_minimum_required(VERSION 3.0.2)
project(vip3_data)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs std_srvs sensor_msgs geometry_msgs diagnostic_msgs morai_msgs rosbag rosgraph)
catkin_python_setup()
catkin_package()
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_bag_support.py)
  catkin_add_nosetests(test/test_profiles.py)
endif()
catkin_install_python(PROGRAMS
  scripts/record_node.py scripts/replay_node.py scripts/snapshot_node.py
  scripts/audit_bag.py scripts/reject_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
