cmake_minimum_required(VERSION 3.0.2)
project(data_collection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  morai_msgs
  message_filters
  rosbag
  rosgraph
)

catkin_python_setup()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_capture_mode.py)
  catkin_add_nosetests(test/test_capture_sync.py)
  catkin_add_nosetests(test/test_mask_baker.py)
  catkin_add_nosetests(test/test_perception_curation.py)
  catkin_add_nosetests(test/test_perception_dataset.py)
  catkin_add_nosetests(test/test_bag_support.py)
endif()

catkin_install_python(PROGRAMS
  scripts/sync_collector_node.py
  scripts/capture_collector_node.py
  scripts/bag_recorder_node.py
  scripts/bag_replay_node.py
  scripts/audit_rosbag_run.py
  scripts/reject_bag_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
