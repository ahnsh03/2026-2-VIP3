cmake_minimum_required(VERSION 3.0.2)
project(vip3_hd_map)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs morai_msgs rospy sensor_msgs std_msgs visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frames.py)
  catkin_add_nosetests(test/test_live_pose.py)
  catkin_add_nosetests(test/test_local_crop.py)
  catkin_add_nosetests(test/test_map_overlay.py)
  catkin_add_nosetests(test/test_mgeo_layers.py)
endif()

catkin_install_python(PROGRAMS
  scripts/katri_map_viz_node.py
  scripts/inspect_katri_mgeo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
