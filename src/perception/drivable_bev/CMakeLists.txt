cmake_minimum_required(VERSION 3.0.2)
project(drivable_bev)

find_package(catkin REQUIRED COMPONENTS
  camera_semantic_perception
  diagnostic_msgs
  geometry_msgs
  message_filters
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    camera_semantic_perception
    diagnostic_msgs
    geometry_msgs
    message_filters
    morai_msgs
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_grid.py)
  catkin_add_nosetests(test/test_calibration.py)
  catkin_add_nosetests(test/test_projector.py)
  catkin_add_nosetests(test/test_fusion.py)
  catkin_add_nosetests(test/test_rear_camera.py)
  catkin_add_nosetests(test/test_performance_monitor.py)
  catkin_add_nosetests(test/test_visualization.py)
  catkin_add_nosetests(test/test_model_grid_projection.py)
  catkin_add_nosetests(test/test_lane_evidence.py)
  catkin_add_nosetests(test/test_lane_tracking.py)
  catkin_add_nosetests(test/test_spline_fitting.py)
  catkin_add_nosetests(test/test_rgb_bev.py)
endif()

catkin_install_python(PROGRAMS
  scripts/camera_bev_node.py
  scripts/camera_bev_fusion_node.py
  scripts/camera_bev_visualizer_node.py
  scripts/rgb_bev_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
