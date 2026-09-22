cmake_minimum_required(VERSION 3.0.2)
project(vip3_hd_map)

find_package(catkin REQUIRED COMPONENTS

  geometry_msgs
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs morai_msgs nav_msgs rospy sensor_msgs std_msgs visualization_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frames.py)
  catkin_add_nosetests(test/test_correspondence.py)
  catkin_add_nosetests(test/test_schema.py)
  catkin_add_nosetests(test/test_map_overlay.py)
  catkin_add_nosetests(test/test_corridor.py)
  catkin_add_nosetests(test/test_nominal.py)
  catkin_add_nosetests(test/test_current_builder.py)
  catkin_add_nosetests(test/test_observed.py)
  catkin_add_nosetests(test/test_observed_builder.py)
  catkin_add_nosetests(test/test_live_pose.py)
  catkin_add_nosetests(test/test_lane_association.py)
  catkin_add_nosetests(test/test_local_crop.py)
  catkin_add_nosetests(test/test_review_bundle.py)
  catkin_add_nosetests(test/test_legacy_adoption.py)
  catkin_add_nosetests(test/test_hybrid_completion.py)
  catkin_add_nosetests(test/test_operational_map.py)
  catkin_add_nosetests(test/test_review_context.py)
  catkin_add_nosetests(test/test_route_curation.py)
  catkin_add_nosetests(test/test_competition_metadata.py)
  catkin_add_nosetests(test/test_runtime_bundle.py)
  catkin_add_nosetests(test/test_drivable_area_audit.py)
  catkin_add_nosetests(test/test_drivable_area_audit_workspace.py)
  catkin_add_nosetests(test/test_compact_drivable.py)
  catkin_add_nosetests(test/test_compact_drivable_workspace.py)
  catkin_add_nosetests(test/test_development_visualization.py)
endif()

catkin_install_python(PROGRAMS
  scripts/audit_legacy_alignment.py
  scripts/build_candidate_review.py
  scripts/build_legacy_adoption.py
  scripts/build_hybrid_completion.py
  scripts/build_operational_map.py
  scripts/build_route_curation.py
  scripts/build_competition_metadata.py
  scripts/build_drivable_area_input_audit.py
  scripts/build_compact_drivable_map.py
  scripts/competition_map_publisher_node.py
  scripts/local_map_crop_node.py
  scripts/hd_map_consumer_example.py
  scripts/verify_competition_map_topics.py
  scripts/verify_local_map_topics.py
  scripts/map_review_logger_node.py
  scripts/build_current_corridor.py
  scripts/calibrate_observed_lanes.py
  scripts/candidate_map_overlay_node.py
  scripts/legacy_origin_overlay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz schemas
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY reports/kcity2025_operational_map_20260916
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/reports
)
