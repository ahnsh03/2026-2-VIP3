cmake_minimum_required(VERSION 3.0.2)
project(vip3_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/morai_topic_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
