cmake_minimum_required(VERSION 3.0.2)
project(vip3_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# 커스텀 타입은 최소로 유지한다. 표준 메시지로 표현되는 것은 표준을 쓴다
# (/vehicle/odom = nav_msgs/Odometry, /vehicle/gear = std_msgs/Int32).
# 새 타입을 추가하기 전에 README.md 를 읽을 것.
add_message_files(
  FILES
  VehicleState.msg
)

add_service_files(
  FILES
  SetGear.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
