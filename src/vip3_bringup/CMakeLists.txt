cmake_minimum_required(VERSION 3.0.2)
project(integration_launch)

find_package(catkin REQUIRED)

catkin_package()

## launch 파일을 devel/install 공간에도 노출시켜서 $(find integration_launch)가
## 소스 트리 밖에서도(설치된 워크스페이스에서도) 동작하게 함.
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
