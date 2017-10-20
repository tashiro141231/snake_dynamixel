cmake_minimum_required(VERSION 2.8.3)
project(snake_dynamixel)
find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS dynamixel_controllers roscpp rospy std_msgs
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
