cmake_minimum_required(VERSION 3.0.2)
project(manipulation_playground)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  sensor_msgs
  message_generation
)

# add_service_files(
#   FILES
# )

# add_action_files(
#    FILES
# )

# generate_messages(
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
# )

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
