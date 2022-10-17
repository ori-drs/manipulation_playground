cmake_minimum_required(VERSION 3.0.2)
project(manipulation_playground_components)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  affordances
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
