cmake_minimum_required(VERSION 2.8.3)
project(baxter_cppkdl)

include_directories(
  include/${PROJECT_NAME}             
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Eigen3 REQUIRED)
find_package(catkin
  REQUIRED
  COMPONENTS
  roscpp
  kdl_parser
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  DEPENDS
  orocos_kdl
  kdl_parser
  LIBRARIES baxter_cppkdl
  INCLUDE_DIRS include
)

add_library(baxter_cppkdl src/baxter_cppkdl.cpp)
target_link_libraries(baxter_cppkdl ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


