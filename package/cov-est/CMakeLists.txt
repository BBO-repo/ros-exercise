cmake_minimum_required(VERSION 2.8.3)
project(cov_est)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  ## geometry_msgs
  nav_msgs  
  ## tf
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ## Add dependencies of other libraries
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs ## geometry_msgs tf 
   DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Definition of libraries
add_library(CovEst
  src/CovEstNode.cpp
)
target_link_libraries(CovEst
  ${catkin_LIBRARIES}
)
add_dependencies(CovEst
  ${catkin_EXPORTED_TARGETS}
)

## Definition of executables
add_executable(cov_est_exe
  src/CovEstNode_main.cpp
)
target_link_libraries(cov_est_exe
  CovEst
  ${catkin_LIBRARIES}
)

install(TARGETS cov_est_exe

  CovEst

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

