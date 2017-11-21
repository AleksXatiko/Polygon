cmake_minimum_required(VERSION 2.8.3)
project(poly_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  mavros
  mavros_msgs
  libmavconn
  eigen_conversions
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(directmav_ros
#   src/${PROJECT_NAME}/directmav_ros.cpp
# )

# add_dependencies(directmav_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_message_files(
    FILES
    obstacle.msg
    obstacles.msg
	robotModel_parametrs.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES directmav_ros
#  CATKIN_DEPENDS std_msg
  CATKIN_DEPENDS message_runtime
  DEPENDS EIGEN3
)
## Declare a C++ executable
add_executable(result src/result.cpp)
add_executable(main src/main.cpp)
add_executable(robotModel src/robotModel.cpp)
##add_executable(rpnodesub src/rpnodesub.cpp)

add_library(computationMovement src/computationMovement.cpp)
add_library(communication src/communication.cpp)
add_library(power src/power.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(directmav_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(result
  ${catkin_LIBRARIES}
)

##target_link_libraries(rpnodesub
##  ${catkin_LIBRARIES}
##)

target_link_libraries(main
  ${catkin_LIBRARIES} computationMovement communication power
)

target_link_libraries(robotModel
  ${catkin_LIBRARIES} communication
)

target_link_libraries(power
 ${catkin_LIBRARIES}
)

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS directmav_ros directmav_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
