cmake_minimum_required(VERSION 2.8.3)
project(calib_extrinsic)
add_definitions(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")	
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mytest
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include 
include/common
include/contrib
include/libdmtx
${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(usr/lib/x86_64-linux-gnu)
link_directories(lib)

add_executable(sender1 src/send.cpp)
target_link_libraries(sender1 pthread ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PTHREAD_LIB})

add_executable(calib_extrinsic src/calib_extrinsic.cpp src/calibratebasecamera.cpp src/basemovement.cpp src/apriltag_opencv.cpp)
target_link_libraries(calib_extrinsic DataMatrixDetector ImageProjector apriltag pthread ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PTHREAD_LIB})

add_executable(linkRobot src/linkrobot.cpp src/basemovement.cpp)
target_link_libraries(linkRobot pthread ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PTHREAD_LIB})

install(TARGETS  calib_extrinsic
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/lib 
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/install/libstatic)

install(TARGETS  calib_extrinsic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES calibExtrinsic.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)