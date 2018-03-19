cmake_minimum_required(VERSION 2.8.3)
project(calib_intrinsic)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  sensor_msgs
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intrinsic_matlb
#  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(include
  ${catkin_INCLUDE_DIRS} cd 

)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imgSender src/sendImg.cpp)
target_link_libraries(imgSender ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(show_camera src/show_camera.cpp)
target_link_libraries(show_camera ${OpenCV_LIBS} ${catkin_LIBRARIES})

include_directories(/usr/local/MATLAB/MATLAB_Runtime/v90/extern/include)
link_directories(/usr/local/MATLAB/MATLAB_Runtime/v90/runtime/glnxa64)
link_directories(/usr/local/MATLAB/MATLAB_Runtime/v90/bin/glnxa64)
# include_directories(/usr/local/MATLAB/R2015b/extern/include)
# link_directories(/usr/local/MATLAB/R2015b/runtime/glnxa64)
# link_directories(/usr/local/MATLAB/R2015b/bin/glnxa64)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(calib_matlab src/calib_using_matlab.cpp)

target_link_libraries(calib_matlab libmatlabCalib.so mwmclmcrrt mwmclmcr)

install(FILES lib/libmatlabCalib.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS  calib_matlab show_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES calibIntrinsic.sh showCamera.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
