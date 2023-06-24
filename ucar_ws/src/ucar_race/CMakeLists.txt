cmake_minimum_required(VERSION 3.0.2)
project(ucar_race)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

# set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(TENSORRT_LIB_PATH /usr/lib/aarch64-linux-gnu/)
set(CUDNN_LIB_PATH /usr/lib/aarch64-linux-gnu/)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  xf_mic_asr_offline
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  # roslib
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ucar_race
  # CATKIN_DEPENDS message_runtime
  DEPENDS roslib
)


include_directories(
  include 
  src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)
link_directories(/usr/lib/aarch64-linux-gnu/tegra)
# # ucar_race
add_executable(ucar_race src/ucar_race.cpp)
target_link_libraries(ucar_race ${catkin_LIBRARIES})#


# # approach_server
add_executable(ar_code_server src/ar_code_server.cpp)
target_link_libraries(ar_code_server jetson-inference ${OpenCV_LIBS}  ${CUDA_LIBRARIES} ${catkin_LIBRARIES}  nvinfer nvinfer_plugin nvparsers -lopencv_dnn)# ${Boost_LIBRARIES}

# player_test
add_executable(player_test src/player_test.cpp)
target_link_libraries(player_test ${catkin_LIBRARIES})#
