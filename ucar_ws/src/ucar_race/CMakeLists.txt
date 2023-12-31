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
target_link_libraries(ucar_race ${catkin_LIBRARIES})
add_executable(wav_node src/wav.cpp)
target_link_libraries(wav_node ${catkin_LIBRARIES})

add_executable(move_node src/move.cpp)
target_link_libraries(move_node ${catkin_LIBRARIES})

add_executable(daohang_node src/daohang.cpp)
target_link_libraries(daohang_node ${catkin_LIBRARIES})

add_executable(music_node src/music.cpp)
target_link_libraries(music_node ${catkin_LIBRARIES})

add_executable(move1quick_node src/move1_quick.cpp)
target_link_libraries(move1quick_node ${catkin_LIBRARIES})

add_executable(move2quick_node src/move2_quick.cpp)
target_link_libraries(move2quick_node ${catkin_LIBRARIES})

add_executable(move3quick_node src/move3_quick.cpp)
target_link_libraries(move3quick_node ${catkin_LIBRARIES})


add_executable(move1slow_node src/move1_slow.cpp)
target_link_libraries(move1slow_node ${catkin_LIBRARIES})

add_executable(move2slow_node src/move2_slow.cpp)
target_link_libraries(move2slow_node ${catkin_LIBRARIES})

add_executable(move3slow_node src/move3_slow.cpp)
target_link_libraries(move3slow_node ${catkin_LIBRARIES})

# # approach_server
add_executable(ar_code_server src/ar_code_server.cpp)
target_link_libraries(ar_code_server jetson-inference ${OpenCV_LIBS}  ${CUDA_LIBRARIES} ${catkin_LIBRARIES}  nvinfer nvinfer_plugin nvparsers -lopencv_dnn)# ${Boost_LIBRARIES}

# player_test
add_executable(player_test src/player_test.cpp)
target_link_libraries(player_test ${catkin_LIBRARIES})#
