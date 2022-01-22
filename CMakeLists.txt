cmake_minimum_required (VERSION 3.8)

SET(CMAKE_BUILD_TYPE release)

# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/lib/x86_64-linux-gnu/cmake")
# set(OpenCV_DIR "/usr/local/share/OpenCV")

project (LidarIris)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories("./LidarIris/")

add_executable(demo "demo.cpp" "LidarIris/LidarIris.cpp" "LidarIris/LidarIris.h" "LidarIris/fftm/fftm.cpp" "LidarIris/fftm/fftm.hpp")
target_link_libraries(demo ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})


# add_executable(front "front_view.cpp")
# target_link_libraries(front ${PCL_LIBRARIES} ${OpenCV_LIBS})


# add_executable(test_icp "test_icp.cpp" "LidarIris/LidarIris.cpp" "LidarIris/LidarIris.h" "LidarIris/fftm/fftm.cpp" "LidarIris/fftm/fftm.hpp")
# target_link_libraries(test_icp ${PCL_LIBRARIES} ${OpenCV_LIBS})

