cmake_minimum_required(VERSION 3.0.2)
project(image_nav)

# C++17 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 컴파일러 플래그 설정
add_compile_options(-std=c++11)
# find_package(GeographicLib REQUIRED)

set(GeographicLib_DIR /usr/local/lib/cmake/GeographicLib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  nav_msgs
  message_generation
  GeographicLib
  roslib
)

add_message_files(
  FILES
  PoseArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge nav_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/image_processor.cpp
  src/path_planner.cpp
)

add_executable(image_processor_node src/image_processor_node.cpp)
target_link_libraries(image_processor_node
 ${PROJECT_NAME} 
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
${GeographicLib_LIBRARIES}

 )

add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node
 ${PROJECT_NAME} 
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
 ${GeographicLib_LIBRARIES}

 )

add_executable(image_loader_node src/image_loader_node.cpp)
target_link_libraries(image_loader_node
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
${GeographicLib_LIBRARIES}

 )


 add_executable(csv_out_node src/csv_out_node.cpp)
target_link_libraries(csv_out_node
 ${catkin_LIBRARIES} 
 ${OpenCV_LIBRARIES}
${GeographicLib_LIBRARIES}

 )
