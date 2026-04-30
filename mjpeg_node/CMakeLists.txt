cmake_minimum_required(VERSION 3.8)
project(mjpeg_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge   REQUIRED)
find_package(OpenCV      REQUIRED)
find_package(CURL        REQUIRED)

add_executable(mjpeg_node
  src/mjpeg_node.cpp
  src/mjpeg_node_main.cpp
)
ament_target_dependencies(mjpeg_node rclcpp sensor_msgs cv_bridge)
target_include_directories(mjpeg_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
target_link_libraries(mjpeg_node ${OpenCV_LIBRARIES} ${CURL_LIBRARIES})

install(TARGETS mjpeg_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
