cmake_minimum_required(VERSION 3.8)
project(satellite_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(OpenCV             REQUIRED)
find_package(CURL               REQUIRED)

# ── 타일 페처 공유 라이브러리 ─────────────────────────────────────────────────
add_library(tile_fetcher SHARED src/tile_fetcher.cpp)
target_include_directories(tile_fetcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
target_link_libraries(tile_fetcher ${OpenCV_LIBRARIES} ${CURL_LIBRARIES})

# ── satellite_node ────────────────────────────────────────────────────────────
add_executable(satellite_node
  src/satellite_node.cpp
  src/satellite_node_main.cpp
)
ament_target_dependencies(satellite_node
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs cv_bridge)
target_include_directories(satellite_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(satellite_node tile_fetcher ${OpenCV_LIBRARIES})

# ── 설치 ─────────────────────────────────────────────────────────────────────
install(TARGETS tile_fetcher satellite_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch meshes rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
