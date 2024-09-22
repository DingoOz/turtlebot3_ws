cmake_minimum_required(VERSION 3.8)
project(dashboard_turtlebot3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

add_executable(dashboard_node
  src/dashboard_node.cpp
  include/${PROJECT_NAME}/dashboard_node.hpp
)

target_include_directories(dashboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dashboard_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dashboard_node
  rclcpp
  geometry_msgs
)
target_link_libraries(dashboard_node Qt5::Widgets)

install(TARGETS dashboard_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

ament_package()
