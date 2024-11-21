cmake_minimum_required(VERSION 3.8)
project(turtlebot3_webcam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)

# Add these lines for libudev
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

# Find CURL
find_package(CURL REQUIRED)

# Make sure all packages are found
if(NOT LIBUDEV_FOUND)
    message(FATAL_ERROR "libudev not found")
endif()

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found")
endif()

if(NOT CURL_FOUND)
    message(FATAL_ERROR "CURL not found")
endif()

# camera_publisher executable
add_executable(camera_publisher src/camera_publisher.cpp)
target_include_directories(camera_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBUDEV_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
target_compile_features(camera_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  std_msgs
)
target_link_libraries(camera_publisher 
  ${LIBUDEV_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CURL_LIBRARIES}
)

# camera_viewer executable
add_executable(camera_viewer src/camera_viewer.cpp)
target_include_directories(camera_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)
target_compile_features(camera_viewer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  camera_viewer
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  std_msgs
  rmw_fastrtps_cpp
)
target_link_libraries(camera_viewer
  ${OpenCV_LIBRARIES}
  ${CURL_LIBRARIES}
)

# Install targets
install(TARGETS
  camera_publisher
  camera_viewer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

