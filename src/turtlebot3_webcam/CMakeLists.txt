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

# Add these lines for libudev
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

# camera_publisher executable
add_executable(camera_publisher src/camera_publisher.cpp)
target_include_directories(camera_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBUDEV_INCLUDE_DIRS}  # Add this line
)
target_compile_features(camera_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
# Add this line to link against libudev
target_link_libraries(camera_publisher ${LIBUDEV_LIBRARIES})

# camera_viewer executable
add_executable(camera_viewer src/camera_viewer.cpp)
target_include_directories(camera_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(camera_viewer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  camera_viewer
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install targets
install(TARGETS
  camera_publisher
  camera_viewer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
