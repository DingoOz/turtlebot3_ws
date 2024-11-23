cmake_minimum_required(VERSION 3.10)
project(qt5_webcam_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenCV REQUIRED)

# Include Qt5 directories
include_directories(${Qt5Widgets_INCLUDE_DIRS})

# Add Qt5 definitions
add_definitions(${Qt5Widgets_DEFINITIONS})

# Set Qt5 flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

# Create the executable
add_executable(webcam_viewer src/webcam_viewer.cpp)
target_include_directories(webcam_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(webcam_viewer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  webcam_viewer
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)

# Link Qt5 and OpenCV
target_link_libraries(webcam_viewer
  ${Qt5Widgets_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS webcam_viewer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
