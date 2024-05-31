cmake_minimum_required(VERSION 3.8)
project(xsens_mtw_driver_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(imu_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(include)
link_directories(lib)


# -------------------- xsens_mtw_manager --------------------
add_executable(xsens_mtw_manager
  src/conio.c
  src/findClosestUpdateRate.cpp
  src/mastercallback.cpp
  src/mtwcallback.cpp
  src/vqf.cpp
  src/xsens_mtw_manager_node.cpp
  src/xsens_mtw_manager.cpp
)

# Link libraries
target_link_libraries(xsens_mtw_manager
  xsensdeviceapi
  xstypes
  pthread
  dl
)

# Add ament dependencies
ament_target_dependencies(xsens_mtw_manager
  rclcpp
  std_msgs
  geometry_msgs
  imu_msgs
)


# ----------------- xsens_mtw_visualization -----------------
add_executable(xsens_mtw_visualization
src/xsens_mtw_visualization_node.cpp
src/xsens_mtw_visualization.cpp
)

ament_target_dependencies(xsens_mtw_visualization
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  imu_msgs
)


# -------------------------- Install --------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY lib/
  DESTINATION lib
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS xsens_mtw_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS xsens_mtw_visualization
  DESTINATION lib/${PROJECT_NAME}
)


# -------------------------- Tests --------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
