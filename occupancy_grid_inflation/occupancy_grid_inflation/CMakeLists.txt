cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_inflation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the inflation node executable
add_executable(inflation_node
  src/inflation_node.cpp
)


# Link dependencies
target_include_directories(inflation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_definitions(inflation_node
  PRIVATE "DEFAULT_RMW_IMPLEMENTATION=rmw_fastrtps_cpp")

ament_target_dependencies(inflation_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS inflation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_package()