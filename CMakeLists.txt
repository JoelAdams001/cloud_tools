cmake_minimum_required(VERSION 3.8)
project(cloud_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(voxelizer src/voxel_grid.cpp)
target_link_libraries (voxelizer ${PCL_LIBRARIES})
ament_target_dependencies(voxelizer rclcpp sensor_msgs)

add_executable(cloud_processor src/process_cloud.cpp)
target_link_libraries (cloud_processor ${PCL_LIBRARIES})
ament_target_dependencies(cloud_processor rclcpp sensor_msgs)

install(TARGETS
  voxelizer
  cloud_processor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
