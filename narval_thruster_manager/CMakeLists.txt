cmake_minimum_required(VERSION 3.8)
project(narval_thruster_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(narval_thruster_manager SHARED src/thruster_manager.cpp src/thruster_manager_node.cpp)
target_include_directories(narval_thruster_manager PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(narval_thruster_manager rclcpp rclcpp_lifecycle urdf tf2_ros sensor_msgs geometry_msgs kdl_parser rclcpp_components)

add_executable(publish_wrenches src/publish_wrenches.cpp)
target_include_directories(publish_wrenches PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(publish_wrenches narval_thruster_manager)

rclcpp_components_register_node(narval_thruster_manager
  PLUGIN "thruster_manager::ThrusterManagerNode"
  EXECUTABLE thruster_manager_node)


install(TARGETS narval_thruster_manager publish_wrenches
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_libraries(narval_thruster_manager)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp_lifecycle rclcpp_components)

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
