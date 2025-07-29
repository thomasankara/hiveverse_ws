cmake_minimum_required(VERSION 3.8)
project(hive_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch model worlds parameters
  DESTINATION share/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(multi_odom_tf_broadcaster src/multi_odom_tf_broadcaster.cpp)
ament_target_dependencies(multi_odom_tf_broadcaster rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS multi_odom_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

add_executable(lidar_frame_mapper src/lidar_frame_mapper.cpp)
ament_target_dependencies(lidar_frame_mapper rclcpp sensor_msgs)
install(TARGETS lidar_frame_mapper DESTINATION lib/${PROJECT_NAME})

add_executable(multi_robot_tf_and_lidar_mapper src/multi_robot_tf_and_lidar_mapper.cpp)
ament_target_dependencies(multi_robot_tf_and_lidar_mapper rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS multi_robot_tf_and_lidar_mapper DESTINATION lib/${PROJECT_NAME})


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