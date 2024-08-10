cmake_minimum_required(VERSION 3.8)
project(moveit_reach_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka_msgs REQUIRED)

add_executable(moveit_reach_target src/moveit_reach_target.cpp)
add_executable(moveit_get_eefpose src/moveit_get_eefpose.cpp)
add_executable(sim_reach_target src/sim_reach_target.cpp)
add_executable(publish_trajectory src/publish_trajectory.cpp)

target_include_directories(moveit_reach_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(moveit_get_eefpose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(sim_reach_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(publish_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_compile_features(moveit_reach_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_reach_target
  "moveit_ros_planning_interface"
  "rclcpp"
  franka_msgs
)
target_compile_features(moveit_get_eefpose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_get_eefpose
  "moveit_ros_planning_interface"
  "rclcpp"
  franka_msgs
)
target_compile_features(sim_reach_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_reach_target
  "moveit_ros_planning_interface"
  "rclcpp"
  franka_msgs
)
target_compile_features(publish_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  publish_trajectory
  "moveit_ros_planning_interface"
  "rclcpp"
  franka_msgs
)



install(TARGETS moveit_reach_target
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moveit_get_eefpose
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS sim_reach_target
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS publish_trajectory
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
