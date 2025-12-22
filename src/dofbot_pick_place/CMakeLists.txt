cmake_minimum_required(VERSION 3.8)
project(dofbot_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ros_gz_interfaces REQUIRED)




# --- Pick Place Node ---
add_executable(pick_place_node src/pick_place.cpp)
ament_target_dependencies(pick_place_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  ros_gz_interfaces


              # ✅ Added here
)

# --- Test Node ---
add_executable(pick_place_test_node src/pick_place_test.cpp)
ament_target_dependencies(pick_place_test_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
 ros_gz_interfaces


              # ✅ Added here too
)

install(TARGETS
  pick_place_node
  pick_place_test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
