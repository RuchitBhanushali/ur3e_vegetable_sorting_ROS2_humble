cmake_minimum_required(VERSION 3.8)
project(ur3e_pick_place_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MoveIt + MTC
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

# Messages used by PlanningSceneInterface / collision objects
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# If you still use tf2 conversions anywhere
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ---- Pick & Place executable (main) ----
add_executable(pick_place_mtc
  src/pick_place_main.cpp
  src/pick_place_task.cpp
  src/scene_utils.cpp
)

ament_target_dependencies(pick_place_mtc
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_msgs
  geometry_msgs
  sensor_msgs 
  shape_msgs
  tf2_geometry_msgs
)

install(TARGETS pick_place_mtc
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (good practice)
install(DIRECTORY include/
  DESTINATION include
)

# Install launch + config (you said config is directly in package)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
