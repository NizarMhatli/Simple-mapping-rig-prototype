cmake_minimum_required(VERSION 3.8)
project(jig_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Data: URDF, configs (gz_bridge.yaml, fast_lio_sim.yaml), launch, meshes, worlds, rviz
install(DIRECTORY urdf config launch meshes worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes: one install() per script, because RENAME only works on a single file.
# The RENAME value is the executable name used in mapping.launch.py.
install(PROGRAMS
  jig_description/jig_path_motion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jig_path_motion
)
install(PROGRAMS
  jig_description/lidar_to_fastlio.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_to_fastlio
)

# Godot export tools (not ROS nodes): run with `ros2 run jig_description export_maps.sh`
install(PROGRAMS
  jig_description/export_maps.sh
  jig_description/map_to_godot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()   # keep this last