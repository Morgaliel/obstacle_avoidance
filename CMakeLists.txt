cmake_minimum_required(VERSION 3.14)
project(obstacle_avoidance)

find_package(ament_cmake_auto REQUIRED)
find_package(visualization_msgs REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/obstacle_avoidance/obstacle_avoidance.hpp
  include/obstacle_avoidance/visibility_control.hpp
  include/obstacle_avoidance/occupancy_grid.hpp
  include/obstacle_avoidance/visualizer.hpp
  src/obstacle_avoidance.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/obstacle_avoidance/obstacle_avoidance_node.hpp
  include/obstacle_avoidance/occupancy_grid.hpp
  include/obstacle_avoidance/visualizer.hpp
  src/obstacle_avoidance_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "obstacle_avoidance::ObstacleAvoidanceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_obstacle_avoidance
    test/test_obstacle_avoidance.cpp
  )
  ament_target_dependencies(test_obstacle_avoidance ${TEST_SOURCES})
  target_link_libraries(test_obstacle_avoidance ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
