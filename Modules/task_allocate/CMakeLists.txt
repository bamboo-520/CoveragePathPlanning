cmake_minimum_required(VERSION 3.0.2)
project(prometheus_task_allocate)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  gazebo_msgs
  sensor_msgs
  prometheus_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    gazebo_msgs
    sensor_msgs
    prometheus_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(swarm_task_assignment_mappo
  src/swarm_task_assignment_mappo.cpp
  src/mappo_actor.cpp
  src/mappo_sequential_solver.cpp
  src/astar_path_planner.cpp
  src/geometry_utils.cpp
  src/visualization_utils.cpp
  src/sensor_obstacle_map.cpp
  src/path_command_dispatcher.cpp
)
add_dependencies(swarm_task_assignment_mappo ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_task_assignment_mappo ${catkin_LIBRARIES})

install(TARGETS swarm_task_assignment_mappo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
