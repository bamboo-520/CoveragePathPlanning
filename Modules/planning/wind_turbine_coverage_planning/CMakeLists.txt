cmake_minimum_required(VERSION 2.8.3)
project(wind_turbine_coverage_planning)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    visualization_msgs
    prometheus_msgs
)

catkin_package()

include_directories(
    SYSTEM
    include
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../../common/include
    #include了global_planning里的头文件
    ${PROJECT_SOURCE_DIR}/../global_planning/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(wind_turbine_model_pcl_pub
    src/wind_turbine_model_pcl_pub.cpp
)

target_link_libraries(wind_turbine_model_pcl_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(wind_turbine_coverage_planner
    src/wind_turbine_coverage_planner.cpp
)

target_link_libraries(wind_turbine_coverage_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(wind_turbine_coverage_plannner_reciprocate
    src/wind_turbine_coverage_plannner_reciprocate.cpp
)

target_link_libraries(wind_turbine_coverage_plannner_reciprocate
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(wind_turbine_real_model_pcl_pub
    src/wind_turbine_real_model_pcl_pub.cpp
)

target_link_libraries(wind_turbine_real_model_pcl_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(wind_turbine_real_coverage_planner
    src/wind_turbine_real_coverage_planner.cpp
)

target_link_libraries(wind_turbine_real_coverage_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(wind_turbine_real_coverage_planner_reciprocate
    src/wind_turbine_real_coverage_planner_reciprocate.cpp
)

target_link_libraries(wind_turbine_real_coverage_planner_reciprocate
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)