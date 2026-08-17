echo "========== 1. 编译 Prometheus msgs =========="
catkin_make --source Modules/common/msgs --build build/msgs
source devel/setup.bash
echo "========== 2. 编译 quadrotor_msgs =========="
catkin_make \
  --source Modules/ego_planner/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs \
  --build build/quadrotor_msgs
source devel/setup.bash
echo "========== 3. 编译 ground_station =========="
catkin_make --source Modules/ground_station --build build/ground_station
source devel/setup.bash
echo "========== 4. 编译 swarm_control =========="
catkin_make --source Modules/swarm_control --build build/swarm_control
source devel/setup.bash
echo "========== 5. 检查功能包 =========="
rospack profile
rospack find quadrotor_msgs
rospack find prometheus_swarm_control
echo "========== compile_swarm finished =========="
# catkin_make --source Modules/common/msgs --build build/msgs
# catkin_make --source Modules/ground_station --build build/ground_station
# catkin_make --source Modules/swarm_control --build build/swarm_control