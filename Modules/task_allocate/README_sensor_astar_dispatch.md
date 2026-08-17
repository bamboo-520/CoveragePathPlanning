# MAPPO传感器障碍物风险 + A*实际路径 + 可选控制下发版本

## 本版本定位

本版本继续保持“任务分配和路径规划解耦合”：先由 MAPPO 顺序决策 + 动作掩码完成 UAV→Task 分配，再对每个分配结果执行 A* 路径规划。

主要变化：

1. 删除预计路线/直线 fallback 路线；RViz 和 Gazebo 中只显示 A* 成功规划出的路径。
2. 障碍物不再通过 launch 中的 obstacle_list 硬编码参与规划。
3. 新增 `worlds/obstacle.world`，障碍物场景单独编码，后续可直接替换 world 文件测试不同场景。
4. 新增 `SensorObstacleMap`，从点云/激光话题构建障碍物点集。
5. MAPPO 任务分配风险项和 A* 占据判断默认使用传感器障碍物点。
6. 新增 `PathCommandDispatcher`，当 `auto_dispatch_commands:=true` 时，会按 A* 路径点逐点发布 `SwarmCommand::Move / XYZ_POS` 控制指令。
7. 新增简单传感器近距停车保护：若无人机附近传感器障碍物点过近，则发布当前位置保持指令。

## 目录结构

```text
task_allocate/
├── CMakeLists.txt
├── package.xml
├── include/task_allocate/
│   ├── common_types.h
│   ├── geometry_utils.h
│   ├── mappo_actor.h
│   ├── mappo_sequential_solver.h
│   ├── astar_path_planner.h
│   ├── sensor_obstacle_map.h
│   ├── path_command_dispatcher.h
│   └── visualization_utils.h
├── src/
│   ├── swarm_task_assignment_mappo.cpp
│   ├── mappo_actor.cpp
│   ├── mappo_sequential_solver.cpp
│   ├── astar_path_planner.cpp
│   ├── sensor_obstacle_map.cpp
│   ├── path_command_dispatcher.cpp
│   ├── geometry_utils.cpp
│   └── visualization_utils.cpp
├── launch/
│   └── sitl_swarm_5uav_mappo_sensor_astar.launch
├── worlds/
│   └── obstacle.world
└── rviz/
    └── mappo_task_assignment_astar.rviz
```

## 放置

建议先清理旧版本，避免 launch 文件太多：

```bash
cd /home/amov/Prometheus/Modules/task_allocate
rm -rf include src launch rviz worlds CMakeLists.txt package.xml README_*.md
```

复制新版本：

```bash
cp -r task_allocate_sensor_astar_dispatch_patch/* /home/amov/Prometheus/Modules/task_allocate/
```

## 编译

```bash
cd /home/amov/Prometheus
rm -rf build/task_allocate
catkin_make --source Modules/task_allocate --build build/task_allocate
source devel/setup.bash
```

## 运行：只看分配和 A* 路径

```bash
roslaunch prometheus_task_allocate sitl_swarm_5uav_mappo_sensor_astar.launch
```

## 运行：发布控制指令，按 A* 路径点飞行

```bash
roslaunch prometheus_task_allocate sitl_swarm_5uav_mappo_sensor_astar.launch auto_dispatch_commands:=true
```

## 传感器话题检查

默认订阅：

```text
/uav1/prometheus/sensors/pcl2
/uav1/prometheus/sensors/2Dlidar_scan
...
/uav5/prometheus/sensors/pcl2
/uav5/prometheus/sensors/2Dlidar_scan
```

如果你的实际话题不同，先查询：

```bash
rostopic list | grep -E "pcl|cloud|scan|lidar"
```

然后启动时修改模板，例如：

```bash
roslaunch prometheus_task_allocate sitl_swarm_5uav_mappo_sensor_astar.launch \
  sensor_pointcloud_topic_template:=/uav%d/prometheus/sensors/pcl2 \
  sensor_laserscan_topic_template:=/uav%d/prometheus/sensors/2Dlidar_scan
```

如果没有任何传感器话题，节点仍能运行，但 A* 看不到障碍物，只会在空地图上规划；这时需要先给无人机模型增加激光或点云传感器。

## 关键参数

```xml
<param name="use_param_obstacles_for_assignment" value="false" />
<param name="use_param_obstacles_for_astar" value="false" />
```

这两个参数默认关闭，表示不使用代码硬编码障碍物。

```xml
<param name="astar/use_sensor_obstacles" value="true" />
<param name="astar/sensor_obstacle_inflation" value="1.2" />
```

A* 使用传感器障碍物点并进行膨胀。

```xml
<param name="path_dispatch/enabled" value="true" />
<param name="path_dispatch/waypoint_reach_threshold" value="0.8" />
```

控制下发开启时，逐点发布路径点。

```xml
<param name="sensor_stop_failsafe/enabled" value="true" />
<param name="sensor_stop_failsafe/stop_distance" value="1.2" />
```

若传感器障碍物点距离无人机过近，则发布当前位置保持指令。

## 更换障碍物场景

直接复制或修改：

```text
/home/amov/Prometheus/Modules/task_allocate/worlds/obstacle.world
```

也可以启动时指定：

```bash
roslaunch prometheus_task_allocate sitl_swarm_5uav_mappo_sensor_astar.launch world:=/home/amov/your_world.world
```
