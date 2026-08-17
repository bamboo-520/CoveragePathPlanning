# fix7: 解决传感器急停导致无法起飞

问题现象：运行 `auto_dispatch_commands:=true` 后，RViz 有 A* 路径，但 `/uavX/prometheus/swarm_command` 一直是 `sensor_stop_hold`，无人机不执行起飞。

原因：fix6 中传感器急停保护在 OFFBOARD 准备/起飞之前就生效，2D 雷达近场可能检测到机体自身、起落架或近场伪点，导致节点一直发布 Hold。

本版修改：

1. 急停保护只在 `sensor_astar_path_follow` 阶段生效；
2. `offboard_arm_prepare` 和 `takeoff_before_path` 阶段不再被急停打断；
3. `minDistance2D()` 忽略 UAV 周围 `self_filter_radius=1.2m` 的近场点；
4. 默认急停距离调整为 `1.0m`，先保证能起飞并进入 A* 路径跟踪。

替换后重新编译：

```bash
cd /home/amov/Prometheus
rm -rf build/task_allocate
catkin_make --source Modules/task_allocate --build build/task_allocate
source devel/setup.bash
```

运行：

```bash
roslaunch prometheus_task_allocate sitl_swarm_5uav_mappo_sensor_astar.launch auto_dispatch_commands:=true
```

检查命令序列：

```bash
rostopic echo /uav1/prometheus/swarm_command
```

应先看到：

- `/prometheus_task_allocate/offboard_arm_prepare`
- `/prometheus_task_allocate/takeoff_before_path`
- `/prometheus_task_allocate/sensor_astar_path_follow`

不应一开始就进入 `sensor_stop_hold`。
