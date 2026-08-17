#ifndef TASK_ALLOCATE_COMMON_TYPES_H
#define TASK_ALLOCATE_COMMON_TYPES_H

#include <Eigen/Eigen>
#include <string>
#include <vector>

namespace task_allocate
{

enum TaskType
{
    ATTACK = 0,
    RECON  = 1,
    JAM    = 2
};

inline std::string taskTypeToString(int type)
{
    switch(type)
    {
        case ATTACK: return "Attack";
        case RECON:  return "Recon";
        case JAM:    return "Jam";
        default:     return "Unknown";
    }
}

inline std::string taskTypeToChinese(int type)
{
    switch(type)
    {
        case ATTACK: return "攻击";
        case RECON:  return "侦察";
        case JAM:    return "干扰";
        default:     return "未知";
    }
}

struct TaskInfo
{
    int id;
    int type;
    Eigen::Vector3d position;
    double priority;
    bool assigned;

    TaskInfo()
        : id(0), type(RECON), position(Eigen::Vector3d::Zero()), priority(1.0), assigned(false)
    {
    }
};

struct UavInfo
{
    int id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double energy;
    bool available;
    bool has_state;
    double yaw;
    std::vector<int> capability;

    UavInfo()
        : id(0), position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
          energy(100.0), available(true), has_state(false), yaw(0.0), capability(3, 1)
    {
    }
};

struct StaticObstacle
{
    int id;
    // 0 = box/building, 1 = cylinder/tower, 2 = danger zone cylinder.
    int type;
    Eigen::Vector3d position;
    // Box: [length_x, width_y, height_z]. Cylinder/danger: [radius, unused, height_z].
    Eigen::Vector3d size;
    double r;
    double g;
    double b;
    double a;

    StaticObstacle()
        : id(0), type(0), position(Eigen::Vector3d::Zero()), size(Eigen::Vector3d::Ones()),
          r(0.5), g(0.5), b(0.5), a(0.85)
    {
    }

    bool isDangerZone() const
    {
        return type == 2;
    }
};

struct AssignmentResult
{
    int uav_id;
    int task_id;
    int task_index;
    int task_type;
    Eigen::Vector3d uav_position;
    Eigen::Vector3d task_position;
    double score;
    double distance;
    double obstacle_risk;

    AssignmentResult()
        : uav_id(0), task_id(-1), task_index(-1), task_type(-1),
          uav_position(Eigen::Vector3d::Zero()), task_position(Eigen::Vector3d::Zero()),
          score(0.0), distance(0.0), obstacle_risk(0.0)
    {
    }
};

struct AssignmentMetrics
{
    int total_uav;
    int total_task;
    int assigned_num;
    int unassigned_task_num;
    int idle_uav_num;
    double total_distance;
    double avg_distance;
    double total_obstacle_risk;
    double avg_obstacle_risk;
    double success_rate;
    double compute_time_ms;

    AssignmentMetrics()
        : total_uav(0), total_task(0), assigned_num(0), unassigned_task_num(0), idle_uav_num(0),
          total_distance(0.0), avg_distance(0.0), total_obstacle_risk(0.0), avg_obstacle_risk(0.0),
          success_rate(0.0), compute_time_ms(0.0)
    {
    }
};

struct PathPlan
{
    int uav_id;
    int task_id;
    bool success;
    bool used_fallback_straight_line;
    std::string planner_name;
    std::string message;
    std::vector<Eigen::Vector3d> waypoints;
    double length;

    PathPlan()
        : uav_id(0), task_id(-1), success(false), used_fallback_straight_line(false),
          planner_name("A*"), message(""), length(0.0)
    {
    }
};

struct AStarParams
{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double resolution;
    double obstacle_inflation;
    bool avoid_danger_zone;
    bool allow_diagonal;
    bool smooth_path;
    double path_z_offset;
    bool use_sensor_obstacles;
    double sensor_obstacle_inflation;

    AStarParams()
        : x_min(-20.0), x_max(80.0), y_min(-35.0), y_max(35.0), resolution(0.5),
          obstacle_inflation(1.2), avoid_danger_zone(false), allow_diagonal(true), smooth_path(true),
          path_z_offset(0.0), use_sensor_obstacles(true), sensor_obstacle_inflation(1.0)
    {
    }
};

} // namespace task_allocate

#endif // TASK_ALLOCATE_COMMON_TYPES_H
