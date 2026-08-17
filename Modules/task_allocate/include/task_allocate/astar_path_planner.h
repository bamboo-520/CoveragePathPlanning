#ifndef TASK_ALLOCATE_ASTAR_PATH_PLANNER_H
#define TASK_ALLOCATE_ASTAR_PATH_PLANNER_H

#include <Eigen/Eigen>
#include <vector>

#include "task_allocate/common_types.h"

namespace task_allocate
{

class AStarPathPlanner
{
public:
    AStarPathPlanner();

    void setParams(const AStarParams& params);
    void setObstacles(const std::vector<StaticObstacle>& obstacles);
    void setSensorObstaclePoints(const std::vector<Eigen::Vector3d>& points);
    PathPlan plan(int uav_id, int task_id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const;

private:
    int gridWidth() const;
    int gridHeight() const;
    bool worldToGrid(double x, double y, int& ix, int& iy) const;
    Eigen::Vector3d gridToWorld(int ix, int iy, double z) const;
    int toIndex(int ix, int iy) const;
    bool inGrid(int ix, int iy) const;
    bool occupied(int ix, int iy) const;
    bool segmentCollisionFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
    double heuristic(int ix, int iy, int gx, int gy) const;
    std::vector<Eigen::Vector3d> reconstructPath(const std::vector<int>& parent,
                                                 int start_index,
                                                 int goal_index,
                                                 double start_z,
                                                 double goal_z,
                                                 const Eigen::Vector3d& real_start,
                                                 const Eigen::Vector3d& real_goal) const;
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& raw_path) const;

private:
    AStarParams params_;
    std::vector<StaticObstacle> obstacles_;
    std::vector<Eigen::Vector3d> sensor_obstacle_points_;
};

} // namespace task_allocate

#endif // TASK_ALLOCATE_ASTAR_PATH_PLANNER_H
