#include "task_allocate/astar_path_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>

#include "task_allocate/geometry_utils.h"

namespace task_allocate
{

namespace
{
struct QueueNode
{
    int index;
    double f;
    double g;
};

struct QueueGreater
{
    bool operator()(const QueueNode& a, const QueueNode& b) const
    {
        return a.f > b.f;
    }
};
}

AStarPathPlanner::AStarPathPlanner()
{
}

void AStarPathPlanner::setParams(const AStarParams& params)
{
    params_ = params;
    params_.resolution = std::max(0.1, params_.resolution);
    if(params_.x_max <= params_.x_min + params_.resolution)
    {
        params_.x_max = params_.x_min + 10.0;
    }
    if(params_.y_max <= params_.y_min + params_.resolution)
    {
        params_.y_max = params_.y_min + 10.0;
    }
}

void AStarPathPlanner::setObstacles(const std::vector<StaticObstacle>& obstacles)
{
    obstacles_ = obstacles;
}

void AStarPathPlanner::setSensorObstaclePoints(const std::vector<Eigen::Vector3d>& points)
{
    sensor_obstacle_points_ = points;
}

int AStarPathPlanner::gridWidth() const
{
    return static_cast<int>(std::floor((params_.x_max - params_.x_min) / params_.resolution)) + 1;
}

int AStarPathPlanner::gridHeight() const
{
    return static_cast<int>(std::floor((params_.y_max - params_.y_min) / params_.resolution)) + 1;
}

bool AStarPathPlanner::worldToGrid(double x, double y, int& ix, int& iy) const
{
    ix = static_cast<int>(std::round((x - params_.x_min) / params_.resolution));
    iy = static_cast<int>(std::round((y - params_.y_min) / params_.resolution));
    return inGrid(ix, iy);
}

Eigen::Vector3d AStarPathPlanner::gridToWorld(int ix, int iy, double z) const
{
    return Eigen::Vector3d(params_.x_min + ix * params_.resolution,
                           params_.y_min + iy * params_.resolution,
                           z + params_.path_z_offset);
}

int AStarPathPlanner::toIndex(int ix, int iy) const
{
    return iy * gridWidth() + ix;
}

bool AStarPathPlanner::inGrid(int ix, int iy) const
{
    return ix >= 0 && iy >= 0 && ix < gridWidth() && iy < gridHeight();
}

bool AStarPathPlanner::occupied(int ix, int iy) const
{
    if(!inGrid(ix, iy))
    {
        return true;
    }
    const double x = params_.x_min + ix * params_.resolution;
    const double y = params_.y_min + iy * params_.resolution;
    for(size_t i = 0; i < obstacles_.size(); ++i)
    {
        if(isPointInsideObstacle2D(x, y, obstacles_[i], params_.obstacle_inflation, params_.avoid_danger_zone))
        {
            return true;
        }
    }

    if(params_.use_sensor_obstacles)
    {
        const double r = std::max(params_.sensor_obstacle_inflation, params_.resolution);
        const double r2 = r * r;
        for(size_t i = 0; i < sensor_obstacle_points_.size(); ++i)
        {
            const double dx = sensor_obstacle_points_[i].x() - x;
            const double dy = sensor_obstacle_points_[i].y() - y;
            if(dx * dx + dy * dy <= r2)
            {
                return true;
            }
        }
    }
    return false;
}

bool AStarPathPlanner::segmentCollisionFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
    if(!isLineSegmentCollisionFree2D(a, b, obstacles_, params_.obstacle_inflation,
                                     params_.avoid_danger_zone, params_.resolution * 0.5))
    {
        return false;
    }

    if(params_.use_sensor_obstacles)
    {
        const Eigen::Vector2d a2(a.x(), a.y());
        const Eigen::Vector2d b2(b.x(), b.y());
        const double r = std::max(params_.sensor_obstacle_inflation, params_.resolution);
        for(size_t i = 0; i < sensor_obstacle_points_.size(); ++i)
        {
            const Eigen::Vector2d p(sensor_obstacle_points_[i].x(), sensor_obstacle_points_[i].y());
            if(distancePointToSegment2D(p, a2, b2) <= r)
            {
                return false;
            }
        }
    }
    return true;
}

double AStarPathPlanner::heuristic(int ix, int iy, int gx, int gy) const
{
    const double dx = static_cast<double>(ix - gx);
    const double dy = static_cast<double>(iy - gy);
    return std::sqrt(dx * dx + dy * dy) * params_.resolution;
}

PathPlan AStarPathPlanner::plan(int uav_id, int task_id, const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const
{
    PathPlan out;
    out.uav_id = uav_id;
    out.task_id = task_id;
    out.planner_name = "A*";

    int sx = 0, sy = 0, gx = 0, gy = 0;
    if(!worldToGrid(start.x(), start.y(), sx, sy) || !worldToGrid(goal.x(), goal.y(), gx, gy))
    {
        out.success = false;
        out.used_fallback_straight_line = false;
        out.message = "start_or_goal_out_of_astar_map";
        out.waypoints.clear();
        out.length = 0.0;
        return out;
    }

    if(occupied(sx, sy) || occupied(gx, gy))
    {
        out.success = false;
        out.used_fallback_straight_line = false;
        out.message = "start_or_goal_inside_obstacle_after_inflation";
        out.waypoints.clear();
        out.length = 0.0;
        return out;
    }

    const int w = gridWidth();
    const int h = gridHeight();
    const int total = w * h;
    const int start_index = toIndex(sx, sy);
    const int goal_index = toIndex(gx, gy);

    std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
    std::vector<int> parent(total, -1);
    std::vector<unsigned char> closed(total, 0);
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open;

    g_score[start_index] = 0.0;
    open.push(QueueNode{start_index, heuristic(sx, sy, gx, gy), 0.0});

    const int dx4[4] = {1, -1, 0, 0};
    const int dy4[4] = {0, 0, 1, -1};
    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int neighbor_num = params_.allow_diagonal ? 8 : 4;

    bool found = false;
    while(!open.empty())
    {
        QueueNode cur = open.top();
        open.pop();
        if(closed[cur.index])
        {
            continue;
        }
        closed[cur.index] = 1;
        if(cur.index == goal_index)
        {
            found = true;
            break;
        }

        const int cx = cur.index % w;
        const int cy = cur.index / w;
        for(int n = 0; n < neighbor_num; ++n)
        {
            const int nx = cx + (params_.allow_diagonal ? dx8[n] : dx4[n]);
            const int ny = cy + (params_.allow_diagonal ? dy8[n] : dy4[n]);
            if(!inGrid(nx, ny) || occupied(nx, ny))
            {
                continue;
            }

            // Prevent diagonal corner cutting between two occupied cells.
            if(params_.allow_diagonal && std::abs(nx - cx) == 1 && std::abs(ny - cy) == 1)
            {
                if(occupied(nx, cy) || occupied(cx, ny))
                {
                    continue;
                }
            }

            const int ni = toIndex(nx, ny);
            if(closed[ni])
            {
                continue;
            }
            const double step_cost = (std::abs(nx - cx) + std::abs(ny - cy) == 2) ?
                                     std::sqrt(2.0) * params_.resolution : params_.resolution;
            const double tentative = g_score[cur.index] + step_cost;
            if(tentative < g_score[ni])
            {
                g_score[ni] = tentative;
                parent[ni] = cur.index;
                const double f = tentative + heuristic(nx, ny, gx, gy);
                open.push(QueueNode{ni, f, tentative});
            }
        }
    }

    if(!found)
    {
        out.success = false;
        out.used_fallback_straight_line = false;
        out.message = "astar_failed_no_path";
        out.waypoints.clear();
        out.length = 0.0;
        return out;
    }

    std::vector<Eigen::Vector3d> raw_path = reconstructPath(parent, start_index, goal_index,
                                                            start.z(), goal.z(), start, goal);
    if(params_.smooth_path)
    {
        out.waypoints = smoothPath(raw_path);
    }
    else
    {
        out.waypoints = raw_path;
    }

    out.success = true;
    out.used_fallback_straight_line = false;
    out.message = "astar_success";
    out.length = pathLength(out.waypoints);
    return out;
}

std::vector<Eigen::Vector3d> AStarPathPlanner::reconstructPath(const std::vector<int>& parent,
                                                               int start_index,
                                                               int goal_index,
                                                               double start_z,
                                                               double goal_z,
                                                               const Eigen::Vector3d& real_start,
                                                               const Eigen::Vector3d& real_goal) const
{
    const int w = gridWidth();
    std::vector<int> ids;
    int cur = goal_index;
    while(cur >= 0)
    {
        ids.push_back(cur);
        if(cur == start_index)
        {
            break;
        }
        cur = parent[cur];
    }
    std::reverse(ids.begin(), ids.end());

    std::vector<Eigen::Vector3d> path;
    path.reserve(ids.size() + 2);
    path.push_back(real_start + Eigen::Vector3d(0.0, 0.0, params_.path_z_offset));

    for(size_t i = 1; i + 1 < ids.size(); ++i)
    {
        const int idx = ids[i];
        const int ix = idx % w;
        const int iy = idx / w;
        const double t = static_cast<double>(i) / static_cast<double>(std::max<size_t>(1, ids.size() - 1));
        const double z = start_z + t * (goal_z - start_z);
        path.push_back(gridToWorld(ix, iy, z));
    }

    path.push_back(real_goal + Eigen::Vector3d(0.0, 0.0, params_.path_z_offset));
    return path;
}

std::vector<Eigen::Vector3d> AStarPathPlanner::smoothPath(const std::vector<Eigen::Vector3d>& raw_path) const
{
    if(raw_path.size() <= 2)
    {
        return raw_path;
    }

    std::vector<Eigen::Vector3d> smoothed;
    smoothed.push_back(raw_path.front());
    size_t anchor = 0;
    while(anchor + 1 < raw_path.size())
    {
        size_t best = anchor + 1;
        for(size_t j = raw_path.size() - 1; j > anchor + 1; --j)
        {
            if(segmentCollisionFree(raw_path[anchor], raw_path[j]))
            {
                best = j;
                break;
            }
        }
        smoothed.push_back(raw_path[best]);
        anchor = best;
    }
    return smoothed;
}

} // namespace task_allocate
