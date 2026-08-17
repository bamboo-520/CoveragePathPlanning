#include "task_allocate/mappo_actor.h"

#include <algorithm>
#include <cmath>

#include "task_allocate/geometry_utils.h"

namespace task_allocate
{

LinearMappoActor::LinearMappoActor()
    : max_distance_(90.0), obstacle_risk_enabled_(true), obstacle_risk_weight_(4.0),
      obstacle_risk_safe_margin_(5.0), risk_include_danger_zone_(true), risk_sample_num_(50)
{
    // [bias, near_score, priority_norm, energy_norm, capability, availability, task_unassigned]
    weights_ = { -1.0, 3.0, 2.0, 0.5, 5.0, 2.0, 1.0 };
    // [bias, energy_norm, no_feasible_task]
    idle_weights_ = { 0.0, -0.2, 2.0 };
}

void LinearMappoActor::setWeights(const std::vector<double>& weights)
{
    if(weights.size() == 7)
    {
        weights_ = weights;
    }
}

void LinearMappoActor::setIdleWeights(const std::vector<double>& weights)
{
    if(weights.size() == 3)
    {
        idle_weights_ = weights;
    }
}

void LinearMappoActor::setMaxDistance(double max_distance)
{
    max_distance_ = std::max(1.0, max_distance);
}

void LinearMappoActor::setObstacleRiskEnabled(bool enabled)
{
    obstacle_risk_enabled_ = enabled;
}

void LinearMappoActor::setObstacleRiskWeight(double weight)
{
    obstacle_risk_weight_ = std::max(0.0, weight);
}

void LinearMappoActor::setObstacleRiskSafeMargin(double margin)
{
    obstacle_risk_safe_margin_ = std::max(0.1, margin);
}

void LinearMappoActor::setRiskIncludeDangerZone(bool include_danger_zone)
{
    risk_include_danger_zone_ = include_danger_zone;
}

void LinearMappoActor::setRiskSampleNum(int sample_num)
{
    risk_sample_num_ = std::max(5, sample_num);
}

void LinearMappoActor::setObstacles(const std::vector<StaticObstacle>& obstacles)
{
    obstacles_ = obstacles;
}

void LinearMappoActor::setSensorObstaclePoints(const std::vector<Eigen::Vector3d>& points)
{
    sensor_obstacle_points_ = points;
}

double LinearMappoActor::calcTaskLogit(const UavInfo& uav, const TaskInfo& task) const
{
    const double distance = (uav.position - task.position).norm();
    const double near_score = 1.0 - std::min(distance / max_distance_, 1.0);
    const double priority_norm = std::max(0.0, std::min(task.priority / 10.0, 1.0));
    const double energy_norm = std::max(0.0, std::min(uav.energy / 100.0, 1.0));
    const double capability = hasCapability(uav, task.type) ? 1.0 : 0.0;
    const double availability = uav.available ? 1.0 : 0.0;
    const double unassigned = task.assigned ? 0.0 : 1.0;

    const double obs[7] = {1.0, near_score, priority_norm, energy_norm, capability, availability, unassigned};
    double logit = 0.0;
    for(size_t i = 0; i < weights_.size(); ++i)
    {
        logit += weights_[i] * obs[i];
    }

    if(obstacle_risk_enabled_)
    {
        logit -= obstacle_risk_weight_ * calcObstacleRisk(uav, task);
    }

    if(!uav.available)
    {
        logit -= 100.0;
    }
    if(!hasCapability(uav, task.type))
    {
        logit -= 100.0;
    }
    if(task.assigned)
    {
        logit -= 100.0;
    }
    return logit;
}

double LinearMappoActor::calcIdleLogit(const UavInfo& uav, const std::vector<TaskInfo>& tasks) const
{
    int feasible_task_num = 0;
    for(size_t i = 0; i < tasks.size(); ++i)
    {
        if(!tasks[i].assigned && hasCapability(uav, tasks[i].type))
        {
            feasible_task_num++;
        }
    }
    const double energy_norm = std::max(0.0, std::min(uav.energy / 100.0, 1.0));
    const double no_feasible = feasible_task_num == 0 ? 1.0 : 0.0;
    return idle_weights_[0] + idle_weights_[1] * energy_norm + idle_weights_[2] * no_feasible;
}

double LinearMappoActor::calcObstacleRisk(const UavInfo& uav, const TaskInfo& task) const
{
    if(!obstacle_risk_enabled_)
    {
        return 0.0;
    }

    double risk = 0.0;
    if(!obstacles_.empty())
    {
        risk += segmentObstacleRisk(uav.position, task.position, obstacles_, obstacle_risk_safe_margin_,
                                    risk_include_danger_zone_, risk_sample_num_);
    }

    if(!sensor_obstacle_points_.empty())
    {
        const Eigen::Vector2d a(uav.position.x(), uav.position.y());
        const Eigen::Vector2d b(task.position.x(), task.position.y());
        const double margin = std::max(0.1, obstacle_risk_safe_margin_);
        double min_dist = 1e9;
        int close_count = 0;
        for(size_t i = 0; i < sensor_obstacle_points_.size(); ++i)
        {
            const Eigen::Vector2d p(sensor_obstacle_points_[i].x(), sensor_obstacle_points_[i].y());
            const double d = distancePointToSegment2D(p, a, b);
            min_dist = std::min(min_dist, d);
            if(d < margin)
            {
                close_count++;
            }
        }
        if(min_dist < margin)
        {
            risk += (margin - min_dist) / margin + 0.02 * static_cast<double>(close_count);
        }
    }

    return risk;
}

bool LinearMappoActor::hasCapability(const UavInfo& uav, int task_type)
{
    if(task_type < 0 || task_type >= static_cast<int>(uav.capability.size()))
    {
        return false;
    }
    return uav.capability[task_type] != 0;
}

} // namespace task_allocate
