#ifndef TASK_ALLOCATE_MAPPO_ACTOR_H
#define TASK_ALLOCATE_MAPPO_ACTOR_H

#include <vector>

#include "task_allocate/common_types.h"

namespace task_allocate
{

class LinearMappoActor
{
public:
    LinearMappoActor();

    void setWeights(const std::vector<double>& weights);
    void setIdleWeights(const std::vector<double>& weights);
    void setMaxDistance(double max_distance);
    void setObstacleRiskEnabled(bool enabled);
    void setObstacleRiskWeight(double weight);
    void setObstacleRiskSafeMargin(double margin);
    void setRiskIncludeDangerZone(bool include_danger_zone);
    void setRiskSampleNum(int sample_num);
    void setObstacles(const std::vector<StaticObstacle>& obstacles);
    void setSensorObstaclePoints(const std::vector<Eigen::Vector3d>& points);

    double calcTaskLogit(const UavInfo& uav, const TaskInfo& task) const;
    double calcIdleLogit(const UavInfo& uav, const std::vector<TaskInfo>& tasks) const;
    double calcObstacleRisk(const UavInfo& uav, const TaskInfo& task) const;

    static bool hasCapability(const UavInfo& uav, int task_type);

private:
    std::vector<double> weights_;
    std::vector<double> idle_weights_;
    double max_distance_;
    bool obstacle_risk_enabled_;
    double obstacle_risk_weight_;
    double obstacle_risk_safe_margin_;
    bool risk_include_danger_zone_;
    int risk_sample_num_;
    std::vector<StaticObstacle> obstacles_;
    std::vector<Eigen::Vector3d> sensor_obstacle_points_;
};

} // namespace task_allocate

#endif // TASK_ALLOCATE_MAPPO_ACTOR_H
