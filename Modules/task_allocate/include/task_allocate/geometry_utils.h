#ifndef TASK_ALLOCATE_GEOMETRY_UTILS_H
#define TASK_ALLOCATE_GEOMETRY_UTILS_H

#include <Eigen/Eigen>
#include <vector>

#include "task_allocate/common_types.h"

namespace task_allocate
{

double pathLength(const std::vector<Eigen::Vector3d>& pts);
double distancePointToSegment2D(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b);
double signedDistanceToObstacle2D(double x, double y, const StaticObstacle& obs);
bool isPointInsideObstacle2D(double x, double y, const StaticObstacle& obs, double inflation, bool avoid_danger_zone);
bool isLineSegmentCollisionFree2D(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b,
                                  const std::vector<StaticObstacle>& obstacles,
                                  double inflation,
                                  bool avoid_danger_zone,
                                  double sample_step);

double segmentObstacleRisk(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& goal,
                           const std::vector<StaticObstacle>& obstacles,
                           double safe_margin,
                           bool include_danger_zone,
                           int sample_num);

std::vector<Eigen::Vector3d> makeStraightPath(const Eigen::Vector3d& start,
                                              const Eigen::Vector3d& goal,
                                              int sample_num,
                                              double z_offset);

std::vector<Eigen::Vector3d> makeCurvedDemoPath(const Eigen::Vector3d& start,
                                                const Eigen::Vector3d& goal,
                                                int sample_num,
                                                double side_offset,
                                                double mid_height);

} // namespace task_allocate

#endif // TASK_ALLOCATE_GEOMETRY_UTILS_H
