#ifndef TASK_ALLOCATE_VISUALIZATION_UTILS_H
#define TASK_ALLOCATE_VISUALIZATION_UTILS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <string>

#include "task_allocate/common_types.h"

namespace task_allocate
{

geometry_msgs::Point toPointMsg(const Eigen::Vector3d& p);
geometry_msgs::Pose toPoseMsg(const Eigen::Vector3d& p);
std_msgs::ColorRGBA makeColor(double r, double g, double b, double a);
void setMarkerColor(visualization_msgs::Marker& marker, double r, double g, double b, double a);

visualization_msgs::Marker makeBasicMarker(const std::string& frame_id,
                                           const std::string& ns,
                                           int id,
                                           int type,
                                           const Eigen::Vector3d& position);

std::string gazeboColorScript(double r, double g, double b, double a);
std::string makeGazeboSphereSdf(double radius,
                                double r,
                                double g,
                                double b,
                                double a,
                                bool with_collision);
std::string makeGazeboBoxSdf(const Eigen::Vector3d& size,
                             double r,
                             double g,
                             double b,
                             double a,
                             bool with_collision);
std::string makeGazeboCylinderSdf(double radius,
                                  double height,
                                  double r,
                                  double g,
                                  double b,
                                  double a,
                                  bool with_collision);

} // namespace task_allocate

#endif // TASK_ALLOCATE_VISUALIZATION_UTILS_H
