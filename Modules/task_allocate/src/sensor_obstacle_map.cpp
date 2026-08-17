#include "task_allocate/sensor_obstacle_map.h"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

namespace task_allocate
{

SensorObstacleMap::SensorObstacleMap()
{
}

void SensorObstacleMap::setParams(const SensorMapParams& params)
{
    params_ = params;
    params_.max_range = std::max(0.5, params_.max_range);
    params_.downsample_resolution = std::max(0.05, params_.downsample_resolution);
    params_.point_lifetime = std::max(0.1, params_.point_lifetime);
    params_.max_points = std::max(100, params_.max_points);
    params_.self_filter_radius = std::max(0.0, params_.self_filter_radius);
}

Eigen::Vector3d SensorObstacleMap::transformSensorPointToWorld(const Eigen::Vector3d& local_point,
                                                               const Eigen::Vector3d& uav_position,
                                                               double uav_yaw) const
{
    if(params_.sensor_points_are_world_frame)
    {
        return local_point;
    }

    const double c = std::cos(uav_yaw);
    const double s = std::sin(uav_yaw);
    Eigen::Vector3d p;
    p.x() = uav_position.x() + c * local_point.x() - s * local_point.y();
    p.y() = uav_position.y() + s * local_point.x() + c * local_point.y();
    p.z() = uav_position.z() + local_point.z();
    return p;
}

std::string SensorObstacleMap::cellKey(const Eigen::Vector3d& p) const
{
    const double r = params_.downsample_resolution;
    const int ix = static_cast<int>(std::floor(p.x() / r));
    const int iy = static_cast<int>(std::floor(p.y() / r));
    const int iz = static_cast<int>(std::floor(p.z() / r));
    std::ostringstream ss;
    ss << ix << "_" << iy << "_" << iz;
    return ss.str();
}

void SensorObstacleMap::insertPoint(const Eigen::Vector3d& p, const ros::Time& stamp)
{
    if(!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
    {
        return;
    }
    if(p.z() < params_.z_min || p.z() > params_.z_max)
    {
        return;
    }

    MapPoint mp;
    mp.p = p;
    mp.stamp = stamp;
    points_.push_back(mp);

    if(static_cast<int>(points_.size()) > params_.max_points)
    {
        const int erase_num = static_cast<int>(points_.size()) - params_.max_points;
        points_.erase(points_.begin(), points_.begin() + erase_num);
    }
}

void SensorObstacleMap::addPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                      const Eigen::Vector3d& uav_position,
                                      double uav_yaw,
                                      const ros::Time& stamp)
{
    if(!params_.enabled || !params_.subscribe_pointcloud)
    {
        return;
    }

    prune(stamp);
    std::map<std::string, Eigen::Vector3d> unique_cells;

    try
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            Eigen::Vector3d local(*iter_x, *iter_y, *iter_z);
            if(!std::isfinite(local.x()) || !std::isfinite(local.y()) || !std::isfinite(local.z()))
            {
                continue;
            }
            Eigen::Vector3d world = transformSensorPointToWorld(local, uav_position, uav_yaw);
            if((world - uav_position).norm() > params_.max_range)
            {
                continue;
            }
            unique_cells[cellKey(world)] = world;
        }
    }
    catch(...)
    {
        return;
    }

    for(std::map<std::string, Eigen::Vector3d>::const_iterator it = unique_cells.begin(); it != unique_cells.end(); ++it)
    {
        insertPoint(it->second, stamp);
    }
}

void SensorObstacleMap::addLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg,
                                     const Eigen::Vector3d& uav_position,
                                     double uav_yaw,
                                     const ros::Time& stamp)
{
    if(!params_.enabled || !params_.subscribe_laserscan)
    {
        return;
    }

    prune(stamp);
    std::map<std::string, Eigen::Vector3d> unique_cells;
    double angle = msg->angle_min;
    for(size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment)
    {
        const double r = msg->ranges[i];
        if(!std::isfinite(r) || r < msg->range_min || r > msg->range_max || r > params_.max_range)
        {
            continue;
        }
        Eigen::Vector3d local(r * std::cos(angle), r * std::sin(angle), 0.0);
        Eigen::Vector3d world = transformSensorPointToWorld(local, uav_position, uav_yaw);
        unique_cells[cellKey(world)] = world;
    }

    for(std::map<std::string, Eigen::Vector3d>::const_iterator it = unique_cells.begin(); it != unique_cells.end(); ++it)
    {
        insertPoint(it->second, stamp);
    }
}

void SensorObstacleMap::prune(const ros::Time& now) const
{
    const double lifetime = params_.point_lifetime;
    points_.erase(std::remove_if(points_.begin(), points_.end(),
                                 [&](const MapPoint& mp)
                                 {
                                     return (now - mp.stamp).toSec() > lifetime;
                                 }),
                  points_.end());
}

std::vector<Eigen::Vector3d> SensorObstacleMap::getObstaclePoints(const ros::Time& now) const
{
    prune(now);
    std::map<std::string, Eigen::Vector3d> unique_cells;
    for(size_t i = 0; i < points_.size(); ++i)
    {
        unique_cells[cellKey(points_[i].p)] = points_[i].p;
    }
    std::vector<Eigen::Vector3d> out;
    out.reserve(unique_cells.size());
    for(std::map<std::string, Eigen::Vector3d>::const_iterator it = unique_cells.begin(); it != unique_cells.end(); ++it)
    {
        out.push_back(it->second);
    }
    return out;
}

bool SensorObstacleMap::hasRecentData(const ros::Time& now) const
{
    prune(now);
    return !points_.empty();
}

double SensorObstacleMap::minDistance2D(const Eigen::Vector3d& position, const ros::Time& now) const
{
    prune(now);
    double min_dist = 1e9;
    for(size_t i = 0; i < points_.size(); ++i)
    {
        const double dx = points_[i].p.x() - position.x();
        const double dy = points_[i].p.y() - position.y();
        const double d = std::sqrt(dx * dx + dy * dy);
        // Filter very close returns around the UAV body. 2D lidar on the
        // Multisolo model can see propeller/landing gear/self-shadow points;
        // using them for failsafe makes the vehicle publish Hold forever.
        if(d < params_.self_filter_radius)
        {
            continue;
        }
        min_dist = std::min(min_dist, d);
    }
    return min_dist;
}

void SensorObstacleMap::clear()
{
    points_.clear();
}

} // namespace task_allocate
