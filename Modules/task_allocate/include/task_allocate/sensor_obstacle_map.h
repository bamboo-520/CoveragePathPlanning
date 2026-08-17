#ifndef TASK_ALLOCATE_SENSOR_OBSTACLE_MAP_H
#define TASK_ALLOCATE_SENSOR_OBSTACLE_MAP_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

namespace task_allocate
{

struct SensorMapParams
{
    bool enabled;
    bool subscribe_pointcloud;
    bool subscribe_laserscan;
    bool sensor_points_are_world_frame;
    double max_range;
    double z_min;
    double z_max;
    double downsample_resolution;
    double point_lifetime;
    int max_points;
    // Points closer than this 2D radius to the UAV are ignored when computing
    // emergency stop distance. This filters self-body / landing gear / sensor
    // near-field returns that otherwise keep the vehicle in Hold.
    double self_filter_radius;

    SensorMapParams()
        : enabled(true), subscribe_pointcloud(true), subscribe_laserscan(true),
          sensor_points_are_world_frame(false), max_range(35.0), z_min(-0.5), z_max(8.0),
          downsample_resolution(0.6), point_lifetime(8.0), max_points(5000),
          self_filter_radius(1.2)
    {
    }
};

class SensorObstacleMap
{
public:
    SensorObstacleMap();

    void setParams(const SensorMapParams& params);
    void addPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       const Eigen::Vector3d& uav_position,
                       double uav_yaw,
                       const ros::Time& stamp);
    void addLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg,
                      const Eigen::Vector3d& uav_position,
                      double uav_yaw,
                      const ros::Time& stamp);

    std::vector<Eigen::Vector3d> getObstaclePoints(const ros::Time& now) const;
    bool hasRecentData(const ros::Time& now) const;
    double minDistance2D(const Eigen::Vector3d& position, const ros::Time& now) const;
    void clear();

private:
    struct MapPoint
    {
        Eigen::Vector3d p;
        ros::Time stamp;
    };

    Eigen::Vector3d transformSensorPointToWorld(const Eigen::Vector3d& local_point,
                                                const Eigen::Vector3d& uav_position,
                                                double uav_yaw) const;
    void insertPoint(const Eigen::Vector3d& p, const ros::Time& stamp);
    void prune(const ros::Time& now) const;
    std::string cellKey(const Eigen::Vector3d& p) const;

private:
    SensorMapParams params_;
    mutable std::vector<MapPoint> points_;
};

} // namespace task_allocate

#endif // TASK_ALLOCATE_SENSOR_OBSTACLE_MAP_H
