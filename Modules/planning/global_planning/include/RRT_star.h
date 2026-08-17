#ifndef _RRT_STAR_H
#define _RRT_STAR_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <random>
#include <vector>
#include <limits>

#include <nav_msgs/Path.h>

// 直接复用 RRT 中已经定义好的 RRTNode
#include "RRT.h"
#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

#define NODE_NAME "Global_Planner [RRT*]"

namespace Global_Planning
{

extern ros::Publisher message_pub;

class RRTStar
{
public:
  RRTStar() {}
  ~RRTStar() {}

  enum
  {
    REACH_END = 1,
    NO_PATH = 2
  };

  Occupy_map::Ptr Occupy_map_ptr;

  void init(ros::NodeHandle& nh);
  void reset();

  int  search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
  bool check_safety(Eigen::Vector3d& cur_pos, double safe_distance);

  std::vector<Eigen::Vector3d> getPath() const;
  nav_msgs::Path get_ros_path() const;

  typedef std::shared_ptr<RRTStar> Ptr;

private:
  bool is_2D_{true};
  double fly_height_2D_{1.0};

  int max_iter_{5000};
  double step_size_{0.5};
  double goal_radius_{0.5};
  double goal_sample_rate_{0.1};
  double collision_check_res_{0.1};
  double near_radius_{1.5};

  Eigen::Vector3d min_bound_{-5,-5,-0.5};
  Eigen::Vector3d max_bound_{ 5, 5, 2.5};

  Eigen::Vector3d start_pos_{0,0,0};
  Eigen::Vector3d goal_pos_{0,0,0};

  std::vector<RRTNode> tree_;
  std::vector<Eigen::Vector3d> path_;

  std::mt19937 rng_{std::random_device{}()};
  std::uniform_real_distribution<double> uni01_{0.0, 1.0};

private:
  Eigen::Vector3d sampleFree();
  int nearest(const Eigen::Vector3d& x) const;
  std::vector<int> near(const Eigen::Vector3d& x, double radius) const;
  Eigen::Vector3d steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

  bool isStateValid(const Eigen::Vector3d& p) const;
  bool isSegmentValid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

  void buildPath(int goal_idx);
};

}

#endif
