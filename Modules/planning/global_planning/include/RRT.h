#ifndef _RRT_H
#define _RRT_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <random>
#include <vector>
#include <limits>

#include <nav_msgs/Path.h>

#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

#define NODE_NAME "Global_Planner [RRT]"

namespace Global_Planning
{

extern ros::Publisher message_pub;

struct RRTNode
{
  Eigen::Vector3d pos;
  int parent;     // index of parent node in tree, -1 for root
  double cost;    // optional (for debug / future RRT*)
  RRTNode(const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), int pa = -1, double c = 0.0)
    : pos(p), parent(pa), cost(c) {}
};

class RRT
{
public:
  RRT() {}
  ~RRT() {}

  enum
  {
    REACH_END = 1,
    NO_PATH = 2
  };

  // 占据图类
  Occupy_map::Ptr Occupy_map_ptr;

  // 初始化/重置
  void init(ros::NodeHandle& nh);
  void reset();

  // 搜索与查询
  int  search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
  bool check_safety(Eigen::Vector3d& cur_pos, double safe_distance);

  // 结果输出
  std::vector<Eigen::Vector3d> getPath() const;
  nav_msgs::Path get_ros_path() const;

  typedef std::shared_ptr<RRT> Ptr;

private:
  // params
  bool is_2D_{true};
  double fly_height_2D_{1.0};

  int max_iter_{5000};
  double step_size_{0.5};           // 每次扩展步长
  double goal_radius_{0.5};         // 到达目标判据半径
  double goal_sample_rate_{0.1};    // 采样到目标的概率 [0,1]
  double collision_check_res_{0.1}; // 碰撞检测分辨率（沿边采样间隔）

  // bounds from map
  Eigen::Vector3d min_bound_{-5,-5,-0.5};
  Eigen::Vector3d max_bound_{ 5, 5, 2.5};

  // state
  Eigen::Vector3d start_pos_{0,0,0};
  Eigen::Vector3d goal_pos_{0,0,0};

  std::vector<RRTNode> tree_;
  std::vector<Eigen::Vector3d> path_;

  // RNG
  std::mt19937 rng_{std::random_device{}()};
  std::uniform_real_distribution<double> uni01_{0.0, 1.0};

private:
  // helpers
  Eigen::Vector3d sampleFree();
  int  nearest(const Eigen::Vector3d& x) const;
  Eigen::Vector3d steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

  bool isStateValid(const Eigen::Vector3d& p) const;
  bool isSegmentValid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

  void buildPath(int goal_idx);
};

} // namespace Global_Planning

#endif
