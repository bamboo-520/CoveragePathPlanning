#include "RRT_star.h"

namespace Global_Planning
{

void RRTStar::init(ros::NodeHandle& nh)
{
  // global parameters
  nh.param("global_planner/is_2D", is_2D_, true);
  nh.param("global_planner/fly_height_2D", fly_height_2D_, 1.0);

  // rrt parameters
  nh.param("rrt_star/max_iter", max_iter_, 5000);
  nh.param("rrt_star/step_size", step_size_, 0.5);
  nh.param("rrt_star/goal_radius", goal_radius_, 0.5);
  nh.param("rrt_star/goal_sample_rate", goal_sample_rate_, 0.1);
  nh.param("rrt_star/collision_check_res", collision_check_res_, 0.1);
  nh.param("rrt_star/near_radius", near_radius_, 1.5);

  // init occupy map
  Occupy_map_ptr.reset(new Occupy_map);
  Occupy_map_ptr->init(nh);

  // bounds from map (min_range_/max_range_ in Occupy_map)
  min_bound_ = Occupy_map_ptr->min_range_;
  max_bound_ = Occupy_map_ptr->max_range_;

  reset();

  pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,
              "RRT* init. max_iter=" + std::to_string(max_iter_) +
              ", step_size=" + std::to_string(step_size_) +
              ", goal_radius=" + std::to_string(goal_radius_) +
              ", near_radius=" + std::to_string(near_radius_));
}

void RRTStar::reset()
{
  tree_.clear();
  path_.clear();
}

bool RRTStar::check_safety(Eigen::Vector3d& cur_pos, double safe_distance)
{
  return Occupy_map_ptr->check_safety(cur_pos, safe_distance);
}

Eigen::Vector3d RRTStar::sampleFree()
{
  // goal biased sampling
  if (uni01_(rng_) < goal_sample_rate_)
  {
    return goal_pos_;
  }

  std::uniform_real_distribution<double> ux(min_bound_.x(), max_bound_.x());
  std::uniform_real_distribution<double> uy(min_bound_.y(), max_bound_.y());
  std::uniform_real_distribution<double> uz(min_bound_.z(), max_bound_.z());

  Eigen::Vector3d p(ux(rng_), uy(rng_), uz(rng_));

  if (is_2D_)
  {
    p.z() = fly_height_2D_;
  }
  return p;
}

int RRTStar::nearest(const Eigen::Vector3d& x) const
{
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)tree_.size(); i++)
  {
    const double d2 = (tree_[i].pos - x).squaredNorm();
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

std::vector<int> RRTStar::near(const Eigen::Vector3d& x, double radius) const
{
  std::vector<int> ids;
  const double r2 = radius * radius;
  for (int i = 0; i < (int)tree_.size(); ++i)
  {
    if ((tree_[i].pos - x).squaredNorm() <= r2) ids.push_back(i);
  }
  return ids;
}

Eigen::Vector3d RRTStar::steer(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
{
  Eigen::Vector3d dir = to - from;
  double d = dir.norm();
  if (d <= step_size_)
  {
    return to;
  }
  if (d < 1e-6)
  {
    return from;
  }
  dir /= d;
  return from + dir * step_size_;
}

bool RRTStar::isStateValid(const Eigen::Vector3d& p) const
{
  if (!Occupy_map_ptr->isInMap(p)) return false;
  if (Occupy_map_ptr->getOccupancy(p) == 1) return false;
  return true;
}

bool RRTStar::isSegmentValid(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
  // discretize along segment
  const double len = (b - a).norm();
  const double step = std::max(1e-3, collision_check_res_);
  const int n = std::max(1, (int)std::ceil(len / step));
  for (int i = 0; i <= n; i++)
  {
    const double t = (double)i / (double)n;
    Eigen::Vector3d p = a + t * (b - a);
    if (is_2D_) p.z() = fly_height_2D_;
    if (!isStateValid(p)) return false;
  }
  return true;
}

void RRTStar::buildPath(int goal_idx)
{
  path_.clear();
  int idx = goal_idx;
  while (idx >= 0)
  {
    path_.push_back(tree_[idx].pos);
    idx = tree_[idx].parent;
  }
  std::reverse(path_.begin(), path_.end());
}

int RRTStar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt)
{
  reset();

  start_pos_ = start_pt;
  goal_pos_  = end_pt;

  if (is_2D_)
  {
    start_pos_.z() = fly_height_2D_;
    goal_pos_.z()  = fly_height_2D_;
  }

  if (!isStateValid(start_pos_))
  {
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "RRT* start is invalid (occupied/out of map).");
    return NO_PATH;
  }
  if (!isStateValid(goal_pos_))
  {
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "RRT* goal is invalid (occupied/out of map).");
    return NO_PATH;
  }

  tree_.reserve((size_t)max_iter_ + 1);
  tree_.emplace_back(start_pos_, -1, 0.0);

  int goal_idx = -1;

  for (int iter = 0; iter < max_iter_; iter++)
  {
    Eigen::Vector3d x_rand = sampleFree();
    int nn = nearest(x_rand);
    if (nn < 0) continue;

    Eigen::Vector3d x_new = steer(tree_[nn].pos, x_rand);
    if (is_2D_) x_new.z() = fly_height_2D_;

    if (!isSegmentValid(tree_[nn].pos, x_new))
      continue;

    std::vector<int> near_ids = near(x_new, near_radius_);
    int best_parent = nn;
    double best_cost = tree_[nn].cost + (x_new - tree_[nn].pos).norm();
    for (int id : near_ids)
    {
      double cand_cost = tree_[id].cost + (x_new - tree_[id].pos).norm();
      if (cand_cost < best_cost && isSegmentValid(tree_[id].pos, x_new))
      {
        best_cost = cand_cost;
        best_parent = id;
      }
    }

    tree_.emplace_back(x_new, best_parent, best_cost);
    int new_idx = (int)tree_.size() - 1;

    for (int id : near_ids)
    {
      if (id == new_idx || id == best_parent) continue;
      double rewired_cost = tree_[new_idx].cost + (tree_[id].pos - x_new).norm();
      if (rewired_cost + 1e-9 < tree_[id].cost && isSegmentValid(x_new, tree_[id].pos))
      {
        tree_[id].parent = new_idx;
        tree_[id].cost = rewired_cost;
      }
    }

    if ((x_new - goal_pos_).norm() <= goal_radius_ && isSegmentValid(x_new, goal_pos_))
    {
      tree_.emplace_back(goal_pos_, new_idx, tree_[new_idx].cost + (goal_pos_ - x_new).norm());
      goal_idx = (int)tree_.size() - 1;
      break;
    }
  }

  if (goal_idx < 0)
  {
    int near_goal = nearest(goal_pos_);
    if (near_goal >= 0 && isSegmentValid(tree_[near_goal].pos, goal_pos_))
    {
      tree_.emplace_back(goal_pos_, near_goal, tree_[near_goal].cost + (goal_pos_ - tree_[near_goal].pos).norm());
      goal_idx = (int)tree_.size() - 1;
    }
  }

  if (goal_idx < 0)
  {
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,
                "RRT* failed to find path within max_iter=" + std::to_string(max_iter_));
    return NO_PATH;
  }

  buildPath(goal_idx);
  pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,
              "RRT* path found. nodes=" + std::to_string(tree_.size()) +
              ", waypoints=" + std::to_string(path_.size()));
  return REACH_END;
}

std::vector<Eigen::Vector3d> RRTStar::getPath() const
{
  return path_;
}

nav_msgs::Path RRTStar::get_ros_path() const
{
  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();

  for (size_t i = 0; i < path_.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = path_[i].x();
    pose.pose.position.y = path_[i].y();
    pose.pose.position.z = path_[i].z();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

} // namespace Global_Planning
