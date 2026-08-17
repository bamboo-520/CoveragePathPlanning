#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <prometheus_msgs/DroneState.h>
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>

class WindTurbineCoveragePlanner
{
public:
    WindTurbineCoveragePlanner(ros::NodeHandle& nh)
    {
        nh.param("coverage/frame_id", frame_id_, std::string("world"));
        nh.param("coverage/goal_topic", goal_topic_, std::string("/prometheus/planning/goal"));
        nh.param("coverage/drone_state_topic", drone_state_topic_, std::string("/prometheus/drone_state"));

        nh.param("coverage/base_x", base_x_, 0.0);
        nh.param("coverage/base_y", base_y_, 0.0);
        nh.param("coverage/base_z", base_z_, 0.0);

        nh.param("coverage/tower_radius", tower_radius_, 0.60);
        nh.param("coverage/tower_height", tower_height_, 13.6);

        nh.param("coverage/hub_x", hub_x_, 0.0);
        nh.param("coverage/hub_y", hub_y_, -1.1365);
        nh.param("coverage/hub_z", hub_z_, 13.168);

        // 塔筒螺旋
        nh.param("coverage/tower_offset_distance", tower_offset_distance_, 0.8);
        nh.param("coverage/tower_spiral_pitch", tower_spiral_pitch_, 3.0);
        nh.param("coverage/tower_wp_arc_step", tower_wp_arc_step_, 0.4);
        nh.param("coverage/tower_z_start", tower_z_start_, 1.5);
        nh.param("coverage/tower_z_end", tower_z_end_, 11.5);

        // 叶片螺旋
        nh.param("coverage/blade_root_margin", blade_root_margin_, 0.45);
        nh.param("coverage/blade_tip_margin", blade_tip_margin_, 0.35);
        nh.param("coverage/blade_helix_radius", blade_helix_radius_, 1.35);
        nh.param("coverage/blade_helix_pitch", blade_helix_pitch_, 1.0);
        nh.param("coverage/blade_axis_step", blade_axis_step_, 0.25);

        // 叶片之间的安全转移：只保留 pre_entry_safe
        nh.param("coverage/blade_transfer_side_offset", blade_transfer_side_offset_, 1.8);
        nh.param("coverage/blade_transfer_axis_margin", blade_transfer_axis_margin_, 0.4);

        // 叶片 root/tip
        nh.param("coverage/blade1_root_x", blade1_root_x_, 0.3943);
        nh.param("coverage/blade1_root_y", blade1_root_y_, -0.9067);
        nh.param("coverage/blade1_root_z", blade1_root_z_, 13.1903);
        nh.param("coverage/blade1_tip_x",  blade1_tip_x_,  5.8692);
        nh.param("coverage/blade1_tip_y",  blade1_tip_y_, -1.0068);
        nh.param("coverage/blade1_tip_z",  blade1_tip_z_,  9.8347);

        nh.param("coverage/blade2_root_x", blade2_root_x_, -0.1730);
        nh.param("coverage/blade2_root_y", blade2_root_y_, -0.9067);
        nh.param("coverage/blade2_root_z", blade2_root_z_, 12.8054);
        nh.param("coverage/blade2_tip_x",  blade2_tip_x_, -5.8158);
        nh.param("coverage/blade2_tip_y",  blade2_tip_y_, -1.0068);
        nh.param("coverage/blade2_tip_z",  blade2_tip_z_,  9.7422);

        nh.param("coverage/blade3_root_x", blade3_root_x_, -0.2218);
        nh.param("coverage/blade3_root_y", blade3_root_y_, -0.9068);
        nh.param("coverage/blade3_root_z", blade3_root_z_, 13.4852);
        nh.param("coverage/blade3_tip_x",  blade3_tip_x_, -0.0534);
        nh.param("coverage/blade3_tip_y",  blade3_tip_y_, -1.0068);
        nh.param("coverage/blade3_tip_z",  blade3_tip_z_, 19.9080);

        // 起终点
        nh.param("coverage/approach_radius", approach_radius_, 3.2);
        nh.param("coverage/approach_height", approach_height_, 1.8);
        nh.param("coverage/final_hover_radius", final_hover_radius_, 3.5);
        nh.param("coverage/final_hover_height", final_hover_height_, 3.0);

        // 执行
        nh.param("coverage/reach_dist", reach_dist_, 0.45);
        nh.param("coverage/publish_interval", publish_interval_, 1.0);
        nh.param("coverage/auto_start_delay", auto_start_delay_, 2.0);

        // 覆盖任务总统计（基于 drone_state）
        nh.param("coverage/metrics_min_step", metrics_min_step_, 0.02);
        nh.param("coverage/small_turn_thresh_deg", small_turn_thresh_deg_, 5.0);
        nh.param("coverage/large_turn_thresh_deg", large_turn_thresh_deg_, 15.0);

        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
        drone_sub_ = nh.subscribe(drone_state_topic_, 10, &WindTurbineCoveragePlanner::droneStateCb, this);

        timer_ = nh.createTimer(ros::Duration(publish_interval_),
                                &WindTurbineCoveragePlanner::timerCb,
                                this,
                                false,
                                false);

        drone_ready_ = false;
        mission_started_ = false;
        mission_finished_ = false;
        first_send_ = true;
        current_idx_ = 0;

        // 统计初始化
        metrics_running_ = false;
        metrics_last_pos_valid_ = false;
        metrics_distance_m_ = 0.0;
        metrics_avg_turn_deg_ = 0.0;
        metrics_smoothness_pct_ = 100.0;
        metrics_traj_points_.clear();

        buildMission();
    }

    void start()
    {
        if (wps_.empty())
        {
            ROS_WARN("[coverage_planner] waypoint list is empty.");
            return;
        }

        metrics_running_ = true;
        metrics_last_pos_valid_ = false;
        metrics_distance_m_ = 0.0;
        metrics_avg_turn_deg_ = 0.0;
        metrics_smoothness_pct_ = 100.0;
        metrics_traj_points_.clear();
        metrics_start_time_ = ros::Time::now();

        timer_.start();
        mission_started_ = true;
        ROS_INFO("[coverage_planner] mission started, total waypoints = %zu", wps_.size());
    }

private:
    struct BladeInfo
    {
        std::string name;
        Eigen::Vector3d root;
        Eigen::Vector3d tip;
    };

    struct BladeTransitionInfo
    {
        Eigen::Vector3d pre_entry_safe;
        Eigen::Vector3d entry;
        Eigen::Vector3d helix_end_point;
        Eigen::Vector3d dir;
        Eigen::Vector3d e1;
        Eigen::Vector3d e2;
        double axis_len;
        double s0;
        double s1;
    };

    ros::Publisher goal_pub_;
    ros::Subscriber drone_sub_;
    ros::Timer timer_;

    std::string frame_id_;
    std::string goal_topic_;
    std::string drone_state_topic_;

    double base_x_, base_y_, base_z_;
    double tower_radius_, tower_height_;
    double hub_x_, hub_y_, hub_z_;

    double tower_offset_distance_;
    double tower_spiral_pitch_;
    double tower_wp_arc_step_;
    double tower_z_start_;
    double tower_z_end_;

    double blade_root_margin_;
    double blade_tip_margin_;
    double blade_helix_radius_;
    double blade_helix_pitch_;
    double blade_axis_step_;

    double blade_transfer_side_offset_;
    double blade_transfer_axis_margin_;

    double blade1_root_x_, blade1_root_y_, blade1_root_z_;
    double blade1_tip_x_,  blade1_tip_y_,  blade1_tip_z_;
    double blade2_root_x_, blade2_root_y_, blade2_root_z_;
    double blade2_tip_x_,  blade2_tip_y_,  blade2_tip_z_;
    double blade3_root_x_, blade3_root_y_, blade3_root_z_;
    double blade3_tip_x_,  blade3_tip_y_,  blade3_tip_z_;

    double approach_radius_;
    double approach_height_;
    double final_hover_radius_;
    double final_hover_height_;

    double reach_dist_;
    double publish_interval_;
    double auto_start_delay_;

    // 覆盖任务总统计
    bool metrics_running_;
    bool metrics_last_pos_valid_;
    ros::Time metrics_start_time_;
    Eigen::Vector3d metrics_last_pos_{0, 0, 0};
    double metrics_distance_m_;
    double metrics_min_step_;
    double metrics_avg_turn_deg_;
    double metrics_smoothness_pct_;
    double small_turn_thresh_deg_;
    double large_turn_thresh_deg_;
    std::vector<Eigen::Vector3d> metrics_traj_points_;

    bool drone_ready_;
    bool mission_started_;
    bool mission_finished_;
    bool first_send_;
    size_t current_idx_;

    Eigen::Vector3d drone_pos_{0, 0, 0};
    std::vector<Eigen::Vector3d> wps_;

private:
    void droneStateCb(const prometheus_msgs::DroneStateConstPtr& msg)
    {
        drone_pos_ << msg->position[0], msg->position[1], msg->position[2];
        drone_ready_ = true;

        if (metrics_running_)
        {
            if (!metrics_last_pos_valid_)
            {
                metrics_last_pos_ = drone_pos_;
                metrics_last_pos_valid_ = true;
                metrics_traj_points_.push_back(drone_pos_);
            }
            else
            {
                const double step = (drone_pos_ - metrics_last_pos_).norm();
                if (step >= metrics_min_step_)
                {
                    metrics_distance_m_ += step;
                    metrics_last_pos_ = drone_pos_;
                    metrics_traj_points_.push_back(drone_pos_);
                }
            }
        }
    }

    void publishGoal(const Eigen::Vector3d& p)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = frame_id_;
        goal.pose.position.x = p.x();
        goal.pose.position.y = p.y();
        goal.pose.position.z = p.z();
        goal.pose.orientation.w = 1.0;
        goal_pub_.publish(goal);

        ROS_INFO("[coverage_planner] send goal [%zu/%zu]: %.2f %.2f %.2f",
                 current_idx_ + 1, wps_.size(), p.x(), p.y(), p.z());
    }

    void computeTurnMetrics(double& avg_turn_deg, double& smoothness_pct) const
    {
        avg_turn_deg = 0.0;
        smoothness_pct = 100.0;

        if (metrics_traj_points_.size() < 3)
            return;

        int turn_count = 0;
        int sharp_turn_count = 0;
        double turn_sum_deg = 0.0;

        for (size_t i = 1; i + 1 < metrics_traj_points_.size(); ++i)
        {
            const Eigen::Vector3d v1 = metrics_traj_points_[i] - metrics_traj_points_[i - 1];
            const Eigen::Vector3d v2 = metrics_traj_points_[i + 1] - metrics_traj_points_[i];

            const double n1 = v1.norm();
            const double n2 = v2.norm();

            if (n1 < metrics_min_step_ || n2 < metrics_min_step_)
                continue;

            double cos_theta = v1.dot(v2) / (n1 * n2);
            cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            const double angle_deg = std::acos(cos_theta) * 180.0 / M_PI;

            if (angle_deg >= small_turn_thresh_deg_)
            {
                ++turn_count;
                turn_sum_deg += angle_deg;

                if (angle_deg >= large_turn_thresh_deg_)
                    ++sharp_turn_count;
            }
        }

        if (turn_count > 0)
        {
            avg_turn_deg = turn_sum_deg / static_cast<double>(turn_count);
            smoothness_pct = 100.0 - 100.0 * static_cast<double>(sharp_turn_count) / static_cast<double>(turn_count);
        }
    }

    void printMissionMetrics()
    {
        if (!metrics_running_)
            return;

        computeTurnMetrics(metrics_avg_turn_deg_, metrics_smoothness_pct_);
        metrics_running_ = false;

        const double t = (ros::Time::now() - metrics_start_time_).toSec();

        std::ostringstream ss;
        ss << "[coverage_planner][TOTAL] Arrived. time=" << std::fixed << std::setprecision(2)
           << t
           << " s, distance=" << std::fixed << std::setprecision(2)
           << metrics_distance_m_
           << " m, avg_turn=" << std::fixed << std::setprecision(2)
           << metrics_avg_turn_deg_
           << " deg, smoothness=" << std::fixed << std::setprecision(2)
           << metrics_smoothness_pct_ << " %";

        ROS_INFO("%s", ss.str().c_str());
    }

    void timerCb(const ros::TimerEvent&)
    {
        if (!mission_started_ || mission_finished_ || wps_.empty())
            return;

        if (!drone_ready_)
        {
            ROS_WARN_THROTTLE(2.0, "[coverage_planner] waiting for drone state...");
            return;
        }

        if (current_idx_ >= wps_.size())
        {
            mission_finished_ = true;
            printMissionMetrics();
            ROS_INFO("[coverage_planner] all waypoints finished.");
            return;
        }

        const Eigen::Vector3d target = wps_[current_idx_];
        const double dist = (drone_pos_ - target).norm();

        if (first_send_)
        {
            publishGoal(target);
            first_send_ = false;
            return;
        }

        if (dist < reach_dist_)
        {
            current_idx_++;
            if (current_idx_ < wps_.size())
            {
                publishGoal(wps_[current_idx_]);
            }
            else
            {
                mission_finished_ = true;
                printMissionMetrics();
                ROS_INFO("[coverage_planner] all waypoints finished.");
            }
        }
    }

    void pushWp(const Eigen::Vector3d& p)
    {
        wps_.push_back(p);
    }

    Eigen::Vector3d rotorSafeOffset(double side_offset) const
    {
        return Eigen::Vector3d(0.0, -side_offset, 0.0);
    }

    void makeOrthonormalBasis(const Eigen::Vector3d& axis,
                              Eigen::Vector3d& e1,
                              Eigen::Vector3d& e2) const
    {
        Eigen::Vector3d ref(0.0, 1.0, 0.0);
        if (std::fabs(axis.dot(ref)) > 0.95)
            ref = Eigen::Vector3d(1.0, 0.0, 0.0);

        e1 = axis.cross(ref).normalized();
        e2 = axis.cross(e1).normalized();
    }

    Eigen::Vector3d getTowerEndPoint() const
    {
        const double scan_r = tower_radius_ + tower_offset_distance_;
        const double z0 = base_z_ + tower_z_start_;
        const double z1 = base_z_ + std::min(tower_z_end_, tower_height_);

        if (z1 <= z0)
            return Eigen::Vector3d(base_x_ + scan_r, base_y_, z0);

        const double total_h = z1 - z0;
        const double total_turns = total_h / tower_spiral_pitch_;
        const double theta_end = 2.0 * M_PI * total_turns;

        return Eigen::Vector3d(base_x_ + scan_r * std::cos(theta_end),
                               base_y_ + scan_r * std::sin(theta_end),
                               z1);
    }

    BladeTransitionInfo computeBladeTransitionInfo(const Eigen::Vector3d& root,
                                                   const Eigen::Vector3d& tip) const
    {
        BladeTransitionInfo info;
        info.axis_len = (tip - root).norm();

        if (info.axis_len < 1.0)
        {
            info.dir = Eigen::Vector3d::UnitX();
            info.e1  = Eigen::Vector3d::UnitY();
            info.e2  = Eigen::Vector3d::UnitZ();
            info.s0 = 0.1;
            info.s1 = 0.2;
            info.pre_entry_safe = root;
            info.entry = root;
            info.helix_end_point = tip;
            return info;
        }

        info.dir = (tip - root).normalized();
        makeOrthonormalBasis(info.dir, info.e1, info.e2);

        info.s0 = std::min(std::max(0.10, blade_root_margin_), info.axis_len * 0.25);
        info.s1 = std::max(info.s0 + 0.6, info.axis_len - blade_tip_margin_);

        info.entry =
            root + info.dir * info.s0 + blade_helix_radius_ * info.e1;

        info.pre_entry_safe =
            root
          + info.dir * std::max(0.05, info.s0 - blade_transfer_axis_margin_)
          + blade_helix_radius_ * info.e1
          + rotorSafeOffset(blade_transfer_side_offset_);

        const double work_len = std::max(0.0, info.s1 - info.s0);
        const double turns = work_len / std::max(0.2, blade_helix_pitch_);
        const double theta_end = 2.0 * M_PI * turns;

        const Eigen::Vector3d center_end = root + info.dir * info.s1;
        const Eigen::Vector3d offset_end =
            blade_helix_radius_ * (std::cos(theta_end) * info.e1 + std::sin(theta_end) * info.e2);

        info.helix_end_point = center_end + offset_end;

        return info;
    }

    void buildApproach()
    {
        Eigen::Vector3d p(base_x_ + approach_radius_,
                          base_y_,
                          base_z_ + approach_height_);
        pushWp(p);
    }

    void buildTowerSpiral()
    {
        const double scan_r = tower_radius_ + tower_offset_distance_;
        const double z0 = base_z_ + tower_z_start_;
        const double z1 = base_z_ + std::min(tower_z_end_, tower_height_);

        if (z1 <= z0)
            return;

        const double circumference = 2.0 * M_PI * scan_r;
        const int n_per_circle = std::max(8, (int)std::ceil(circumference / tower_wp_arc_step_));

        const double total_h = z1 - z0;
        const double total_turns = total_h / tower_spiral_pitch_;
        const int total_steps = std::max(n_per_circle, (int)std::ceil(total_turns * n_per_circle));

        ROS_INFO("[coverage_planner] tower scan_r = %.2f, total_turns = %.2f",
                 scan_r, total_turns);

        for (int k = 0; k <= total_steps; ++k)
        {
            const double ratio = (double)k / (double)total_steps;
            const double theta = 2.0 * M_PI * total_turns * ratio;
            const double z = z0 + total_h * ratio;
            const double x = base_x_ + scan_r * std::cos(theta);
            const double y = base_y_ + scan_r * std::sin(theta);
            pushWp(Eigen::Vector3d(x, y, z));
        }
    }

    Eigen::Vector3d appendBladeHelixFromEndpoints(const BladeInfo& blade)
    {
        const Eigen::Vector3d& root = blade.root;
        const Eigen::Vector3d& tip  = blade.tip;

        BladeTransitionInfo info = computeBladeTransitionInfo(root, tip);
        if (info.axis_len < 1.0 || info.s1 <= info.s0)
        {
            ROS_WARN("[coverage_planner] invalid blade endpoints for %s", blade.name.c_str());
            return root;
        }

        pushWp(info.pre_entry_safe);
        pushWp(info.entry);

        const double work_len = info.s1 - info.s0;
        const double turns = work_len / std::max(0.2, blade_helix_pitch_);
        const int steps = std::max(8, (int)std::ceil(work_len / std::max(0.05, blade_axis_step_)));

        ROS_INFO("[coverage_planner] append %s helix, axis_len=%.2f", blade.name.c_str(), info.axis_len);

        Eigen::Vector3d last_pt = info.entry;

        for (int k = 1; k <= steps; ++k)
        {
            const double ratio = (double)k / (double)steps;
            const double s = info.s0 + work_len * ratio;
            const double theta = 2.0 * M_PI * turns * ratio;

            const Eigen::Vector3d center = root + info.dir * s;
            const Eigen::Vector3d offset =
                blade_helix_radius_ * (std::cos(theta) * info.e1 + std::sin(theta) * info.e2);

            last_pt = center + offset;
            pushWp(last_pt);
        }

        return last_pt;
    }

    void buildFinalHover()
    {
        Eigen::Vector3d p(base_x_ + final_hover_radius_,
                          base_y_,
                          base_z_ + final_hover_height_);
        pushWp(p);
    }

    void buildMission()
    {
        wps_.clear();

        std::vector<BladeInfo> blades = {
            {"blade_1",
             Eigen::Vector3d(blade1_root_x_, blade1_root_y_, blade1_root_z_),
             Eigen::Vector3d(blade1_tip_x_,  blade1_tip_y_,  blade1_tip_z_)},

            {"blade_2",
             Eigen::Vector3d(blade2_root_x_, blade2_root_y_, blade2_root_z_),
             Eigen::Vector3d(blade2_tip_x_,  blade2_tip_y_,  blade2_tip_z_)},

            {"blade_3",
             Eigen::Vector3d(blade3_root_x_, blade3_root_y_, blade3_root_z_),
             Eigen::Vector3d(blade3_tip_x_,  blade3_tip_y_,  blade3_tip_z_)}
        };

        buildApproach();
        buildTowerSpiral();

        const Eigen::Vector3d tower_end = getTowerEndPoint();

        std::vector<int> remaining = {0, 1, 2};
        std::vector<int> order;
        Eigen::Vector3d current_ref = tower_end;

        while (!remaining.empty())
        {
            double best_dist = std::numeric_limits<double>::max();
            int best_pos = -1;

            for (int i = 0; i < (int)remaining.size(); ++i)
            {
                const int idx = remaining[i];
                const BladeTransitionInfo info = computeBladeTransitionInfo(blades[idx].root, blades[idx].tip);
                const double d = (info.pre_entry_safe - current_ref).norm();

                if (d < best_dist)
                {
                    best_dist = d;
                    best_pos = i;
                }
            }

            const int chosen_idx = remaining[best_pos];
            order.push_back(chosen_idx);

            const BladeTransitionInfo chosen_info =
                computeBladeTransitionInfo(blades[chosen_idx].root, blades[chosen_idx].tip);
            current_ref = chosen_info.helix_end_point;

            remaining.erase(remaining.begin() + best_pos);
        }

        ROS_INFO("[coverage_planner] blade order after tower: %s -> %s -> %s",
                 blades[order[0]].name.c_str(),
                 blades[order[1]].name.c_str(),
                 blades[order[2]].name.c_str());

        appendBladeHelixFromEndpoints(blades[order[0]]);
        appendBladeHelixFromEndpoints(blades[order[1]]);
        appendBladeHelixFromEndpoints(blades[order[2]]);

        buildFinalHover();

        ROS_INFO("[coverage_planner] mission waypoints generated = %zu", wps_.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_turbine_coverage_planner");
    ros::NodeHandle nh("~");

    WindTurbineCoveragePlanner node(nh);

    double delay = 2.0;
    nh.param("coverage/auto_start_delay", delay, 2.0);
    ros::Duration(delay).sleep();

    node.start();
    ros::spin();
    return 0;
}