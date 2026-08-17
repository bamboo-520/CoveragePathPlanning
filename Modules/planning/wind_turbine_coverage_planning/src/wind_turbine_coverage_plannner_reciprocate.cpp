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

class WindTurbineCoveragePlannerReciprocate
{
public:
    explicit WindTurbineCoveragePlannerReciprocate(ros::NodeHandle& nh)
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

        // 叶片正面 / 反面往复式覆盖
        nh.param("coverage/blade_root_margin", blade_root_margin_, 0.45);
        nh.param("coverage/blade_tip_margin", blade_tip_margin_, 0.35);
        nh.param("coverage/blade_axis_step", blade_axis_step_, 0.25);

        // 沿风轮平面法向（正反面方向）的表面偏移
        if (!nh.getParam("coverage/blade_surface_offset", blade_surface_offset_))
        {
            // 兼容你之前 launch 里的旧参数名
            if (!nh.getParam("coverage/blade_line_offset", blade_surface_offset_))
            {
                blade_surface_offset_ = 0.9;
            }
        }

        // 从正/反面扫描线再往外退开的安全余量（同样沿正反面方向）
        nh.param("coverage/blade_transfer_side_offset", blade_transfer_side_offset_, 0.8);
        // 进出叶片时沿叶片轴向预留的缓冲
        nh.param("coverage/blade_transfer_axis_margin", blade_transfer_axis_margin_, 0.4);
        // 叶尖处绕到另一面的外扩量（沿叶片径向外侧）
        nh.param("coverage/blade_tip_turn_offset", blade_tip_turn_offset_, 1.2);

        // 风轮平面法向，默认认为风轮平面近似 x-z 平面，因此法向取 +y
        nh.param("coverage/rotor_face_nx", rotor_face_nx_, 0.0);
        nh.param("coverage/rotor_face_ny", rotor_face_ny_, 1.0);
        nh.param("coverage/rotor_face_nz", rotor_face_nz_, 0.0);

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
        drone_sub_ = nh.subscribe(drone_state_topic_, 10,
                                  &WindTurbineCoveragePlannerReciprocate::droneStateCb, this);

        timer_ = nh.createTimer(ros::Duration(publish_interval_),
                                &WindTurbineCoveragePlannerReciprocate::timerCb,
                                this,
                                false,
                                false);

        drone_ready_ = false;
        mission_started_ = false;
        mission_finished_ = false;
        first_send_ = true;
        current_idx_ = 0;

        metrics_running_ = false;
        metrics_last_pos_valid_ = false;
        metrics_distance_m_ = 0.0;
        metrics_avg_turn_deg_ = 0.0;
        metrics_smoothness_pct_ = 100.0;
        metrics_traj_points_.clear();

        rotor_face_normal_ = normalizeOrDefault(Eigen::Vector3d(rotor_face_nx_, rotor_face_ny_, rotor_face_nz_),
                                                Eigen::Vector3d(0.0, 1.0, 0.0));

        buildMission();
    }

    void start()
    {
        if (wps_.empty())
        {
            ROS_WARN("[coverage_planner_reciprocate] waypoint list is empty.");
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
        ROS_INFO("[coverage_planner_reciprocate] mission started, total waypoints = %zu", wps_.size());
    }

private:
    struct BladeInfo
    {
        std::string name;
        Eigen::Vector3d root;
        Eigen::Vector3d tip;
    };

    struct BladeGeometry
    {
        std::string name;
        Eigen::Vector3d root;
        Eigen::Vector3d tip;
        Eigen::Vector3d dir;
        Eigen::Vector3d face_normal;
        Eigen::Vector3d radial_outward;
        double axis_len;
        double s0;
        double s1;

        Eigen::Vector3d axis_root;
        Eigen::Vector3d axis_tip;

        Eigen::Vector3d front_root;
        Eigen::Vector3d front_tip;
        Eigen::Vector3d back_root;
        Eigen::Vector3d back_tip;

        Eigen::Vector3d front_root_safe;
        Eigen::Vector3d back_root_safe;
        Eigen::Vector3d front_tip_safe;
        Eigen::Vector3d back_tip_safe;
    };

    struct BladeVisitPlan
    {
        int blade_idx;
        bool start_on_front;
        std::string blade_name;
        Eigen::Vector3d entry_ref;
        Eigen::Vector3d finish_ref;
        std::vector<Eigen::Vector3d> waypoints;
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
    double blade_surface_offset_;
    double blade_axis_step_;

    double blade_transfer_side_offset_;
    double blade_transfer_axis_margin_;
    double blade_tip_turn_offset_;

    double rotor_face_nx_, rotor_face_ny_, rotor_face_nz_;
    Eigen::Vector3d rotor_face_normal_{0.0, 1.0, 0.0};

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
    static Eigen::Vector3d normalizeOrDefault(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback)
    {
        if (v.norm() > 1e-6)
            return v.normalized();
        return fallback.normalized();
    }

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

        ROS_INFO("[coverage_planner_reciprocate] send goal [%zu/%zu]: %.2f %.2f %.2f",
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
        ss << "[coverage_planner_reciprocate][TOTAL] Arrived. time=" << std::fixed << std::setprecision(2)
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
            ROS_WARN_THROTTLE(2.0, "[coverage_planner_reciprocate] waiting for drone state...");
            return;
        }

        if (current_idx_ >= wps_.size())
        {
            mission_finished_ = true;
            printMissionMetrics();
            ROS_INFO("[coverage_planner_reciprocate] all waypoints finished.");
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
            ++current_idx_;
            if (current_idx_ < wps_.size())
            {
                publishGoal(wps_[current_idx_]);
            }
            else
            {
                mission_finished_ = true;
                printMissionMetrics();
                ROS_INFO("[coverage_planner_reciprocate] all waypoints finished.");
            }
        }
    }

    void pushWp(const Eigen::Vector3d& p)
    {
        wps_.push_back(p);
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

    BladeGeometry computeBladeGeometry(const BladeInfo& blade) const
    {
        BladeGeometry g;
        g.name = blade.name;
        g.root = blade.root;
        g.tip = blade.tip;
        g.axis_len = (blade.tip - blade.root).norm();

        if (g.axis_len < 1.0)
        {
            g.dir = Eigen::Vector3d::UnitX();
            g.face_normal = rotor_face_normal_;
            g.radial_outward = Eigen::Vector3d::UnitZ();
            g.s0 = 0.1;
            g.s1 = 0.2;
            g.axis_root = blade.root;
            g.axis_tip = blade.tip;
            g.front_root = blade.root;
            g.front_tip = blade.tip;
            g.back_root = blade.root;
            g.back_tip = blade.tip;
            g.front_root_safe = blade.root;
            g.back_root_safe = blade.root;
            g.front_tip_safe = blade.tip;
            g.back_tip_safe = blade.tip;
            return g;
        }

        g.dir = (blade.tip - blade.root).normalized();
        g.face_normal = rotor_face_normal_;

        g.s0 = std::min(std::max(0.10, blade_root_margin_), g.axis_len * 0.25);
        g.s1 = std::max(g.s0 + 0.6, g.axis_len - blade_tip_margin_);

        g.axis_root = blade.root + g.dir * g.s0;
        g.axis_tip = blade.root + g.dir * g.s1;

        const Eigen::Vector3d hub(hub_x_, hub_y_, hub_z_);
        Eigen::Vector3d radial_vec = g.axis_tip - hub;
        radial_vec = radial_vec - radial_vec.dot(g.face_normal) * g.face_normal;
        g.radial_outward = normalizeOrDefault(radial_vec, g.dir);

        const Eigen::Vector3d face_offset = blade_surface_offset_ * g.face_normal;
        const Eigen::Vector3d safe_face_offset = (blade_surface_offset_ + blade_transfer_side_offset_) * g.face_normal;
        const Eigen::Vector3d root_axis_buffer = blade_transfer_axis_margin_ * g.dir;
        const Eigen::Vector3d tip_radial_buffer = blade_tip_turn_offset_ * g.radial_outward;

        g.front_root = g.axis_root + face_offset;
        g.front_tip = g.axis_tip + face_offset;
        g.back_root = g.axis_root - face_offset;
        g.back_tip = g.axis_tip - face_offset;

        g.front_root_safe = g.axis_root + safe_face_offset - root_axis_buffer;
        g.back_root_safe = g.axis_root - safe_face_offset - root_axis_buffer;
        g.front_tip_safe = g.axis_tip + safe_face_offset + tip_radial_buffer;
        g.back_tip_safe = g.axis_tip - safe_face_offset + tip_radial_buffer;

        return g;
    }

    static void appendLineSegmentToVector(const Eigen::Vector3d& start,
                                          const Eigen::Vector3d& end,
                                          int min_steps,
                                          bool include_first,
                                          std::vector<Eigen::Vector3d>& out)
    {
        const double len = (end - start).norm();
        const int steps = std::max(min_steps, static_cast<int>(std::ceil(len / 0.25)));

        for (int k = 0; k <= steps; ++k)
        {
            if (!include_first && k == 0)
                continue;

            const double ratio = static_cast<double>(k) / static_cast<double>(steps);
            out.push_back(start + (end - start) * ratio);
        }
    }

    void appendBladeMainLineToVector(const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& end,
                                     bool include_first,
                                     std::vector<Eigen::Vector3d>& out) const
    {
        const double len = (end - start).norm();
        const int steps = std::max(1,
                                   static_cast<int>(std::ceil(len / std::max(0.05, blade_axis_step_))));

        for (int k = 0; k <= steps; ++k)
        {
            if (!include_first && k == 0)
                continue;

            const double ratio = static_cast<double>(k) / static_cast<double>(steps);
            out.push_back(start + (end - start) * ratio);
        }
    }

    static double wrapAngleNear(double angle, double ref)
    {
        const double two_pi = 2.0 * M_PI;
        while (angle - ref > M_PI)
            angle -= two_pi;
        while (angle - ref < -M_PI)
            angle += two_pi;
        return angle;
    }

    void appendTipTransitionArcToVector(const BladeGeometry& g,
                                        bool front_to_back,
                                        bool include_first,
                                        std::vector<Eigen::Vector3d>& out) const
    {
        const double a = std::max(1e-6, blade_surface_offset_);
        const double b = blade_tip_turn_offset_;

        if (b <= 1e-6)
        {
            appendLineSegmentToVector(front_to_back ? g.front_tip : g.back_tip,
                                      front_to_back ? g.back_tip : g.front_tip,
                                      6,
                                      include_first,
                                      out);
            return;
        }

        const double center_shift = (b * b - a * a) / (2.0 * b);
        const Eigen::Vector3d circle_center = g.axis_tip + center_shift * g.radial_outward;

        const Eigen::Vector3d start_pt = front_to_back ? g.front_tip : g.back_tip;
        const Eigen::Vector3d end_pt   = front_to_back ? g.back_tip  : g.front_tip;
        const Eigen::Vector3d mid_pt   = g.axis_tip + b * g.radial_outward;

        auto angle_on_plane = [&](const Eigen::Vector3d& p) -> double
        {
            const Eigen::Vector3d v = p - circle_center;
            return std::atan2(v.dot(g.radial_outward), v.dot(g.face_normal));
        };

        const double start_angle = angle_on_plane(start_pt);
        const double mid_angle_raw = angle_on_plane(mid_pt);
        const double end_angle_raw = angle_on_plane(end_pt);

        double mid_angle = wrapAngleNear(mid_angle_raw, start_angle);
        double end_angle = wrapAngleNear(end_angle_raw, start_angle);

        if (!((mid_angle > start_angle && mid_angle < end_angle) ||
              (mid_angle < start_angle && mid_angle > end_angle)))
        {
            const double two_pi = 2.0 * M_PI;
            double candidate1 = end_angle + two_pi;
            double candidate2 = end_angle - two_pi;

            if ((mid_angle > start_angle && mid_angle < candidate1) ||
                (mid_angle < start_angle && mid_angle > candidate1))
            {
                end_angle = candidate1;
            }
            else
            {
                end_angle = candidate2;
            }
        }

        const double radius = (start_pt - circle_center).norm();
        const double arc_len = std::fabs(end_angle - start_angle) * radius;
        const int steps = std::max(8, static_cast<int>(std::ceil(arc_len / std::max(0.05, blade_axis_step_))));

        for (int k = 0; k <= steps; ++k)
        {
            if (!include_first && k == 0)
                continue;

            const double ratio = static_cast<double>(k) / static_cast<double>(steps);
            const double theta = start_angle + (end_angle - start_angle) * ratio;
            const Eigen::Vector3d p = circle_center
                                    + radius * std::cos(theta) * g.face_normal
                                    + radius * std::sin(theta) * g.radial_outward;
            out.push_back(p);
        }
    }

    BladeVisitPlan makeBladeVisitPlan(int blade_idx, const BladeInfo& blade, bool start_on_front) const
    {
        const BladeGeometry g = computeBladeGeometry(blade);

        BladeVisitPlan plan;
        plan.blade_idx = blade_idx;
        plan.start_on_front = start_on_front;
        plan.blade_name = blade.name;

        if (start_on_front)
        {
            plan.entry_ref = g.front_root_safe;
            plan.finish_ref = g.back_root_safe;

            plan.waypoints.push_back(g.front_root_safe);
            appendBladeMainLineToVector(g.front_root, g.front_tip, true, plan.waypoints);
            appendTipTransitionArcToVector(g, true, false, plan.waypoints);
            appendBladeMainLineToVector(g.back_tip, g.back_root, false, plan.waypoints);
            plan.waypoints.push_back(g.back_root_safe);
        }
        else
        {
            plan.entry_ref = g.back_root_safe;
            plan.finish_ref = g.front_root_safe;

            plan.waypoints.push_back(g.back_root_safe);
            appendBladeMainLineToVector(g.back_root, g.back_tip, true, plan.waypoints);
            appendTipTransitionArcToVector(g, false, false, plan.waypoints);
            appendBladeMainLineToVector(g.front_tip, g.front_root, false, plan.waypoints);
            plan.waypoints.push_back(g.front_root_safe);
        }

        return plan;
    }

    void appendLineSegment(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& end,
                           int min_steps,
                           bool include_first)
    {
        const double len = (end - start).norm();
        const int steps = std::max(min_steps,
                                   static_cast<int>(std::ceil(len / std::max(0.05, blade_axis_step_))));

        for (int k = 0; k <= steps; ++k)
        {
            if (!include_first && k == 0)
                continue;

            const double ratio = static_cast<double>(k) / static_cast<double>(steps);
            pushWp(start + (end - start) * ratio);
        }
    }

    void appendBladeVisitPlan(const BladeVisitPlan& plan)
    {
        if (plan.waypoints.empty())
            return;

        // 使用当前 blade_axis_step_ 重采样，避免上面静态函数里写死步长
        pushWp(plan.waypoints.front());
        for (size_t i = 1; i < plan.waypoints.size(); ++i)
        {
            appendLineSegment(plan.waypoints[i - 1], plan.waypoints[i], 1, false);
        }
    }

    void buildApproach()
    {
        pushWp(Eigen::Vector3d(base_x_ + approach_radius_,
                               base_y_,
                               base_z_ + approach_height_));
    }

    void buildTowerSpiral()
    {
        const double scan_r = tower_radius_ + tower_offset_distance_;
        const double z0 = base_z_ + tower_z_start_;
        const double z1 = base_z_ + std::min(tower_z_end_, tower_height_);

        if (z1 <= z0)
            return;

        const double circumference = 2.0 * M_PI * scan_r;
        const int n_per_circle = std::max(8, static_cast<int>(std::ceil(circumference / tower_wp_arc_step_)));

        const double total_h = z1 - z0;
        const double total_turns = total_h / tower_spiral_pitch_;
        const int total_steps = std::max(n_per_circle,
                                         static_cast<int>(std::ceil(total_turns * n_per_circle)));

        ROS_INFO("[coverage_planner_reciprocate] tower scan_r = %.2f, total_turns = %.2f",
                 scan_r, total_turns);

        for (int k = 0; k <= total_steps; ++k)
        {
            const double ratio = static_cast<double>(k) / static_cast<double>(total_steps);
            const double theta = 2.0 * M_PI * total_turns * ratio;
            const double z = z0 + total_h * ratio;
            const double x = base_x_ + scan_r * std::cos(theta);
            const double y = base_y_ + scan_r * std::sin(theta);
            pushWp(Eigen::Vector3d(x, y, z));
        }
    }

    void buildFinalHover()
    {
        pushWp(Eigen::Vector3d(base_x_ + final_hover_radius_,
                               base_y_,
                               base_z_ + final_hover_height_));
    }

    void buildMission()
    {
        wps_.clear();

        const std::vector<BladeInfo> blades = {
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

        Eigen::Vector3d current_ref = getTowerEndPoint();
        std::vector<int> remaining = {0, 1, 2};

        while (!remaining.empty())
        {
            double best_dist = std::numeric_limits<double>::max();
            BladeVisitPlan best_plan;
            int best_pos = -1;

            for (int i = 0; i < static_cast<int>(remaining.size()); ++i)
            {
                const int idx = remaining[i];

                const BladeVisitPlan plan_front = makeBladeVisitPlan(idx, blades[idx], true);
                const BladeVisitPlan plan_back  = makeBladeVisitPlan(idx, blades[idx], false);

                const double d_front = (plan_front.entry_ref - current_ref).norm();
                if (d_front < best_dist)
                {
                    best_dist = d_front;
                    best_plan = plan_front;
                    best_pos = i;
                }

                const double d_back = (plan_back.entry_ref - current_ref).norm();
                if (d_back < best_dist)
                {
                    best_dist = d_back;
                    best_plan = plan_back;
                    best_pos = i;
                }
            }

            if (best_pos < 0)
                break;

            ROS_INFO("[coverage_planner_reciprocate] choose %s, start_on_%s, transition_dist=%.2f",
                     best_plan.blade_name.c_str(),
                     best_plan.start_on_front ? "front" : "back",
                     best_dist);

            appendBladeVisitPlan(best_plan);
            current_ref = best_plan.finish_ref;
            remaining.erase(remaining.begin() + best_pos);
        }

        buildFinalHover();
        ROS_INFO("[coverage_planner_reciprocate] mission waypoints generated = %zu", wps_.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_turbine_coverage_plannner_reciprocate");
    ros::NodeHandle nh("~");

    WindTurbineCoveragePlannerReciprocate node(nh);

    double delay = 2.0;
    nh.param("coverage/auto_start_delay", delay, 2.0);
    ros::Duration(delay).sleep();

    node.start();
    ros::spin();
    return 0;
}
