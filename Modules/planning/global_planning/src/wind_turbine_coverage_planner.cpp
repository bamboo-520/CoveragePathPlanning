#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <prometheus_msgs/DroneState.h>
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

class WindTurbineCoveragePlanner
{
public:
    WindTurbineCoveragePlanner(ros::NodeHandle& nh)
    {
        nh.param("coverage/frame_id", frame_id_, std::string("world"));
        nh.param("coverage/goal_topic", goal_topic_, std::string("/prometheus/planning/goal"));
        nh.param("coverage/drone_state_topic", drone_state_topic_, std::string("/prometheus/drone_state"));

        // 风机底座中心
        nh.param("coverage/base_x", base_x_, 0.0);
        nh.param("coverage/base_y", base_y_, 0.0);
        nh.param("coverage/base_z", base_z_, 0.0);

        // 塔筒
        nh.param("coverage/tower_radius", tower_radius_, 0.60);
        nh.param("coverage/tower_height", tower_height_, 13.6);

        // 轮毂中心
        nh.param("coverage/hub_x", hub_x_, 0.0);
        nh.param("coverage/hub_y", hub_y_, 0.0);
        nh.param("coverage/hub_z", hub_z_, 13.168);

        // 塔筒螺旋
        nh.param("coverage/tower_offset_distance", tower_offset_distance_, 2.0);
        nh.param("coverage/tower_spiral_pitch", tower_spiral_pitch_, 1.0);
        nh.param("coverage/tower_wp_arc_step", tower_wp_arc_step_, 0.4);
        nh.param("coverage/tower_z_start", tower_z_start_, 1.5);
        nh.param("coverage/tower_z_end", tower_z_end_, 11.5);

        // 机舱环绕：只 1 圈
        nh.param("coverage/nacelle_orbit_radius", nacelle_orbit_radius_, 2.4);
        nh.param("coverage/nacelle_orbit_turns", nacelle_orbit_turns_, 1.0);
        nh.param("coverage/nacelle_orbit_arc_step", nacelle_orbit_arc_step_, 0.35);
        nh.param("coverage/nacelle_orbit_z", nacelle_orbit_z_, 13.2);

        // 叶片螺旋
        nh.param("coverage/blade_root_radius", blade_root_radius_, 1.0);
        nh.param("coverage/blade_length", blade_length_, 6.4);
        nh.param("coverage/blade_tip_margin", blade_tip_margin_, 0.3);
        nh.param("coverage/blade_helix_radius", blade_helix_radius_, 1.0);
        nh.param("coverage/blade_helix_pitch", blade_helix_pitch_, 1.0);
        nh.param("coverage/blade_axis_step", blade_axis_step_, 0.30);

        // 起点/终点
        nh.param("coverage/approach_radius", approach_radius_, 3.0);
        nh.param("coverage/approach_height", approach_height_, 1.8);
        nh.param("coverage/final_hover_radius", final_hover_radius_, 3.2);
        nh.param("coverage/final_hover_height", final_hover_height_, 3.0);

        // 执行
        nh.param("coverage/reach_dist", reach_dist_, 0.45);
        nh.param("coverage/publish_interval", publish_interval_, 1.0);

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
        current_idx_ = 0;
        first_send_ = true;

        buildMission();
    }

    void start()
    {
        if (wps_.empty())
        {
            ROS_WARN("[coverage_planner] waypoint list is empty.");
            return;
        }

        timer_.start();
        mission_started_ = true;
        ROS_INFO("[coverage_planner] mission started, total waypoints = %zu", wps_.size());
    }

private:
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

    double nacelle_orbit_radius_;
    double nacelle_orbit_turns_;
    double nacelle_orbit_arc_step_;
    double nacelle_orbit_z_;

    double blade_root_radius_;
    double blade_length_;
    double blade_tip_margin_;
    double blade_helix_radius_;
    double blade_helix_pitch_;
    double blade_axis_step_;

    double approach_radius_;
    double approach_height_;
    double final_hover_radius_;
    double final_hover_height_;

    double reach_dist_;
    double publish_interval_;

    bool drone_ready_;
    bool mission_started_;
    bool mission_finished_;
    bool first_send_;
    size_t current_idx_;

    Eigen::Vector3d drone_pos_{0, 0, 0};
    std::vector<Eigen::Vector3d> wps_;

private:
    static double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    void droneStateCb(const prometheus_msgs::DroneStateConstPtr& msg)
    {
        drone_pos_ << msg->position[0], msg->position[1], msg->position[2];
        drone_ready_ = true;
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

    void timerCb(const ros::TimerEvent&)
    {
        if (!mission_started_ || mission_finished_ || wps_.empty())
        {
            return;
        }

        if (!drone_ready_)
        {
            ROS_WARN_THROTTLE(2.0, "[coverage_planner] waiting for drone state...");
            return;
        }

        if (current_idx_ >= wps_.size())
        {
            mission_finished_ = true;
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
                ROS_INFO("[coverage_planner] all waypoints finished.");
            }
        }
    }

    void pushWp(const Eigen::Vector3d& p)
    {
        wps_.push_back(p);
    }

    void pushInterpolatedLine(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b,
                              double step)
    {
        const double len = (b - a).norm();
        const int n = std::max(1, (int)std::ceil(len / std::max(0.05, step)));
        for (int i = 1; i <= n; ++i)
        {
            const double t = (double)i / (double)n;
            pushWp(a + t * (b - a));
        }
    }

    void buildApproach()
    {
        Eigen::Vector3d p(base_x_ + approach_radius_, base_y_, base_z_ + approach_height_);
        pushWp(p);
    }

    void buildTowerSpiral()
    {
        const double scan_r = tower_radius_ + tower_offset_distance_;
        const double z0 = base_z_ + tower_z_start_;
        const double z1 = base_z_ + std::min(tower_z_end_, tower_height_);

        if (z1 <= z0)
        {
            ROS_WARN("[coverage_planner] invalid tower z range.");
            return;
        }

        const double circumference = 2.0 * M_PI * scan_r;
        const int n_per_circle = std::max(16, (int)std::ceil(circumference / tower_wp_arc_step_));

        const double total_h = z1 - z0;
        const double total_turns = total_h / tower_spiral_pitch_;
        const int total_steps = std::max(n_per_circle, (int)std::ceil(total_turns * n_per_circle));

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

    void buildNacelleOrbit()
    {
        const double r = nacelle_orbit_radius_;
        const double z = nacelle_orbit_z_;
        const double circumference = 2.0 * M_PI * r;
        const int n_per_circle = std::max(18, (int)std::ceil(circumference / nacelle_orbit_arc_step_));
        const int total_steps = std::max(n_per_circle, (int)std::ceil(nacelle_orbit_turns_ * n_per_circle));

        for (int k = 0; k <= total_steps; ++k)
        {
            const double ratio = (double)k / (double)total_steps;
            const double theta = 2.0 * M_PI * nacelle_orbit_turns_ * ratio;

            const double x = hub_x_ + r * std::cos(theta);
            const double y = hub_y_ + r * std::sin(theta);

            pushWp(Eigen::Vector3d(x, y, z));
        }
    }

    Eigen::Vector3d bladeDirFromAngleDeg(double deg)
    {
        // 叶轮平面近似取 yz 平面
        const double a = deg2rad(deg);
        Eigen::Vector3d d(0.0, std::cos(a), std::sin(a));
        if (d.norm() < 1e-6)
        {
            return Eigen::Vector3d(0.0, 0.0, 1.0);
        }
        return d.normalized();
    }

    void makeOrthonormalBasis(const Eigen::Vector3d& axis,
                              Eigen::Vector3d& e1,
                              Eigen::Vector3d& e2)
    {
        // 优先拿 x 轴做参考，这样螺旋更容易绕到叶片外侧
        Eigen::Vector3d ref(1.0, 0.0, 0.0);
        if (std::fabs(axis.dot(ref)) > 0.95)
        {
            ref = Eigen::Vector3d(0.0, 1.0, 0.0);
        }

        e1 = axis.cross(ref).normalized();
        e2 = axis.cross(e1).normalized();
    }

    void buildBladeHelixOne(const Eigen::Vector3d& dir, const std::string& name)
    {
        const Eigen::Vector3d hub(hub_x_, hub_y_, hub_z_);

        const double s0 = blade_root_radius_;
        const double s1 = std::max(s0 + 0.5, blade_length_ - blade_tip_margin_);
        const double axis_len = s1 - s0;

        if (axis_len <= 0.2)
        {
            ROS_WARN("[coverage_planner] invalid blade axis length for %s", name.c_str());
            return;
        }

        Eigen::Vector3d e1, e2;
        makeOrthonormalBasis(dir, e1, e2);

        const double turns = axis_len / std::max(0.2, blade_helix_pitch_);
        const int steps = std::max(20, (int)std::ceil(axis_len / std::max(0.05, blade_axis_step_)));

        ROS_INFO("[coverage_planner] append %s helix scan.", name.c_str());

        for (int k = 0; k <= steps; ++k)
        {
            const double ratio = (double)k / (double)steps;
            const double s = s0 + axis_len * ratio;
            const double theta = 2.0 * M_PI * turns * ratio;

            const Eigen::Vector3d center = hub + dir * s;
            const Eigen::Vector3d offset =
                blade_helix_radius_ * (std::cos(theta) * e1 + std::sin(theta) * e2);

            pushWp(center + offset);
        }
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

        buildApproach();      // 接近点
        buildTowerSpiral();   // 塔筒螺旋
        buildNacelleOrbit();  // 机舱只绕 1 圈

        // 三片叶片改为螺旋覆盖
        buildBladeHelixOne(bladeDirFromAngleDeg(90.0),  "blade_1");
        buildBladeHelixOne(bladeDirFromAngleDeg(210.0), "blade_2");
        buildBladeHelixOne(bladeDirFromAngleDeg(330.0), "blade_3");

        buildFinalHover();    // 结束悬停

        ROS_INFO("[coverage_planner] mission waypoints generated = %zu", wps_.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_turbine_coverage_planner");
    ros::NodeHandle nh("~");

    WindTurbineCoveragePlanner node(nh);

    ros::Duration(2.0).sleep();
    node.start();

    ros::spin();
    return 0;
}