#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <prometheus_msgs/DroneState.h>
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <string>

class WindTurbineCoveragePlanner
{
public:
    WindTurbineCoveragePlanner(ros::NodeHandle& nh)
    {
        nh.param("coverage/frame_id", frame_id_, std::string("world"));

        nh.param("coverage/base_x", base_x_, 0.0);
        nh.param("coverage/base_y", base_y_, 0.0);
        nh.param("coverage/base_z", base_z_, 0.0);

        nh.param("coverage/tower_radius", tower_radius_, 0.35);
        nh.param("coverage/tower_height", tower_height_, 13.6);

        // 螺旋扫描参数
        nh.param("coverage/offset_distance", offset_distance_, 2.0);   // 离塔筒表面的偏置距离
        nh.param("coverage/spiral_pitch", spiral_pitch_, 1.0);         // 每绕一圈上升高度
        nh.param("coverage/wp_arc_step", wp_arc_step_, 0.4);           // 沿圆周离散的弧长步长
        nh.param("coverage/z_start", z_start_, 1.5);
        nh.param("coverage/z_end", z_end_, 12.5);

        nh.param("coverage/reach_dist", reach_dist_, 0.4);
        nh.param("coverage/publish_interval", publish_interval_, 1.0);
        
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1);
        drone_sub_ = nh.subscribe("/prometheus/drone_state", 10, &WindTurbineCoveragePlanner::droneStateCb, this);

        timer_ = nh.createTimer(ros::Duration(publish_interval_), &WindTurbineCoveragePlanner::timerCb, this, false, false);

        drone_ready_ = false;
        current_idx_ = 0;
        mission_started_ = false;

        buildTowerSpiral();
    }

    void start()
    {
        if (!wps_.empty())
        {
            timer_.start();
            mission_started_ = true;
            ROS_INFO("[coverage_planner] mission started, total waypoints = %zu", wps_.size());
        }
    }

private:
    ros::Publisher goal_pub_;
    ros::Subscriber drone_sub_;
    ros::Timer timer_;

    std::string frame_id_;

    double base_x_, base_y_, base_z_;
    double tower_radius_, tower_height_;

    double offset_distance_;
    double spiral_pitch_;
    double wp_arc_step_;
    double z_start_, z_end_;

    double reach_dist_;
    double publish_interval_;

    bool drone_ready_;
    bool mission_started_;
    size_t current_idx_;
    Eigen::Vector3d drone_pos_;

    std::vector<Eigen::Vector3d> wps_;

private:
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

        ROS_INFO("[coverage_planner] send goal: %.2f %.2f %.2f", p.x(), p.y(), p.z());
    }

    void timerCb(const ros::TimerEvent&)
    {
        if (!mission_started_ || !drone_ready_ || wps_.empty())
        {
            return;
        }

        if (current_idx_ >= wps_.size())
        {
            ROS_INFO_THROTTLE(2.0, "[coverage_planner] mission finished.");
            return;
        }

        Eigen::Vector3d target = wps_[current_idx_];
        double dist = (drone_pos_ - target).norm();

        // 第一个点或者已接近当前点，就推进到下一个
        static bool first_send = true;
        if (first_send)
        {
            publishGoal(target);
            first_send = false;
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
                ROS_INFO("[coverage_planner] all waypoints finished.");
            }
        }
    }

    void buildTowerSpiral()
    {
        wps_.clear();

        double scan_r = tower_radius_ + offset_distance_;
        double z0 = base_z_ + z_start_;
        double z1 = base_z_ + std::min(z_end_, tower_height_);

        if (z1 <= z0)
        {
            ROS_WARN("[coverage_planner] invalid z range.");
            return;
        }

        // 每圈周长
        double circumference = 2.0 * M_PI * scan_r;
        int n_per_circle = std::max(12, (int)std::ceil(circumference / wp_arc_step_));

        // 总圈数
        double total_h = z1 - z0;
        double total_turns = total_h / spiral_pitch_;
        int total_steps = std::max(n_per_circle, (int)std::ceil(total_turns * n_per_circle));

        for (int k = 0; k <= total_steps; ++k)
        {
            double ratio = (double)k / (double)total_steps;
            double theta = 2.0 * M_PI * total_turns * ratio;
            double z = z0 + total_h * ratio;

            double x = base_x_ + scan_r * std::cos(theta);
            double y = base_y_ + scan_r * std::sin(theta);

            wps_.push_back(Eigen::Vector3d(x, y, z));
        }

        ROS_INFO("[coverage_planner] spiral waypoints generated = %zu", wps_.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wind_turbine_coverage_planner");
    ros::NodeHandle nh("~");

    WindTurbineCoveragePlanner node(nh);

    ros::Duration(2.0).sleep(); // 给状态与地图一点启动时间
    node.start();

    ros::spin();
    return 0;
}