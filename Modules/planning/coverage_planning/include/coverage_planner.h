#ifndef COVERAGE_PLANNER
#define COVERAGE_PLANNER

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cstdint>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"

#include "A_star.h"
#include "RRT.h"
#include "occupy_map.h"
#include "tools.h"
#include "message_utils.h"

using namespace std;

#define NODE_NAME "Coverage_Planner [main]"

#define MIN_DIS 0.1

namespace Coverage_Planning
{

using Global_Planning::Astar;
using Global_Planning::RRT;
using Global_Planning::Occupy_map;

extern ros::Publisher message_pub;

class Coverage_Planner
{
private:

    ros::NodeHandle global_planner_nh;

    // 参数
    int algorithm_mode;
    bool is_2D;
    double fly_height_2D;
    double safe_distance;
    double time_per_path;
    int map_input;
    double replan_time;
    bool consider_neighbour;
    bool sim_mode;
    bool map_groundtruth;

    // 本机位置
    // 邻机位置
    // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap
    // 调用路径规划算法 生成路径
    // 调用轨迹优化算法 规划轨迹

    // 订阅无人机状态、目标点、传感器数据（生成地图）
    ros::Subscriber goal_sub;
    ros::Subscriber drone_state_sub;
    // 支持2维激光雷达、3维激光雷达、D435i等实体传感器
    // 支持直接输入全局已知点云
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;
    // ？

    // 发布控制指令
    ros::Publisher command_pub,path_cmd_pub;
    // 发布飞行统计：data[0]=飞行时间(s), data[1]=实际飞行距离(m)
    ros::Publisher metrics_pub;
    ros::Timer mainloop_timer, track_path_timer, safety_timer;

    // A星规划器
    Astar::Ptr Astar_ptr;
    RRT::Ptr RRT_ptr;

    prometheus_msgs::DroneState _DroneState;
    nav_msgs::Odometry Drone_odom;

    nav_msgs::Path path_cmd;
    double distance_walked;
    prometheus_msgs::ControlCommand Command_Now;   

    double distance_to_goal;

    // 规划器状态
    bool odom_ready;
    bool drone_ready;
    bool sensor_ready;
    bool goal_ready; 
    bool is_safety;
    bool is_new_path;
    bool path_ok;
    int start_point_index;
    int Num_total_wp;
    int cur_id;

    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, start_vel, start_acc, goal_pos, goal_vel;

    float desired_yaw;

    ros::Time tra_start_time;
    float tra_running_time;

    // 飞行统计（实际飞行距离 & 到达时间）
    bool metrics_running{false};
    bool metrics_last_pos_valid{false};
    bool final_goal_cmd_sent{false};
    ros::Time metrics_start_time;
    Eigen::Vector3d metrics_last_pos;
    double metrics_distance_m{0.0};
    double arrive_dist{0.3};        // [m] 抵达判定距离阈值
    double arrive_vel{0.2};         // [m/s] 抵达判定速度阈值
    double metrics_min_step{0.001};  // [m] 去除抖动噪声

    // 覆盖任务参数（二维固定高度覆盖，视野为矩形）
    bool coverage_mode{false};
    bool coverage_initialized{false};
    bool coverage_started{false};
    double coverage_x_min{-10.0};
    double coverage_x_max{10.0};
    double coverage_y_min{-10.0};
    double coverage_y_max{10.0};
    double coverage_fixed_z{1.0};
    double coverage_fov_x{3.0};
    double coverage_fov_y{3.0};
    double coverage_grid_res{1.0};
    double coverage_strip_step{2.5};
    double coverage_line_margin{0.5};
    double coverage_goal_reach_dist{0.6};
    int coverage_nx{0};
    int coverage_ny{0};
    int coverage_wp_idx{0};
    std::vector<uint8_t> coverage_mask;
    std::vector<Eigen::Vector3d> coverage_waypoints;
    
    // 打印的提示消息
    string message;

    // 五种状态机
    enum EXEC_STATE
    {
        WAIT_GOAL,
        PLANNING,
        TRACKING,
        LANDING,
    };
    EXEC_STATE exec_state;

    // 回调函数
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr &msg);
    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr &msg);

    void safety_cb(const ros::TimerEvent& e);
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);
   

    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);

    int get_start_point_id(void);

    // 覆盖任务辅助函数
    Occupy_map::Ptr get_occupy_map();
    void init_coverage_mission();
    void mark_coverage_at_position(const Eigen::Vector3d& pos);
    bool is_coverage_complete() const;
    double get_coverage_ratio() const;
    Eigen::Vector3d coverage_cell_center(int ix, int iy) const;
    bool find_nearest_free_coverage_goal(const Eigen::Vector3d& desired, Eigen::Vector3d& adjusted);
    bool pick_next_coverage_goal();
    void finish_current_mission_and_report(const ros::Time& now_t, const std::string& prefix);
    
public:
    Coverage_Planner(void):
        global_planner_nh("~")
    {}~Coverage_Planner(){}

    void init(ros::NodeHandle& nh);
};

}

#endif