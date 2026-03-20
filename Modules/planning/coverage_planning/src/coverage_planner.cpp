#include "coverage_planner.h"

namespace Coverage_Planning
{

ros::Publisher message_pub;

// 初始化函数
void Coverage_Planner::init(ros::NodeHandle& nh)
{
    // 读取参数
    // TRUE代表2D平面规划及搜索,FALSE代表3D 
    nh.param("global_planner/is_2D", is_2D, true); 
    // 2D规划时,定高高度
    nh.param("global_planner/fly_height_2D", fly_height_2D, 1.0);  
    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("global_planner/safe_distance", safe_distance, 0.05);
    // 规划算法选择：0=A*, 1=RRT
    nh.param("global_planner/algorithm_mode", algorithm_mode, 0); 
    nh.param("global_planner/time_per_path", time_per_path, 1.0); 
    // 重规划频率 
    nh.param("global_planner/replan_time", replan_time, 2.0); 
    // 抵达判定阈值（用于统计飞行时间/距离）
    nh.param("global_planner/arrive_dist", arrive_dist, 0.3);
    nh.param("global_planner/arrive_vel", arrive_vel, 0.2);
    nh.param("global_planner/metrics_min_step", metrics_min_step, 0.01);

    nh.param("coverage/enable", coverage_mode, false);
    nh.param("coverage/x_min", coverage_x_min, -10.0);
    nh.param("coverage/x_max", coverage_x_max, 10.0);
    nh.param("coverage/y_min", coverage_y_min, -10.0);
    nh.param("coverage/y_max", coverage_y_max, 10.0);
    nh.param("coverage/fixed_z", coverage_fixed_z, 1.0);
    nh.param("coverage/fov_x", coverage_fov_x, 3.0);
    nh.param("coverage/fov_y", coverage_fov_y, 3.0);
    nh.param("coverage/grid_res", coverage_grid_res, 1.0);
    nh.param("coverage/strip_step", coverage_strip_step, 2.5);
    nh.param("coverage/line_margin", coverage_line_margin, 0.5);
    nh.param("coverage/goal_reach_dist", coverage_goal_reach_dist, 0.6);
    // 选择地图更新方式：　0代表全局点云，１代表局部点云，２代表激光雷达scan数据
    nh.param("global_planner/map_input", map_input, 0); 
    // 是否为仿真模式
    nh.param("global_planner/sim_mode", sim_mode, false); 

    nh.param("global_planner/map_groundtruth", map_groundtruth, false); 

    // 订阅 目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1, &Coverage_Planner::goal_cb, this);

    // 订阅 无人机状态
    drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Coverage_Planner::drone_state_cb, this);

    // 根据map_input选择地图更新方式
    if(map_input == 0)
    {
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/global_planning/global_pcl", 1, &Coverage_Planner::Gpointcloud_cb, this);
    }else if(map_input == 1)
    {
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/global_planning/local_pcl", 1, &Coverage_Planner::Lpointcloud_cb, this);
    }else if(map_input == 2)
    {
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/global_planning/laser_scan", 1, &Coverage_Planner::laser_cb, this);
    }

    // 发布 路径指令
    command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    // 发布提示消息
    message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);
    // 很关键：同步给 Global_Planning 命名空间下的 message_pub
    Global_Planning::message_pub = message_pub;
    // 发布路径用于显示
    path_cmd_pub   = nh.advertise<nav_msgs::Path>("/prometheus/global_planning/path_cmd",  10); 
    // 发布飞行统计
    metrics_pub = nh.advertise<std_msgs::Float64MultiArray>("/prometheus/global_planning/flight_metrics", 10);
    // 定时器 安全检测
    // safety_timer = nh.createTimer(ros::Duration(2.0), &Coverage_Planner::safety_cb, this); 
    // 定时器 规划器算法执行周期
    mainloop_timer = nh.createTimer(ros::Duration(1.5), &Coverage_Planner::mainloop_cb, this);        
    // 路径追踪循环，快速移动场景应当适当提高执行频率
    // time_per_path
    track_path_timer = nh.createTimer(ros::Duration(time_per_path), &Coverage_Planner::track_path_cb, this);
    // Planner algorithm
    if (algorithm_mode == 0)
    {
        Astar_ptr.reset(new Global_Planning::Astar);
        Astar_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "A_star init.");
    }
    else
    {
        RRT_ptr.reset(new Global_Planning::RRT);
        RRT_ptr->init(nh);
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "RRT init.");
    }
    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    drone_ready = false;
    goal_ready = false;
    sensor_ready = false;
    is_safety = true;
    is_new_path = false;

    // metrics init
    metrics_running = false;
    metrics_last_pos_valid = false;
    final_goal_cmd_sent = false;
    metrics_distance_m = 0.0;
    coverage_initialized = false;
    coverage_started = false;
    coverage_wp_idx = 0;
    coverage_mask.clear();
    coverage_waypoints.clear();

    // 初始化发布的指令
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.source = NODE_NAME;
    desired_yaw = 0.0;

    //　仿真模式下直接发送切换模式与起飞指令
    if(sim_mode == true)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Global Planner<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please input 1 for start:"<<endl;
            cin >> start_flag;
        }
        // 起飞
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();
    }else
    {
        //　真实飞行情况：等待飞机状态变为offboard模式，然后发送起飞指令
        // while(_DroneState.mode != "OFFBOARD")
        // {
        //     Command_Now.header.stamp = ros::Time::now();
        //     Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        //     Command_Now.Command_ID = 1 ;
        //     Command_Now.source = NODE_NAME;
        //     command_pub.publish(Command_Now);   
        //     cout << "Waiting for the offboard mode"<<endl;
        //     ros::Duration(1.0).sleep();
        //     ros::spinOnce();
        // }
        // Command_Now.header.stamp = ros::Time::now();
        // Command_Now.Mode = prometheus_msgs::ControlCommand::Takeoff;
        // Command_Now.Command_ID = Command_Now.Command_ID + 1;
        // Command_Now.source = NODE_NAME;
        // command_pub.publish(Command_Now);
        // cout << "Takeoff ..."<<endl;
    }

    if (coverage_mode)
    {
        init_coverage_mission();
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Coverage mission initialized. Waiting for start.");
    }

}

void Coverage_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (coverage_mode)
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,
                    "Coverage mode is enabled. External goal is ignored.");
        return;
    }

    if (is_2D == true)
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, fly_height_2D;
    }else
    {
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    }

    goal_vel.setZero();
    goal_ready = true;

    metrics_running = true;
    metrics_last_pos_valid = false;
    final_goal_cmd_sent = false;
    metrics_distance_m = 0.0;
    metrics_start_time = ros::Time::now();
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Get a new goal point");

    cout << "Get a new goal point:"<< goal_pos(0) << " [m] "  << goal_pos(1) << " [m] "  << goal_pos(2)<< " [m] "   <<endl;

    if(goal_pos(0) == 99 && goal_pos(1) == 99 )
    {
        path_ok = false;
        goal_ready = false;
        exec_state = EXEC_STATE::LANDING;
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME,"Land");
    }

}

void Coverage_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg)
{
    _DroneState = *msg;

    if (is_2D == true)
    {
        start_pos << msg->position[0], msg->position[1], fly_height_2D;
        start_vel << msg->velocity[0], msg->velocity[1], 0.0;

        if(abs(fly_height_2D - msg->position[2]) > 0.2)
        {
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,"Drone is not in the desired height.");
        }
    }else
    {
        start_pos << msg->position[0], msg->position[1], msg->position[2];
        start_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    }

    start_acc << 0.0, 0.0, 0.0;

    // 统计实际飞行距离：从收到goal/开始覆盖任务开始累计，期间不因重规划/状态切换中断（仅在LANDING时停）
    if (metrics_running && exec_state != EXEC_STATE::LANDING)
    {
        Eigen::Vector3d cur_pos(start_pos[0], start_pos[1], start_pos[2]);
        if (!metrics_last_pos_valid)
        {
            metrics_last_pos = cur_pos;
            metrics_last_pos_valid = true;
            if (metrics_start_time.isZero())
            {
                metrics_start_time = (_DroneState.header.stamp.isZero() ? ros::Time::now() : _DroneState.header.stamp);
            }
        }
        else
        {
            const double step = (cur_pos - metrics_last_pos).norm();
            if (step >= metrics_min_step)
            {
                metrics_distance_m += step;
                metrics_last_pos = cur_pos;
            }
        }

        if (coverage_mode)
        {
            mark_coverage_at_position(cur_pos);
        }
    }

    odom_ready = true;

    if (_DroneState.connected == true && _DroneState.armed == true )
    {
        drone_ready = true;
    }else
    {
        drone_ready = false;
    }

    Drone_odom.header = _DroneState.header;
    Drone_odom.child_frame_id = "base_link";

    Drone_odom.pose.pose.position.x = _DroneState.position[0];
    Drone_odom.pose.pose.position.y = _DroneState.position[1];
    Drone_odom.pose.pose.position.z = _DroneState.position[2];

    Drone_odom.pose.pose.orientation = _DroneState.attitude_q;
    Drone_odom.twist.twist.linear.x = _DroneState.velocity[0];
    Drone_odom.twist.twist.linear.y = _DroneState.velocity[1];
    Drone_odom.twist.twist.linear.z = _DroneState.velocity[2];
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Coverage_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    if(!map_groundtruth)
    {
        // 对Astar中的地图进行更新
        (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->map_update_gpcl(msg);
        // 并对地图进行膨胀
        (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->inflate_point_cloud(); 
    }else
    {
        static int update_num=0;
        update_num++;

        // 此处改为根据循环时间计算的数值
        if(update_num == 10)
        {
            // 对Astar中的地图进行更新
            (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->map_update_gpcl(msg);
            // 并对地图进行膨胀
            (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->inflate_point_cloud(); 
            update_num = 0;
        } 
    }
    
}

// 根据局部点云更新地图
// 情况：RGBD相机、三维激光雷达
void Coverage_Planner::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    // 对Astar中的地图进行更新（局部地图+odom）
    (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->map_update_lpcl(msg, Drone_odom);
    // 并对地图进行膨胀
    (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->inflate_point_cloud(); 
}

// 根据2维雷达数据更新地图
// 情况：2维激光雷达
void Coverage_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    /* need odom_ for center radius sensing */
    if (!odom_ready) 
    {
        return;
    }
    
    sensor_ready = true;

    // 对Astar中的地图进行更新（laser+odom）
    (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->map_update_laser(msg, Drone_odom);
    // 并对地图进行膨胀
    (algorithm_mode==0?Astar_ptr->Occupy_map_ptr:RRT_ptr->Occupy_map_ptr)->inflate_point_cloud(); 
}

void Coverage_Planner::track_path_cb(const ros::TimerEvent& e)
{
    if(!path_ok)
    {
        return;
    }

    // if(!is_safety)
    // {
    //     // 若无人机与障碍物之间的距离小于安全距离，则停止执行路径
    //     // 但如何脱离该点呢？
    //     pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Drone Position Dangerous! STOP HERE and wait for new goal.");
        
    //     Command_Now.header.stamp = ros::Time::now();
    //     Command_Now.Mode         = prometheus_msgs::ControlCommand::Hold;
    //     Command_Now.Command_ID   = Command_Now.Command_ID + 1;
    //     Command_Now.source = NODE_NAME;

    //     command_pub.publish(Command_Now);

    //     goal_ready = false;
    //     exec_state = EXEC_STATE::WAIT_GOAL;
        
    //     return;
    // }
    is_new_path = false;

    // 抵达终点
    if(cur_id == Num_total_wp - 1)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = goal_pos[0];
        Command_Now.Reference_State.position_ref[1]     = goal_pos[1];
        Command_Now.Reference_State.position_ref[2]     = goal_pos[2];

        Command_Now.Reference_State.yaw_ref             = desired_yaw;
        command_pub.publish(Command_Now);

        // 仅表示“已发送最终目标点”，真正抵达由 mainloop_cb 根据距离/速度判定
        if (!final_goal_cmd_sent)
        {
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Final goal command sent. Waiting for arrival...");
            final_goal_cmd_sent = true;
        }
        return;
    }
 
    // 计算距离开始追踪轨迹时间
    tra_running_time = get_time_in_sec(tra_start_time);

    int i = cur_id;

    cout << "Moving to Waypoint: [ " << cur_id << " / "<< Num_total_wp<< " ] "<<endl;
    cout << "Moving to Waypoint:"   << path_cmd.poses[i].pose.position.x  << " [m] "
                                    << path_cmd.poses[i].pose.position.y  << " [m] "
                                    << path_cmd.poses[i].pose.position.z  << " [m] "<<endl; 
    // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
    // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；
    
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
    Command_Now.source = NODE_NAME;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::TRAJECTORY;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = path_cmd.poses[i].pose.position.x;
    Command_Now.Reference_State.position_ref[1]     = path_cmd.poses[i].pose.position.y;
    Command_Now.Reference_State.position_ref[2]     = path_cmd.poses[i].pose.position.z;
    Command_Now.Reference_State.velocity_ref[0]     = (path_cmd.poses[i].pose.position.x - _DroneState.position[0])/time_per_path;
    Command_Now.Reference_State.velocity_ref[1]     = (path_cmd.poses[i].pose.position.y - _DroneState.position[1])/time_per_path;
    Command_Now.Reference_State.velocity_ref[2]     = (path_cmd.poses[i].pose.position.z - _DroneState.position[2])/time_per_path;
    Command_Now.Reference_State.yaw_ref             = desired_yaw;
    
    command_pub.publish(Command_Now);

    cur_id = cur_id + 1;
}
 
// 主循环 
void Coverage_Planner::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    // 此处打印消息与后面的冲突了，逻辑上存在问题
    if(!odom_ready || !drone_ready || !sensor_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 10)
        {
            if(!odom_ready)
            {
                message = "Need Odom.";
            }else if(!drone_ready)
            {
                message = "Drone is not ready.";
            }else if(!sensor_ready)
            {
                message = "Need sensor info.";
            }

            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            exec_num=0;
        }  

        return;
    }else
    {
        // 对检查的状态进行重置
        odom_ready = false;
        drone_ready = false;
        sensor_ready = false;
    }
    
    switch (exec_state)
    {
        case WAIT_GOAL:
        {
            path_ok = false;
            if (coverage_mode)
            {
                if (!coverage_started)
                {
                    coverage_started = true;
                    metrics_running = true;
                    metrics_last_pos_valid = false;
                    metrics_distance_m = 0.0;
                    metrics_start_time = ros::Time(0);
                }

                if (is_coverage_complete())
                {
                    finish_current_mission_and_report((_DroneState.header.stamp.isZero() ? ros::Time::now() : _DroneState.header.stamp),
                                                      "Coverage finished. ");
                    exec_state = EXEC_STATE::LANDING;
                }
                else if (pick_next_coverage_goal())
                {
                    exec_state = EXEC_STATE::PLANNING;
                    goal_ready = false;
                }
                else
                {
                    if(exec_num == 10)
                    {
                        std::ostringstream ss;
                        ss << "Coverage running. ratio=" << std::fixed << std::setprecision(1) << get_coverage_ratio() * 100.0 << "%";
                        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, ss.str());
                        exec_num=0;
                    }
                }
            }
            else
            {
                if(!goal_ready)
                {
                    if(exec_num == 10)
                    {
                        message = "Waiting for a new goal.";
                        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME,message);
                        exec_num=0;
                    }
                }else
                {
                    exec_state = EXEC_STATE::PLANNING;
                    goal_ready = false;
                }
            }

            break;
        }
        case PLANNING:
        {
            
            // 重置规划器
            if(algorithm_mode==0) Astar_ptr->reset(); else RRT_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果

            int astar_state;

            // Astar algorithm
            if(algorithm_mode==0) astar_state = Astar_ptr->search(start_pos, goal_pos);
            else astar_state = RRT_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if(astar_state==(algorithm_mode==0?Global_Planning::Astar::NO_PATH:Global_Planning::RRT::NO_PATH))
            {
                path_ok = false;
                exec_state = EXEC_STATE::WAIT_GOAL;
                if (coverage_mode)
                {
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Planner can't find path for current coverage goal, try next one.");
                }
                else
                {
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Planner can't find path!");
                }
            }
            else
            {
                path_ok = true;
                is_new_path = true;
                path_cmd = (algorithm_mode==0?Astar_ptr->get_ros_path():RRT_ptr->get_ros_path());
                Num_total_wp = path_cmd.poses.size();
                start_point_index = get_start_point_id();
                cur_id = start_point_index;
                tra_start_time = ros::Time::now();

                if (!coverage_mode && !metrics_running)
                {
                    metrics_running = true;
                    metrics_distance_m = 0.0;
                    metrics_start_time = ros::Time::now();
                    metrics_last_pos_valid = false;
                }
                final_goal_cmd_sent = false;

                exec_state = EXEC_STATE::TRACKING;
                path_cmd_pub.publish(path_cmd);
                pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "Get a new path!");       
            }

            break;
        }
        case TRACKING:
        {
            if (final_goal_cmd_sent && metrics_running)
            {
                Eigen::Vector3d cur_pos(_DroneState.position[0], _DroneState.position[1], _DroneState.position[2]);
                Eigen::Vector3d cur_vel(_DroneState.velocity[0], _DroneState.velocity[1], _DroneState.velocity[2]);
                const double d = (cur_pos - goal_pos).norm();
                const double v = cur_vel.norm();
                const double reach_dist = coverage_mode ? coverage_goal_reach_dist : arrive_dist;

                if (d <= reach_dist && v <= arrive_vel)
                {
                    const ros::Time now_t = (_DroneState.header.stamp.isZero() ? ros::Time::now() : _DroneState.header.stamp);
                    if (coverage_mode)
                    {
                        mark_coverage_at_position(cur_pos);
                        path_ok = false;
                        if (is_coverage_complete())
                        {
                            finish_current_mission_and_report(now_t, "Coverage finished. ");
                            exec_state = EXEC_STATE::LANDING;
                        }
                        else
                        {
                            exec_state = EXEC_STATE::WAIT_GOAL;
                        }
                    }
                    else
                    {
                        finish_current_mission_and_report(now_t, "");
                        path_ok = false;
                        exec_state = EXEC_STATE::WAIT_GOAL;
                    }
                    break;
                }
            }

            // 本循环是1Hz,此处不是很精准
            if(exec_num >= replan_time)
            {
                exec_state = EXEC_STATE::PLANNING;
                exec_num = 0;
            }

            break;
        }
        case  LANDING:
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode         = prometheus_msgs::ControlCommand::Land;
            Command_Now.Command_ID   = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;

            command_pub.publish(Command_Now);
            break;
        }
    }
}

Global_Planning::Occupy_map::Ptr Coverage_Planner::get_occupy_map()
{
    return (algorithm_mode == 0) ? Astar_ptr->Occupy_map_ptr : RRT_ptr->Occupy_map_ptr;
}

Eigen::Vector3d Coverage_Planner::coverage_cell_center(int ix, int iy) const
{
    return Eigen::Vector3d(coverage_x_min + (ix + 0.5) * coverage_grid_res,
                           coverage_y_min + (iy + 0.5) * coverage_grid_res,
                           coverage_fixed_z);
}

void Coverage_Planner::init_coverage_mission()
{
    coverage_initialized = true;
    coverage_started = false;
    coverage_wp_idx = 0;

    coverage_nx = std::max(1, static_cast<int>(std::ceil((coverage_x_max - coverage_x_min) / coverage_grid_res)));
    coverage_ny = std::max(1, static_cast<int>(std::ceil((coverage_y_max - coverage_y_min) / coverage_grid_res)));
    coverage_mask.assign(coverage_nx * coverage_ny, 0);
    coverage_waypoints.clear();

    const double ymin = coverage_y_min + std::min(coverage_fov_y * 0.5, coverage_line_margin);
    const double ymax = coverage_y_max - std::min(coverage_fov_y * 0.5, coverage_line_margin);
    const double x_start = coverage_x_min + coverage_fov_x * 0.5;
    const double x_end   = coverage_x_max - coverage_fov_x * 0.5;
    const double step = std::max(0.2, std::min(coverage_strip_step, coverage_fov_x));

    bool forward = true;
    for (double x = x_start; x <= x_end + 1e-6; x += step)
    {
        if (forward)
        {
            coverage_waypoints.emplace_back(x, ymin, coverage_fixed_z);
            coverage_waypoints.emplace_back(x, ymax, coverage_fixed_z);
        }
        else
        {
            coverage_waypoints.emplace_back(x, ymax, coverage_fixed_z);
            coverage_waypoints.emplace_back(x, ymin, coverage_fixed_z);
        }
        forward = !forward;
    }

    if (coverage_waypoints.empty())
    {
        coverage_waypoints.emplace_back((coverage_x_min + coverage_x_max) * 0.5,
                                        (coverage_y_min + coverage_y_max) * 0.5,
                                        coverage_fixed_z);
    }
}

void Coverage_Planner::mark_coverage_at_position(const Eigen::Vector3d& pos)
{
    if (!coverage_mode || coverage_mask.empty()) return;

    const double half_x = coverage_fov_x * 0.5;
    const double half_y = coverage_fov_y * 0.5;
    for (int ix = 0; ix < coverage_nx; ++ix)
    {
        const double cx = coverage_x_min + (ix + 0.5) * coverage_grid_res;
        if (std::abs(cx - pos.x()) > half_x) continue;
        for (int iy = 0; iy < coverage_ny; ++iy)
        {
            const double cy = coverage_y_min + (iy + 0.5) * coverage_grid_res;
            if (std::abs(cy - pos.y()) <= half_y)
            {
                coverage_mask[ix * coverage_ny + iy] = 1;
            }
        }
    }
}

bool Coverage_Planner::is_coverage_complete() const
{
    return !coverage_mask.empty() && std::all_of(coverage_mask.begin(), coverage_mask.end(), [](uint8_t v){ return v != 0; });
}

double Coverage_Planner::get_coverage_ratio() const
{
    if (coverage_mask.empty()) return 0.0;
    int covered = 0;
    for (uint8_t v : coverage_mask) if (v) ++covered;
    return static_cast<double>(covered) / static_cast<double>(coverage_mask.size());
}

bool Coverage_Planner::find_nearest_free_coverage_goal(const Eigen::Vector3d& desired, Eigen::Vector3d& adjusted)
{
    auto occ_map = get_occupy_map();
    if (!occ_map) return false;

    Eigen::Vector3d p = desired;
    p.z() = coverage_fixed_z;
    if (occ_map->isInMap(p) && occ_map->getOccupancy(p) == 0)
    {
        adjusted = p;
        return true;
    }

    const double step = std::max(occ_map->resolution_, 0.2);
    for (double r = step; r <= std::max(coverage_fov_x, coverage_fov_y) * 1.5; r += step)
    {
        for (int k = 0; k < 16; ++k)
        {
            const double ang = (2.0 * M_PI * k) / 16.0;
            Eigen::Vector3d cand(desired.x() + r * std::cos(ang), desired.y() + r * std::sin(ang), coverage_fixed_z);
            if (occ_map->isInMap(cand) && occ_map->getOccupancy(cand) == 0)
            {
                adjusted = cand;
                return true;
            }
        }
    }
    return false;
}

bool Coverage_Planner::pick_next_coverage_goal()
{
    if (!coverage_initialized)
        init_coverage_mission();

    Eigen::Vector3d adjusted;
    while (coverage_wp_idx < static_cast<int>(coverage_waypoints.size()))
    {
        Eigen::Vector3d desired = coverage_waypoints[coverage_wp_idx++];
        if ((desired.head<2>() - start_pos.head<2>()).norm() < coverage_goal_reach_dist * 0.5)
            continue;
        if (find_nearest_free_coverage_goal(desired, adjusted))
        {
            goal_pos = adjusted;
            goal_vel.setZero();
            final_goal_cmd_sent = false;
            std::ostringstream ss;
            ss << "Coverage target: (" << std::fixed << std::setprecision(2)
               << goal_pos.x() << ", " << goal_pos.y() << ", " << goal_pos.z() << ")"
               << ", ratio=" << std::setprecision(1) << get_coverage_ratio() * 100.0 << "%";
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, ss.str());
            return true;
        }
    }

    double best_d = 1e9;
    bool found = false;
    for (int ix = 0; ix < coverage_nx; ++ix)
    {
        for (int iy = 0; iy < coverage_ny; ++iy)
        {
            if (coverage_mask[ix * coverage_ny + iy]) continue;
            Eigen::Vector3d center = coverage_cell_center(ix, iy);
            if (find_nearest_free_coverage_goal(center, adjusted))
            {
                double d = (adjusted.head<2>() - start_pos.head<2>()).norm();
                if (d < best_d)
                {
                    best_d = d;
                    goal_pos = adjusted;
                    found = true;
                }
            }
        }
    }

    if (found)
    {
        goal_vel.setZero();
        final_goal_cmd_sent = false;
        std::ostringstream ss;
        ss << "Coverage backfill target: (" << std::fixed << std::setprecision(2)
           << goal_pos.x() << ", " << goal_pos.y() << ", " << goal_pos.z() << ")"
           << ", ratio=" << std::setprecision(1) << get_coverage_ratio() * 100.0 << "%";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, ss.str());
        return true;
    }

    return false;
}

void Coverage_Planner::finish_current_mission_and_report(const ros::Time& now_t, const std::string& prefix)
{
    const double t = (metrics_start_time.isZero() ? 0.0 : (now_t - metrics_start_time).toSec());
    std_msgs::Float64MultiArray m;
    m.data.resize(2);
    m.data[0] = t;
    m.data[1] = metrics_distance_m;
    metrics_pub.publish(m);

    std::ostringstream ss;
    ss << prefix << "Arrived. time=" << std::fixed << std::setprecision(2) << t
       << " s, distance=" << metrics_distance_m << " m";
    if (coverage_mode)
    {
        ss << ", coverage=" << std::setprecision(1) << get_coverage_ratio() * 100.0 << "%";
    }
    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, ss.str());

    metrics_running = false;
    final_goal_cmd_sent = false;
}

// 【获取当前时间函数】 单位：秒
float Coverage_Planner::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Coverage_Planner::safety_cb(const ros::TimerEvent& e)
{
    Eigen::Vector3d cur_pos(_DroneState.position[0], _DroneState.position[1], _DroneState.position[2]);
    
    is_safety = (algorithm_mode==0?Astar_ptr->check_safety(cur_pos, safe_distance):RRT_ptr->check_safety(cur_pos, safe_distance));
}

int Coverage_Planner::get_start_point_id(void)
{
    // 选择与当前无人机所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[0].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[0].pose.position.z - _DroneState.position[2]);
    
    float distance_to_wp;

    for (int j=1; j<Num_total_wp;j++)
    {
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - _DroneState.position[0])
                                + abs(path_cmd.poses[j].pose.position.y - _DroneState.position[1])
                                + abs(path_cmd.poses[j].pose.position.z - _DroneState.position[2]);
        
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 2 < Num_total_wp)
    {
        id = id + 2;
    }

    return id;
}



}