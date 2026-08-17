#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <boost/bind.hpp>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "task_allocate/astar_path_planner.h"
#include "task_allocate/common_types.h"
#include "task_allocate/geometry_utils.h"
#include "task_allocate/mappo_actor.h"
#include "task_allocate/mappo_sequential_solver.h"
#include "task_allocate/path_command_dispatcher.h"
#include "task_allocate/sensor_obstacle_map.h"
#include "task_allocate/visualization_utils.h"

using namespace task_allocate;

#define NODE_NAME "swarm_task_assignment_mappo"
#define MAX_UAV_NUM 50

class SwarmTaskAssignmentMappoNode
{
public:
    SwarmTaskAssignmentMappoNode() : nh_("~")
    {
        loadBasicParams();
        loadUavModelNames();
        loadUavMeshResources();
        loadUavCapabilityMatrix();
        control_uavs_ = uavs_;
        loadTaskList();
        loadObstacleList();
        loadDemoUavPositions();
        loadActorParams();
        loadAStarParams();
        initRosIO();

        actor_.setObstacles(obstacles_);
        solver_.setActor(actor_);
        astar_planner_.setParams(astar_params_);
        astar_planner_.setObstacles(use_param_obstacles_for_astar_ ? obstacles_ : std::vector<StaticObstacle>());
        sensor_map_.setParams(sensor_map_params_);
        path_dispatcher_.setParams(path_dispatch_params_);

        ROS_INFO("[%s] task_allocate MAPPO + sensor obstacle risk + A* node started.", NODE_NAME);
        ROS_INFO("[%s] swarm_num_uav=%d task_num=%zu obstacle_num=%zu auto_dispatch=%s",
                 NODE_NAME, swarm_num_uav_, tasks_.size(), obstacles_.size(),
                 auto_dispatch_commands_ ? "true" : "false");
        ROS_INFO("[%s] assignment=MAPPO_sequential_action_mask, risk=%s, planner=%s, sensor_map=%s",
                 NODE_NAME,
                 enable_obstacle_risk_in_assignment_ ? "enabled" : "disabled",
                 enable_astar_path_planning_ ? "A*_sensor_map" : "disabled",
                 sensor_map_params_.enabled ? "enabled" : "disabled");

        assignment_timer_ = nh_.createTimer(ros::Duration(assignment_period_),
                                            &SwarmTaskAssignmentMappoNode::assignmentTimerCb, this);
        uav_model_marker_timer_ = nh_.createTimer(ros::Duration(uav_model_marker_period_),
                                                  &SwarmTaskAssignmentMappoNode::uavModelMarkerTimerCb, this);
        path_command_timer_ = nh_.createTimer(ros::Duration(std::max(0.05, path_dispatch_params_.command_period)),
                                             &SwarmTaskAssignmentMappoNode::pathCommandTimerCb, this);
    }

private:
    ros::NodeHandle nh_;

    int swarm_num_uav_;
    bool use_demo_positions_;
    bool auto_dispatch_commands_;
    bool run_once_;
    bool has_run_once_;
    double assignment_period_;
    std::string frame_id_;
    double default_energy_;
    double default_height_;
    double yaw_ref_;
    double max_distance_;

    bool enable_rviz_visualization_;
    bool visualize_uav_as_mesh_;
    bool use_gazebo_model_states_for_rviz_uav_;
    bool use_gazebo_model_states_for_assignment_;
    bool wait_for_gazebo_model_states_;
    std::string default_uav_mesh_resource_;
    double rviz_uav_mesh_scale_;
    double rviz_uav_mesh_z_offset_;
    double uav_model_marker_period_;

    bool enable_gazebo_visualization_;
    bool gazebo_respawn_each_assignment_;
    bool gazebo_models_spawned_;
    bool gazebo_visual_collision_;
    bool gazebo_obstacle_collision_;
    int gazebo_path_skip_start_points_;
    int gazebo_path_skip_goal_points_;
    double gazebo_path_marker_z_offset_;
    double gazebo_task_radius_;
    double gazebo_path_marker_radius_;

    bool enable_obstacle_visualization_;
    bool enable_obstacle_risk_in_assignment_;
    double obstacle_risk_weight_;
    double obstacle_risk_safe_margin_;
    bool risk_include_danger_zone_;
    int risk_sample_num_;

    bool enable_astar_path_planning_;
    bool use_param_obstacles_for_assignment_;
    bool use_param_obstacles_for_astar_;
    bool wait_for_sensor_map_before_planning_;
    bool publish_only_astar_success_path_;
    AStarParams astar_params_;

    SensorMapParams sensor_map_params_;
    std::string pointcloud_topic_template_;
    std::string laserscan_topic_template_;
    bool enable_sensor_stop_failsafe_;
    double sensor_stop_distance_;

    PathDispatchParams path_dispatch_params_;
    std::vector<PathPlan> latest_path_plans_;
    bool have_latest_paths_;
    bool dispatch_started_;
    bool replan_while_dispatching_;
    bool dispatch_allow_preflight_without_path_;
    bool dispatch_preflight_only_active_;

    std::vector<UavInfo> uavs_;                 // world/assignment state, may be overwritten by /gazebo/model_states.
    std::vector<UavInfo> control_uavs_;         // real control state from /prometheus/drone_state, used only for command dispatch.
    std::vector<TaskInfo> tasks_;
    std::vector<StaticObstacle> obstacles_;
    std::vector<std::string> uav_model_names_;
    std::vector<std::string> uav_mesh_resources_;
    std::map<std::string, geometry_msgs::Pose> gazebo_model_pose_map_;
    std::vector<std::string> gazebo_spawned_model_names_;

    LinearMappoActor actor_;
    MappoSequentialSolver solver_;
    AStarPathPlanner astar_planner_;
    SensorObstacleMap sensor_map_;
    PathCommandDispatcher path_dispatcher_;

    ros::Subscriber drone_state_sub_[MAX_UAV_NUM + 1];
    ros::Subscriber gazebo_model_state_sub_;
    ros::Subscriber pointcloud_sub_[MAX_UAV_NUM + 1];
    ros::Subscriber laserscan_sub_[MAX_UAV_NUM + 1];
    ros::Publisher command_pub_[MAX_UAV_NUM + 1];
    ros::Publisher path_pub_[MAX_UAV_NUM + 1];
    ros::Publisher marker_pub_;
    ros::Publisher uav_model_marker_pub_;
    ros::Publisher result_text_pub_;
    ros::ServiceClient gazebo_spawn_client_;
    ros::ServiceClient gazebo_delete_client_;
    ros::Timer assignment_timer_;
    ros::Timer uav_model_marker_timer_;
    ros::Timer path_command_timer_;
    int command_id_[MAX_UAV_NUM + 1];

private:
    void loadBasicParams()
    {
        swarm_num_uav_ = 5;
        use_demo_positions_ = false;
        auto_dispatch_commands_ = false;
        run_once_ = false;
        has_run_once_ = false;
        assignment_period_ = 2.0;
        frame_id_ = "world";
        default_energy_ = 100.0;
        default_height_ = 2.5;
        yaw_ref_ = 0.0;
        max_distance_ = 90.0;

        enable_rviz_visualization_ = true;
        visualize_uav_as_mesh_ = true;
        use_gazebo_model_states_for_rviz_uav_ = true;
        use_gazebo_model_states_for_assignment_ = true;
        wait_for_gazebo_model_states_ = true;
        default_uav_mesh_resource_ = "";
        rviz_uav_mesh_scale_ = 1.0;
        rviz_uav_mesh_z_offset_ = 0.0;
        uav_model_marker_period_ = 0.10;

        enable_gazebo_visualization_ = true;
        gazebo_respawn_each_assignment_ = false;
        gazebo_models_spawned_ = false;
        gazebo_visual_collision_ = false;
        gazebo_obstacle_collision_ = true;
        gazebo_path_skip_start_points_ = 2;
        gazebo_path_skip_goal_points_ = 1;
        gazebo_path_marker_z_offset_ = 0.20;
        gazebo_task_radius_ = 0.55;
        gazebo_path_marker_radius_ = 0.15;

        enable_obstacle_visualization_ = true;
        enable_obstacle_risk_in_assignment_ = true;
        obstacle_risk_weight_ = 4.0;
        obstacle_risk_safe_margin_ = 5.0;
        risk_include_danger_zone_ = true;
        risk_sample_num_ = 60;

        enable_astar_path_planning_ = true;
        use_param_obstacles_for_assignment_ = false;
        use_param_obstacles_for_astar_ = false;
        wait_for_sensor_map_before_planning_ = false;
        publish_only_astar_success_path_ = true;
        have_latest_paths_ = false;
        dispatch_started_ = false;
        replan_while_dispatching_ = false;
        dispatch_allow_preflight_without_path_ = true;
        dispatch_preflight_only_active_ = false;

        sensor_map_params_ = SensorMapParams();
        pointcloud_topic_template_ = "/uav%d/prometheus/sensors/pcl2";
        laserscan_topic_template_ = "/uav%d/prometheus/sensors/2Dlidar_scan";
        enable_sensor_stop_failsafe_ = true;
        sensor_stop_distance_ = 1.2;

        path_dispatch_params_ = PathDispatchParams();

        for(int i = 0; i <= MAX_UAV_NUM; ++i)
        {
            command_id_[i] = 0;
        }

        nh_.param<int>("swarm_num_uav", swarm_num_uav_, swarm_num_uav_);
        nh_.param<bool>("use_demo_positions", use_demo_positions_, use_demo_positions_);
        nh_.param<bool>("auto_dispatch_commands", auto_dispatch_commands_, auto_dispatch_commands_);
        nh_.param<bool>("run_once", run_once_, run_once_);
        nh_.param<double>("assignment_period", assignment_period_, assignment_period_);
        nh_.param<std::string>("frame_id", frame_id_, frame_id_);
        nh_.param<double>("default_energy", default_energy_, default_energy_);
        nh_.param<double>("default_height", default_height_, default_height_);
        nh_.param<double>("yaw_ref", yaw_ref_, yaw_ref_);
        nh_.param<double>("max_distance", max_distance_, max_distance_);

        nh_.param<bool>("enable_rviz_visualization", enable_rviz_visualization_, enable_rviz_visualization_);
        nh_.param<bool>("visualize_uav_as_mesh", visualize_uav_as_mesh_, visualize_uav_as_mesh_);
        nh_.param<bool>("use_gazebo_model_states_for_rviz_uav", use_gazebo_model_states_for_rviz_uav_, use_gazebo_model_states_for_rviz_uav_);
        nh_.param<bool>("use_gazebo_model_states_for_assignment", use_gazebo_model_states_for_assignment_, use_gazebo_model_states_for_assignment_);
        nh_.param<bool>("wait_for_gazebo_model_states", wait_for_gazebo_model_states_, wait_for_gazebo_model_states_);
        nh_.param<std::string>("default_uav_mesh_resource", default_uav_mesh_resource_, default_uav_mesh_resource_);
        nh_.param<double>("rviz_uav_mesh_scale", rviz_uav_mesh_scale_, rviz_uav_mesh_scale_);
        nh_.param<double>("rviz_uav_mesh_z_offset", rviz_uav_mesh_z_offset_, rviz_uav_mesh_z_offset_);
        nh_.param<double>("uav_model_marker_period", uav_model_marker_period_, uav_model_marker_period_);

        nh_.param<bool>("enable_gazebo_visualization", enable_gazebo_visualization_, enable_gazebo_visualization_);
        nh_.param<bool>("gazebo_respawn_each_assignment", gazebo_respawn_each_assignment_, gazebo_respawn_each_assignment_);
        nh_.param<bool>("gazebo_visual_collision", gazebo_visual_collision_, gazebo_visual_collision_);
        nh_.param<bool>("gazebo_obstacle_collision", gazebo_obstacle_collision_, gazebo_obstacle_collision_);
        nh_.param<int>("gazebo_path_skip_start_points", gazebo_path_skip_start_points_, gazebo_path_skip_start_points_);
        nh_.param<int>("gazebo_path_skip_goal_points", gazebo_path_skip_goal_points_, gazebo_path_skip_goal_points_);
        nh_.param<double>("gazebo_path_marker_z_offset", gazebo_path_marker_z_offset_, gazebo_path_marker_z_offset_);
        nh_.param<double>("gazebo_task_radius", gazebo_task_radius_, gazebo_task_radius_);
        nh_.param<double>("gazebo_path_marker_radius", gazebo_path_marker_radius_, gazebo_path_marker_radius_);

        nh_.param<bool>("enable_obstacle_visualization", enable_obstacle_visualization_, enable_obstacle_visualization_);
        nh_.param<bool>("enable_obstacle_risk_in_assignment", enable_obstacle_risk_in_assignment_, enable_obstacle_risk_in_assignment_);
        nh_.param<double>("obstacle_risk_weight", obstacle_risk_weight_, obstacle_risk_weight_);
        nh_.param<double>("obstacle_risk_safe_margin", obstacle_risk_safe_margin_, obstacle_risk_safe_margin_);
        nh_.param<bool>("risk_include_danger_zone", risk_include_danger_zone_, risk_include_danger_zone_);
        nh_.param<int>("risk_sample_num", risk_sample_num_, risk_sample_num_);

        nh_.param<bool>("enable_astar_path_planning", enable_astar_path_planning_, enable_astar_path_planning_);
        nh_.param<bool>("use_param_obstacles_for_assignment", use_param_obstacles_for_assignment_, use_param_obstacles_for_assignment_);
        nh_.param<bool>("use_param_obstacles_for_astar", use_param_obstacles_for_astar_, use_param_obstacles_for_astar_);
        nh_.param<bool>("wait_for_sensor_map_before_planning", wait_for_sensor_map_before_planning_, wait_for_sensor_map_before_planning_);
        nh_.param<bool>("publish_only_astar_success_path", publish_only_astar_success_path_, publish_only_astar_success_path_);

        nh_.param<bool>("sensor_map/enabled", sensor_map_params_.enabled, sensor_map_params_.enabled);
        nh_.param<bool>("sensor_map/subscribe_pointcloud", sensor_map_params_.subscribe_pointcloud, sensor_map_params_.subscribe_pointcloud);
        nh_.param<bool>("sensor_map/subscribe_laserscan", sensor_map_params_.subscribe_laserscan, sensor_map_params_.subscribe_laserscan);
        nh_.param<bool>("sensor_map/sensor_points_are_world_frame", sensor_map_params_.sensor_points_are_world_frame, sensor_map_params_.sensor_points_are_world_frame);
        nh_.param<double>("sensor_map/max_range", sensor_map_params_.max_range, sensor_map_params_.max_range);
        nh_.param<double>("sensor_map/z_min", sensor_map_params_.z_min, sensor_map_params_.z_min);
        nh_.param<double>("sensor_map/z_max", sensor_map_params_.z_max, sensor_map_params_.z_max);
        nh_.param<double>("sensor_map/downsample_resolution", sensor_map_params_.downsample_resolution, sensor_map_params_.downsample_resolution);
        nh_.param<double>("sensor_map/point_lifetime", sensor_map_params_.point_lifetime, sensor_map_params_.point_lifetime);
        nh_.param<int>("sensor_map/max_points", sensor_map_params_.max_points, sensor_map_params_.max_points);
        nh_.param<double>("sensor_map/self_filter_radius", sensor_map_params_.self_filter_radius, sensor_map_params_.self_filter_radius);
        nh_.param<std::string>("sensor_map/pointcloud_topic_template", pointcloud_topic_template_, pointcloud_topic_template_);
        nh_.param<std::string>("sensor_map/laserscan_topic_template", laserscan_topic_template_, laserscan_topic_template_);
        nh_.param<bool>("sensor_stop_failsafe/enabled", enable_sensor_stop_failsafe_, enable_sensor_stop_failsafe_);
        nh_.param<double>("sensor_stop_failsafe/stop_distance", sensor_stop_distance_, sensor_stop_distance_);

        nh_.param<bool>("path_dispatch/enabled", path_dispatch_params_.enabled, path_dispatch_params_.enabled);
        nh_.param<bool>("path_dispatch/replan_while_dispatching", replan_while_dispatching_, replan_while_dispatching_);
        nh_.param<bool>("path_dispatch/allow_preflight_without_path", dispatch_allow_preflight_without_path_, dispatch_allow_preflight_without_path_);
        nh_.param<double>("path_dispatch/waypoint_reach_threshold", path_dispatch_params_.waypoint_reach_threshold, path_dispatch_params_.waypoint_reach_threshold);
        nh_.param<double>("path_dispatch/command_period", path_dispatch_params_.command_period, path_dispatch_params_.command_period);
        nh_.param<double>("path_dispatch/yaw_ref", path_dispatch_params_.yaw_ref, yaw_ref_);
        nh_.param<int>("path_dispatch/min_waypoint_index", path_dispatch_params_.min_waypoint_index, path_dispatch_params_.min_waypoint_index);
        nh_.param<bool>("path_dispatch/enable_preflight_sequence", path_dispatch_params_.enable_preflight_sequence, path_dispatch_params_.enable_preflight_sequence);
        nh_.param<double>("path_dispatch/offboard_prepare_time", path_dispatch_params_.offboard_prepare_time, path_dispatch_params_.offboard_prepare_time);
        nh_.param<double>("path_dispatch/takeoff_reach_height", path_dispatch_params_.takeoff_reach_height, path_dispatch_params_.takeoff_reach_height);
        nh_.param<double>("path_dispatch/takeoff_timeout", path_dispatch_params_.takeoff_timeout, path_dispatch_params_.takeoff_timeout);
    }

    double xmlToDouble(const XmlRpc::XmlRpcValue& v) const
    {
        if(v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            return static_cast<int>(v);
        }
        if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            return static_cast<double>(v);
        }
        std::stringstream ss;
        ss << v;
        double out = 0.0;
        ss >> out;
        return out;
    }

    int xmlToInt(const XmlRpc::XmlRpcValue& v) const
    {
        return static_cast<int>(std::round(xmlToDouble(v)));
    }

    bool parseStringArray(const XmlRpc::XmlRpcValue& row, std::vector<std::string>& values) const
    {
        if(row.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            return false;
        }
        values.clear();
        for(int i = 0; i < row.size(); ++i)
        {
            std::stringstream ss;
            ss << row[i];
            std::string s = ss.str();
            if(row[i].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                s = static_cast<std::string>(row[i]);
            }
            values.push_back(s);
        }
        return true;
    }

    void loadUavModelNames()
    {
        uav_model_names_.clear();
        uav_model_names_.resize(swarm_num_uav_);
        for(int i = 0; i < swarm_num_uav_; ++i)
        {
            uav_model_names_[i] = "uav" + std::to_string(i + 1);
        }

        XmlRpc::XmlRpcValue names;
        if(nh_.getParam("uav_model_names", names) && names.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            std::vector<std::string> parsed;
            if(parseStringArray(names, parsed))
            {
                for(size_t i = 0; i < parsed.size() && i < uav_model_names_.size(); ++i)
                {
                    if(!parsed[i].empty())
                    {
                        uav_model_names_[i] = parsed[i];
                    }
                }
            }
        }
    }

    void loadUavMeshResources()
    {
        uav_mesh_resources_.clear();
        uav_mesh_resources_.resize(swarm_num_uav_, default_uav_mesh_resource_);
        XmlRpc::XmlRpcValue meshes;
        if(nh_.getParam("uav_mesh_resources", meshes) && meshes.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            std::vector<std::string> parsed;
            if(parseStringArray(meshes, parsed))
            {
                for(size_t i = 0; i < parsed.size() && i < uav_mesh_resources_.size(); ++i)
                {
                    if(!parsed[i].empty())
                    {
                        uav_mesh_resources_[i] = parsed[i];
                    }
                }
            }
        }
    }

    void loadUavCapabilityMatrix()
    {
        uavs_.clear();
        uavs_.resize(swarm_num_uav_);
        for(int i = 0; i < swarm_num_uav_; ++i)
        {
            uavs_[i].id = i + 1;
            uavs_[i].energy = default_energy_;
            uavs_[i].available = true;
            uavs_[i].has_state = false;
            uavs_[i].position = Eigen::Vector3d(0.0, 0.0, default_height_);
            uavs_[i].capability = std::vector<int>(3, 1);
        }

        XmlRpc::XmlRpcValue mat;
        if(nh_.getParam("uav_capability_matrix", mat) && mat.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < mat.size() && i < swarm_num_uav_; ++i)
            {
                if(mat[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
                {
                    continue;
                }
                uavs_[i].capability.assign(3, 0);
                for(int j = 0; j < mat[i].size() && j < 3; ++j)
                {
                    uavs_[i].capability[j] = xmlToInt(mat[i][j]);
                }
            }
        }
    }

    void loadTaskList()
    {
        tasks_.clear();
        XmlRpc::XmlRpcValue list;
        if(nh_.getParam("task_list", list) && list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < list.size(); ++i)
            {
                if(list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 6)
                {
                    continue;
                }
                TaskInfo task;
                task.id = xmlToInt(list[i][0]);
                task.type = xmlToInt(list[i][1]);
                task.position = Eigen::Vector3d(xmlToDouble(list[i][2]),
                                                xmlToDouble(list[i][3]),
                                                xmlToDouble(list[i][4]));
                task.priority = xmlToDouble(list[i][5]);
                task.assigned = false;
                tasks_.push_back(task);
            }
        }

        if(tasks_.empty())
        {
            TaskInfo t;
            t.id = 1; t.type = ATTACK; t.position = Eigen::Vector3d(42.0, -22.0, default_height_); t.priority = 9.0; tasks_.push_back(t);
            t.id = 2; t.type = RECON;  t.position = Eigen::Vector3d(35.0,  22.0, default_height_); t.priority = 7.0; tasks_.push_back(t);
            t.id = 3; t.type = JAM;    t.position = Eigen::Vector3d(54.0,  -6.0, default_height_); t.priority = 8.0; tasks_.push_back(t);
            t.id = 4; t.type = RECON;  t.position = Eigen::Vector3d(28.0, -12.0, default_height_); t.priority = 5.0; tasks_.push_back(t);
            t.id = 5; t.type = ATTACK; t.position = Eigen::Vector3d(58.0,  16.0, default_height_); t.priority = 6.0; tasks_.push_back(t);
        }
    }

    void loadObstacleList()
    {
        obstacles_.clear();
        XmlRpc::XmlRpcValue list;
        if(nh_.getParam("obstacle_list", list) && list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < list.size(); ++i)
            {
                if(list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 12)
                {
                    continue;
                }
                StaticObstacle obs;
                obs.id = xmlToInt(list[i][0]);
                obs.type = xmlToInt(list[i][1]);
                obs.position = Eigen::Vector3d(xmlToDouble(list[i][2]), xmlToDouble(list[i][3]), xmlToDouble(list[i][4]));
                obs.size = Eigen::Vector3d(xmlToDouble(list[i][5]), xmlToDouble(list[i][6]), xmlToDouble(list[i][7]));
                obs.r = xmlToDouble(list[i][8]);
                obs.g = xmlToDouble(list[i][9]);
                obs.b = xmlToDouble(list[i][10]);
                obs.a = xmlToDouble(list[i][11]);
                obstacles_.push_back(obs);
            }
        }
    }

    void loadDemoUavPositions()
    {
        XmlRpc::XmlRpcValue list;
        if(nh_.getParam("demo_uav_positions", list) && list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < list.size(); ++i)
            {
                if(list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 4)
                {
                    continue;
                }
                const int id = xmlToInt(list[i][0]);
                if(id < 1 || id > swarm_num_uav_)
                {
                    continue;
                }
                uavs_[id - 1].position = Eigen::Vector3d(xmlToDouble(list[i][1]),
                                                          xmlToDouble(list[i][2]),
                                                          xmlToDouble(list[i][3]));
                uavs_[id - 1].has_state = true;
            }
        }
        else
        {
            for(int i = 0; i < swarm_num_uav_; ++i)
            {
                uavs_[i].position = Eigen::Vector3d(0.0, -16.0 + i * 8.0, default_height_);
                uavs_[i].has_state = true;
            }
        }
    }

    void loadActorParams()
    {
        std::vector<double> actor_weights;
        XmlRpc::XmlRpcValue w;
        if(nh_.getParam("actor_weights", w) && w.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < w.size(); ++i)
            {
                actor_weights.push_back(xmlToDouble(w[i]));
            }
        }
        actor_.setWeights(actor_weights);

        std::vector<double> idle_weights;
        XmlRpc::XmlRpcValue iw;
        if(nh_.getParam("idle_weights", iw) && iw.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int i = 0; i < iw.size(); ++i)
            {
                idle_weights.push_back(xmlToDouble(iw[i]));
            }
        }
        actor_.setIdleWeights(idle_weights);
        actor_.setMaxDistance(max_distance_);
        actor_.setObstacleRiskEnabled(enable_obstacle_risk_in_assignment_);
        actor_.setObstacleRiskWeight(obstacle_risk_weight_);
        actor_.setObstacleRiskSafeMargin(obstacle_risk_safe_margin_);
        actor_.setRiskIncludeDangerZone(risk_include_danger_zone_);
        actor_.setRiskSampleNum(risk_sample_num_);
    }

    void loadAStarParams()
    {
        nh_.param<double>("astar/x_min", astar_params_.x_min, -20.0);
        nh_.param<double>("astar/x_max", astar_params_.x_max, 80.0);
        nh_.param<double>("astar/y_min", astar_params_.y_min, -35.0);
        nh_.param<double>("astar/y_max", astar_params_.y_max, 35.0);
        nh_.param<double>("astar/resolution", astar_params_.resolution, 0.5);
        nh_.param<double>("astar/obstacle_inflation", astar_params_.obstacle_inflation, 1.2);
        nh_.param<bool>("astar/avoid_danger_zone", astar_params_.avoid_danger_zone, false);
        nh_.param<bool>("astar/allow_diagonal", astar_params_.allow_diagonal, true);
        nh_.param<bool>("astar/smooth_path", astar_params_.smooth_path, true);
        nh_.param<double>("astar/path_z_offset", astar_params_.path_z_offset, 0.0);
        nh_.param<bool>("astar/use_sensor_obstacles", astar_params_.use_sensor_obstacles, true);
        nh_.param<double>("astar/sensor_obstacle_inflation", astar_params_.sensor_obstacle_inflation, 1.0);
    }



    static std::string topicFromTemplate(const std::string& templ, int id)
    {
        std::string out = templ;
        const std::string id_str = std::to_string(id);
        const size_t pos1 = out.find("%d");
        if(pos1 != std::string::npos)
        {
            out.replace(pos1, 2, id_str);
            return out;
        }
        const size_t pos2 = out.find("{id}");
        if(pos2 != std::string::npos)
        {
            out.replace(pos2, 4, id_str);
            return out;
        }
        return out;
    }

    static double yawFromQuaternion(double w, double x, double y, double z)
    {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }


    void initRosIO()
    {
        for(int i = 1; i <= swarm_num_uav_ && i <= MAX_UAV_NUM; ++i)
        {
            const std::string prefix = "/uav" + std::to_string(i);
            drone_state_sub_[i] = nh_.subscribe<prometheus_msgs::DroneState>(
                prefix + "/prometheus/drone_state", 10,
                boost::bind(&SwarmTaskAssignmentMappoNode::droneStateCb, this, _1, i));
            command_pub_[i] = nh_.advertise<prometheus_msgs::SwarmCommand>(
                prefix + "/prometheus/swarm_command", 10);
            path_pub_[i] = nh_.advertise<nav_msgs::Path>("uav" + std::to_string(i) + "_astar_path", 1, true);

            if(sensor_map_params_.enabled && sensor_map_params_.subscribe_pointcloud)
            {
                const std::string topic = topicFromTemplate(pointcloud_topic_template_, i);
                pointcloud_sub_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(
                    topic, 2, boost::bind(&SwarmTaskAssignmentMappoNode::pointCloudCb, this, _1, i));
                ROS_INFO("[%s] subscribe point cloud: %s", NODE_NAME, topic.c_str());
            }
            if(sensor_map_params_.enabled && sensor_map_params_.subscribe_laserscan)
            {
                const std::string topic = topicFromTemplate(laserscan_topic_template_, i);
                laserscan_sub_[i] = nh_.subscribe<sensor_msgs::LaserScan>(
                    topic, 2, boost::bind(&SwarmTaskAssignmentMappoNode::laserScanCb, this, _1, i));
                ROS_INFO("[%s] subscribe laser scan: %s", NODE_NAME, topic.c_str());
            }
        }

        gazebo_model_state_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 5,
                                                                           &SwarmTaskAssignmentMappoNode::gazeboModelStatesCb, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("assignment_markers", 1, true);
        uav_model_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("uav_model_markers", 1, true);
        result_text_pub_ = nh_.advertise<std_msgs::String>("assignment_result_text", 1, true);
        gazebo_spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        gazebo_delete_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    }

    void droneStateCb(const prometheus_msgs::DroneState::ConstPtr& msg, int id)
    {
        if(id < 1 || id > swarm_num_uav_)
        {
            return;
        }

        // This state is in the MAVROS/PX4 local frame. It must be used for
        // actual command dispatch, otherwise Gazebo-world coordinates can make
        // the preflight/takeoff state machine skip or send unreachable setpoints.
        UavInfo& ctrl = control_uavs_[id - 1];
        ctrl.position = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
        ctrl.velocity = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        ctrl.yaw = yawFromQuaternion(msg->attitude_q.w, msg->attitude_q.x, msg->attitude_q.y, msg->attitude_q.z);
        ctrl.has_state = true;

        // If Gazebo model states are not used for assignment, fall back to DroneState.
        if(!(use_gazebo_model_states_for_assignment_ || use_gazebo_model_states_for_rviz_uav_))
        {
            UavInfo& uav = uavs_[id - 1];
            uav.position = ctrl.position;
            uav.velocity = ctrl.velocity;
            uav.yaw = ctrl.yaw;
            uav.has_state = true;
        }
    }

    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg, int id)
    {
        if(id < 1 || id > swarm_num_uav_)
        {
            return;
        }
        sensor_map_.addPointCloud(msg, uavs_[id - 1].position, uavs_[id - 1].yaw, ros::Time::now());
    }

    void laserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg, int id)
    {
        if(id < 1 || id > swarm_num_uav_)
        {
            return;
        }
        sensor_map_.addLaserScan(msg, uavs_[id - 1].position, uavs_[id - 1].yaw, ros::Time::now());
    }

    void gazeboModelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        gazebo_model_pose_map_.clear();
        for(size_t i = 0; i < msg->name.size(); ++i)
        {
            gazebo_model_pose_map_[msg->name[i]] = msg->pose[i];
        }

        if(use_gazebo_model_states_for_assignment_ || use_gazebo_model_states_for_rviz_uav_)
        {
            for(int i = 0; i < swarm_num_uav_; ++i)
            {
                const std::string name = uav_model_names_[i];
                std::map<std::string, geometry_msgs::Pose>::const_iterator it = gazebo_model_pose_map_.find(name);
                if(it != gazebo_model_pose_map_.end())
                {
                    uavs_[i].position = Eigen::Vector3d(it->second.position.x,
                                                        it->second.position.y,
                                                        std::max(default_height_, it->second.position.z + default_height_));
                    uavs_[i].yaw = yawFromQuaternion(it->second.orientation.w, it->second.orientation.x,
                                                     it->second.orientation.y, it->second.orientation.z);
                    uavs_[i].has_state = true;
                }
            }
        }
    }

    bool allRequiredStatesReady() const
    {
        if(use_demo_positions_)
        {
            return true;
        }
        for(int i = 0; i < swarm_num_uav_; ++i)
        {
            if(!uavs_[i].has_state)
            {
                return false;
            }
            if(wait_for_gazebo_model_states_ && use_gazebo_model_states_for_assignment_)
            {
                if(gazebo_model_pose_map_.find(uav_model_names_[i]) == gazebo_model_pose_map_.end())
                {
                    return false;
                }
            }
            if(auto_dispatch_commands_ && !control_uavs_[i].has_state)
            {
                return false;
            }
        }
        return true;
    }

    void assignmentTimerCb(const ros::TimerEvent&)
    {
        if(run_once_ && has_run_once_)
        {
            return;
        }
        if(!allRequiredStatesReady())
        {
            ROS_WARN_THROTTLE(2.0, "[%s] Waiting for UAV states. If using Gazebo, check /gazebo/model_states contains uav1~uav5.", NODE_NAME);
            return;
        }

        const ros::Time now = ros::Time::now();
        std::vector<Eigen::Vector3d> sensor_obstacle_points = sensor_map_.getObstaclePoints(now);
        if(wait_for_sensor_map_before_planning_ && sensor_map_params_.enabled && sensor_obstacle_points.empty())
        {
            ROS_WARN_THROTTLE(2.0, "[%s] Waiting for sensor obstacle map. Check pointcloud/laserscan topics or set wait_for_sensor_map_before_planning=false.", NODE_NAME);
            return;
        }

        actor_.setObstacles(use_param_obstacles_for_assignment_ ? obstacles_ : std::vector<StaticObstacle>());
        actor_.setSensorObstaclePoints(sensor_obstacle_points);
        solver_.setActor(actor_);
        astar_planner_.setObstacles(use_param_obstacles_for_astar_ ? obstacles_ : std::vector<StaticObstacle>());
        astar_planner_.setSensorObstaclePoints(sensor_obstacle_points);

        ros::Time t0 = ros::Time::now();
        AssignmentMetrics metrics;
        std::vector<int> unassigned_tasks;
        std::vector<int> idle_uavs;
        std::vector<AssignmentResult> results = solver_.solve(uavs_, tasks_, metrics, unassigned_tasks, idle_uavs);
        std::vector<PathPlan> path_plans = planPaths(results);
        latest_path_plans_ = path_plans;
        have_latest_paths_ = !latest_path_plans_.empty();
        if(auto_dispatch_commands_ && path_dispatch_params_.enabled)
        {
            if(have_latest_paths_)
            {
                if(!dispatch_started_ || dispatch_preflight_only_active_ || replan_while_dispatching_)
                {
                    std::vector<PathPlan> dispatch_plans = convertWorldPlansToLocalDispatchPlans(latest_path_plans_, results);
                    path_dispatcher_.reset(dispatch_plans);
                    dispatch_started_ = true;
                    dispatch_preflight_only_active_ = false;
                    ROS_INFO("[%s] Path dispatcher reset with A* paths. Visualization paths are in world frame; dispatch commands are in PX4 local frame.", NODE_NAME);
                }
            }
            else if(dispatch_allow_preflight_without_path_ && !dispatch_started_)
            {
                std::vector<PathPlan> preflight_plans = makePreflightOnlyPlans();
                if(!preflight_plans.empty())
                {
                    path_dispatcher_.reset(preflight_plans);
                    dispatch_started_ = true;
                    dispatch_preflight_only_active_ = true;
                    ROS_WARN("[%s] No A* path is available yet. Start preflight-only dispatch so UAVs can enter OFFBOARD/takeoff; real A* paths will replace it once available.", NODE_NAME);
                }
                else
                {
                    ROS_WARN_THROTTLE(2.0, "[%s] auto_dispatch=true but no A* path and no valid /prometheus/drone_state for preflight-only dispatch.", NODE_NAME);
                }
            }
            else if(!have_latest_paths_)
            {
                ROS_WARN_THROTTLE(2.0, "[%s] auto_dispatch=true but no A* success path. No path-following command will be sent until A* succeeds.", NODE_NAME);
            }
        }
        metrics.compute_time_ms = (ros::Time::now() - t0).toSec() * 1000.0;

        if(enable_rviz_visualization_)
        {
            publishVisualization(results, path_plans, unassigned_tasks, idle_uavs);
            publishPaths(path_plans);
        }
        if(enable_gazebo_visualization_)
        {
            publishGazeboVisualization(results, path_plans);
        }
        publishResultText(results, path_plans, metrics, unassigned_tasks, idle_uavs);
        printResult(results, path_plans, metrics, unassigned_tasks, idle_uavs);

        if(auto_dispatch_commands_ && !path_dispatch_params_.enabled)
        {
            ROS_WARN_THROTTLE(3.0, "[%s] auto_dispatch_commands=true, but path_dispatch/enabled=false. No command is published.", NODE_NAME);
        }
        has_run_once_ = true;
    }

    std::vector<PathPlan> makePreflightOnlyPlans() const
    {
        std::vector<PathPlan> plans;
        for(int id = 1; id <= swarm_num_uav_; ++id)
        {
            if(id < 1 || id > static_cast<int>(control_uavs_.size()) || !control_uavs_[id - 1].has_state)
            {
                continue;
            }
            const Eigen::Vector3d cur = control_uavs_[id - 1].position;
            Eigen::Vector3d hover = cur;
            hover.z() = std::max(cur.z(), path_dispatch_params_.takeoff_reach_height);

            PathPlan p;
            p.uav_id = id;
            p.task_id = 0;
            p.success = true;
            p.used_fallback_straight_line = false;
            p.planner_name = "preflight_only";
            p.message = "no_astar_path_yet_only_offboard_takeoff_hold";
            p.length = std::fabs(hover.z() - cur.z());
            p.waypoints.push_back(cur);
            p.waypoints.push_back(hover);
            plans.push_back(p);
        }
        return plans;
    }

    std::vector<PathPlan> convertWorldPlansToLocalDispatchPlans(const std::vector<PathPlan>& world_plans,
                                                                 const std::vector<AssignmentResult>& results) const
    {
        std::map<int, Eigen::Vector3d> world_start_by_uav;
        for(size_t i = 0; i < results.size(); ++i)
        {
            world_start_by_uav[results[i].uav_id] = results[i].uav_position;
        }

        std::vector<PathPlan> local_plans = world_plans;
        for(size_t i = 0; i < local_plans.size(); ++i)
        {
            PathPlan& plan = local_plans[i];
            if(plan.uav_id < 1 || plan.uav_id > swarm_num_uav_)
            {
                continue;
            }
            std::map<int, Eigen::Vector3d>::const_iterator it = world_start_by_uav.find(plan.uav_id);
            if(it == world_start_by_uav.end())
            {
                continue;
            }

            const Eigen::Vector3d world_start = it->second;
            const Eigen::Vector3d local_start = control_uavs_[plan.uav_id - 1].position;
            for(size_t k = 0; k < plan.waypoints.size(); ++k)
            {
                const Eigen::Vector3d w = plan.waypoints[k];
                Eigen::Vector3d local;
                // x/y are converted from Gazebo world to each PX4 local frame.
                local.x() = local_start.x() + (w.x() - world_start.x());
                local.y() = local_start.y() + (w.y() - world_start.y());
                // z is kept as the desired flight height in the PX4 local ENU frame.
                local.z() = std::max(0.2, w.z());
                plan.waypoints[k] = local;
            }
            plan.message += " | dispatch_frame=px4_local";
        }
        return local_plans;
    }

    std::vector<PathPlan> planPaths(const std::vector<AssignmentResult>& results) const
    {
        std::vector<PathPlan> plans;
        for(size_t i = 0; i < results.size(); ++i)
        {
            const AssignmentResult& r = results[i];
            PathPlan p;
            if(enable_astar_path_planning_)
            {
                p = astar_planner_.plan(r.uav_id, r.task_id, r.uav_position, r.task_position);
            }
            else
            {
                p.uav_id = r.uav_id;
                p.task_id = r.task_id;
                p.success = false;
                p.used_fallback_straight_line = false;
                p.planner_name = "A*_disabled";
                p.message = "astar_disabled_no_demo_path";
                p.length = 0.0;
            }
            if(!publish_only_astar_success_path_ || p.success)
            {
                plans.push_back(p);
            }
        }
        return plans;
    }

    nav_msgs::Path toPathMsg(const PathPlan& plan) const
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = frame_id_;
        for(size_t i = 0; i < plan.waypoints.size(); ++i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose = toPoseMsg(plan.waypoints[i]);
            path.poses.push_back(pose);
        }
        return path;
    }

    void publishPaths(const std::vector<PathPlan>& plans)
    {
        for(size_t i = 0; i < plans.size(); ++i)
        {
            if(plans[i].success && plans[i].uav_id >= 1 && plans[i].uav_id <= swarm_num_uav_)
            {
                path_pub_[plans[i].uav_id].publish(toPathMsg(plans[i]));
            }
        }
    }

    void pathCommandTimerCb(const ros::TimerEvent&)
    {
        if(!auto_dispatch_commands_ || !path_dispatch_params_.enabled)
        {
            return;
        }
        if(!dispatch_started_)
        {
            ROS_WARN_THROTTLE(2.0, "[%s] auto_dispatch=true, but dispatcher has not started yet. Check assignment output and A* path success.", NODE_NAME);
            return;
        }

        const ros::Time now = ros::Time::now();
        for(int id = 1; id <= swarm_num_uav_; ++id)
        {
            if(!path_dispatcher_.hasActivePath(id))
            {
                continue;
            }
            if(id < 1 || id > swarm_num_uav_ || !control_uavs_[id - 1].has_state)
            {
                continue;
            }

            const UavInfo& ctrl_uav = control_uavs_[id - 1];
            prometheus_msgs::SwarmCommand cmd;

            if(path_dispatcher_.makeCommand(id, ctrl_uav, command_id_[id], cmd))
            {
                // Do NOT apply the sensor emergency stop during OFFBOARD preparation
                // or Takeoff.  The 2D lidar may see the UAV body / landing gear /
                // near-field points around the takeoff location; if we check before
                // generating the preflight command, the UAV will stay in Hold forever.
                // The stop is only meaningful while following real A* path points.
                const bool is_path_follow_cmd = (cmd.source == "/prometheus_task_allocate/sensor_astar_path_follow");
                if(enable_sensor_stop_failsafe_ && is_path_follow_cmd)
                {
                    const double min_dist = sensor_map_.minDistance2D(ctrl_uav.position, now);
                    if(min_dist < sensor_stop_distance_)
                    {
                        prometheus_msgs::SwarmCommand hold_cmd = path_dispatcher_.makeHoldCommand(
                            id, ctrl_uav, command_id_[id], "/prometheus_task_allocate/sensor_stop_hold");
                        command_pub_[id].publish(hold_cmd);
                        ROS_WARN_THROTTLE(1.0, "[%s] UAV%d holds during path following: nearest valid sensor obstacle %.2f m < %.2f m.",
                                          NODE_NAME, id, min_dist, sensor_stop_distance_);
                        continue;
                    }
                }

                command_pub_[id].publish(cmd);
                ROS_INFO_THROTTLE(1.0, "[%s] Dispatch UAV%d cmd: mode=%d move_mode=%d pos=[%.2f %.2f %.2f] source=%s",
                                  NODE_NAME, id, cmd.Mode, cmd.Move_mode,
                                  cmd.position_ref[0], cmd.position_ref[1], cmd.position_ref[2], cmd.source.c_str());
            }
        }
    }

    void taskColor(int type, double& r, double& g, double& b) const
    {
        if(type == ATTACK) { r = 1.0; g = 0.15; b = 0.10; return; }
        if(type == RECON)  { r = 0.10; g = 0.45; b = 1.00; return; }
        if(type == JAM)    { r = 0.70; g = 0.20; b = 1.00; return; }
        r = 1.0; g = 1.0; b = 1.0;
    }

    void pathColor(int uav_id, double& r, double& g, double& b) const
    {
        static const double colors[5][3] = {
            {0.1, 0.8, 0.1}, {1.0, 0.6, 0.0}, {0.2, 0.8, 1.0}, {1.0, 0.2, 0.8}, {0.8, 1.0, 0.2}
        };
        const int idx = std::max(0, std::min(4, uav_id - 1));
        r = colors[idx][0]; g = colors[idx][1]; b = colors[idx][2];
    }

    void publishVisualization(const std::vector<AssignmentResult>& results,
                              const std::vector<PathPlan>& path_plans,
                              const std::vector<int>& unassigned_tasks,
                              const std::vector<int>& idle_uavs)
    {
        visualization_msgs::MarkerArray array;
        int id = 0;

        // Clear old markers.
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        array.markers.push_back(clear);

        // Tasks.
        for(size_t i = 0; i < tasks_.size(); ++i)
        {
            const TaskInfo& t = tasks_[i];
            double r = 1.0, g = 1.0, b = 1.0;
            taskColor(t.type, r, g, b);
            visualization_msgs::Marker marker = makeBasicMarker(frame_id_, "tasks", id++, visualization_msgs::Marker::SPHERE, t.position);
            marker.scale.x = 1.2; marker.scale.y = 1.2; marker.scale.z = 1.2;
            setMarkerColor(marker, r, g, b, 0.95);
            array.markers.push_back(marker);

            visualization_msgs::Marker text = makeBasicMarker(frame_id_, "task_text", id++, visualization_msgs::Marker::TEXT_VIEW_FACING,
                                                              t.position + Eigen::Vector3d(0.0, 0.0, 1.2));
            text.scale.z = 0.9;
            setMarkerColor(text, 1.0, 1.0, 1.0, 1.0);
            std::stringstream ss;
            ss << "T" << t.id << " " << taskTypeToChinese(t.type) << " P=" << t.priority;
            text.text = ss.str();
            array.markers.push_back(text);
        }

        // Obstacles.
        if(enable_obstacle_visualization_)
        {
            for(size_t i = 0; i < obstacles_.size(); ++i)
            {
                const StaticObstacle& obs = obstacles_[i];
                const int type = (obs.type == 0) ? visualization_msgs::Marker::CUBE : visualization_msgs::Marker::CYLINDER;
                visualization_msgs::Marker marker = makeBasicMarker(frame_id_, "obstacles", id++, type, obs.position);
                if(obs.type == 0)
                {
                    marker.scale.x = obs.size.x(); marker.scale.y = obs.size.y(); marker.scale.z = obs.size.z();
                }
                else
                {
                    marker.scale.x = obs.size.x() * 2.0; marker.scale.y = obs.size.x() * 2.0; marker.scale.z = obs.size.z();
                }
                setMarkerColor(marker, obs.r, obs.g, obs.b, obs.a);
                array.markers.push_back(marker);
            }
        }

        // Sensor obstacle points used by MAPPO risk and A*.
        if(sensor_map_params_.enabled)
        {
            std::vector<Eigen::Vector3d> sensor_pts = sensor_map_.getObstaclePoints(ros::Time::now());
            if(!sensor_pts.empty())
            {
                visualization_msgs::Marker pts = makeBasicMarker(frame_id_, "sensor_obstacles", id++, visualization_msgs::Marker::SPHERE_LIST, Eigen::Vector3d::Zero());
                pts.scale.x = 0.25; pts.scale.y = 0.25; pts.scale.z = 0.25;
                setMarkerColor(pts, 1.0, 0.9, 0.1, 0.85);
                for(size_t k = 0; k < sensor_pts.size(); ++k)
                {
                    pts.points.push_back(toPointMsg(sensor_pts[k]));
                }
                array.markers.push_back(pts);
            }
        }

        // Assignment lines and text.
        for(size_t i = 0; i < results.size(); ++i)
        {
            const AssignmentResult& rlt = results[i];
            double r = 0.1, g = 1.0, b = 0.1;
            pathColor(rlt.uav_id, r, g, b);

            visualization_msgs::Marker line = makeBasicMarker(frame_id_, "assignment_line", id++, visualization_msgs::Marker::LINE_STRIP, Eigen::Vector3d::Zero());
            line.scale.x = 0.15;
            setMarkerColor(line, r, g, b, 0.95);
            line.points.push_back(toPointMsg(rlt.uav_position));
            line.points.push_back(toPointMsg(rlt.task_position));
            array.markers.push_back(line);

            const Eigen::Vector3d mid = 0.5 * (rlt.uav_position + rlt.task_position) + Eigen::Vector3d(0.0, 0.0, 1.0);
            visualization_msgs::Marker text = makeBasicMarker(frame_id_, "assignment_text", id++, visualization_msgs::Marker::TEXT_VIEW_FACING, mid);
            text.scale.z = 0.8;
            setMarkerColor(text, r, g, b, 1.0);
            std::stringstream ss;
            ss << "UAV" << rlt.uav_id << " -> T" << rlt.task_id << "\nrisk=" << std::fixed << std::setprecision(2) << rlt.obstacle_risk;
            text.text = ss.str();
            array.markers.push_back(text);
        }

        // A* path waypoint spheres in RViz.
        for(size_t i = 0; i < path_plans.size(); ++i)
        {
            const PathPlan& plan = path_plans[i];
            double r = 0.1, g = 1.0, b = 0.1;
            pathColor(plan.uav_id, r, g, b);
            for(size_t k = 0; k < plan.waypoints.size(); ++k)
            {
                visualization_msgs::Marker marker = makeBasicMarker(frame_id_, "astar_waypoints", id++, visualization_msgs::Marker::SPHERE, plan.waypoints[k]);
                marker.scale.x = 0.35; marker.scale.y = 0.35; marker.scale.z = 0.35;
                setMarkerColor(marker, r, g, b, plan.used_fallback_straight_line ? 0.35 : 0.85);
                array.markers.push_back(marker);
            }
        }

        // Summary text.
        visualization_msgs::Marker summary = makeBasicMarker(frame_id_, "summary", id++, visualization_msgs::Marker::TEXT_VIEW_FACING,
                                                             Eigen::Vector3d(3.0, 28.0, 7.0));
        summary.scale.z = 1.0;
        setMarkerColor(summary, 1.0, 1.0, 1.0, 1.0);
        std::stringstream ss;
        ss << "MAPPO顺序决策 + 传感器障碍物风险 + A*实际路径"
           << "\nidle_uavs: " << intVectorToString(idle_uavs)
           << "  unassigned_tasks: " << intVectorToString(unassigned_tasks);
        summary.text = ss.str();
        array.markers.push_back(summary);

        marker_pub_.publish(array);
    }

    void uavModelMarkerTimerCb(const ros::TimerEvent&)
    {
        if(!enable_rviz_visualization_ || !visualize_uav_as_mesh_)
        {
            return;
        }

        visualization_msgs::MarkerArray array;
        int marker_id = 0;
        for(int i = 0; i < swarm_num_uav_; ++i)
        {
            Eigen::Vector3d pos = uavs_[i].position;
            geometry_msgs::Pose pose;
            pose.position = toPointMsg(pos + Eigen::Vector3d(0.0, 0.0, rviz_uav_mesh_z_offset_));
            pose.orientation.w = 1.0;

            if(use_gazebo_model_states_for_rviz_uav_)
            {
                std::map<std::string, geometry_msgs::Pose>::const_iterator it = gazebo_model_pose_map_.find(uav_model_names_[i]);
                if(it != gazebo_model_pose_map_.end())
                {
                    pose = it->second;
                    pose.position.z += rviz_uav_mesh_z_offset_;
                }
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "uav_mesh";
            marker.id = marker_id++;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = rviz_uav_mesh_scale_;
            marker.scale.y = rviz_uav_mesh_scale_;
            marker.scale.z = rviz_uav_mesh_scale_;
            marker.lifetime = ros::Duration(0.3);

            if(!uav_mesh_resources_[i].empty())
            {
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.mesh_resource = uav_mesh_resources_[i];
                marker.mesh_use_embedded_materials = false;
                setMarkerColor(marker, 0.85, 0.85, 0.85, 1.0);
            }
            else
            {
                marker.type = visualization_msgs::Marker::ARROW;
                marker.scale.x = 1.5; marker.scale.y = 0.25; marker.scale.z = 0.25;
                setMarkerColor(marker, 0.1, 0.9, 0.1, 1.0);
            }
            array.markers.push_back(marker);
        }
        uav_model_marker_pub_.publish(array);
    }

    void deleteGazeboModel(const std::string& name)
    {
        gazebo_msgs::DeleteModel srv;
        srv.request.model_name = name;
        gazebo_delete_client_.call(srv);
    }

    void spawnGazeboModel(const std::string& name, const std::string& sdf, const Eigen::Vector3d& pos)
    {
        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = name;
        srv.request.model_xml = sdf;
        srv.request.robot_namespace = "";
        srv.request.initial_pose = toPoseMsg(pos);
        srv.request.reference_frame = "world";
        if(gazebo_spawn_client_.call(srv))
        {
            gazebo_spawned_model_names_.push_back(name);
        }
    }

    void clearGazeboVisualization()
    {
        for(size_t i = 0; i < gazebo_spawned_model_names_.size(); ++i)
        {
            deleteGazeboModel(gazebo_spawned_model_names_[i]);
        }
        gazebo_spawned_model_names_.clear();
        gazebo_models_spawned_ = false;
    }

    void publishGazeboVisualization(const std::vector<AssignmentResult>& results,
                                    const std::vector<PathPlan>& path_plans)
    {
        if(gazebo_models_spawned_ && !gazebo_respawn_each_assignment_)
        {
            return;
        }
        if(gazebo_respawn_each_assignment_)
        {
            clearGazeboVisualization();
        }

        if(!gazebo_spawn_client_.waitForExistence(ros::Duration(2.0)))
        {
            ROS_WARN_THROTTLE(3.0, "[%s] /gazebo/spawn_sdf_model is not available yet.", NODE_NAME);
            return;
        }

        // Task point spheres.
        for(size_t i = 0; i < tasks_.size(); ++i)
        {
            double r = 1.0, g = 1.0, b = 1.0;
            taskColor(tasks_[i].type, r, g, b);
            const std::string sdf = makeGazeboSphereSdf(gazebo_task_radius_, r, g, b, 0.95, gazebo_visual_collision_);
            spawnGazeboModel("ta_task_T" + std::to_string(tasks_[i].id), sdf, tasks_[i].position);
        }

        // Obstacles.
        if(enable_obstacle_visualization_)
        {
            for(size_t i = 0; i < obstacles_.size(); ++i)
            {
                const StaticObstacle& obs = obstacles_[i];
                std::string sdf;
                if(obs.type == 0)
                {
                    sdf = makeGazeboBoxSdf(obs.size, obs.r, obs.g, obs.b, obs.a, gazebo_obstacle_collision_);
                }
                else
                {
                    sdf = makeGazeboCylinderSdf(obs.size.x(), obs.size.z(), obs.r, obs.g, obs.b, obs.a,
                                                obs.isDangerZone() ? false : gazebo_obstacle_collision_);
                }
                spawnGazeboModel("ta_obstacle_" + std::to_string(obs.id), sdf, obs.position);
            }
        }

        // A* path visualization spheres. They have no collision.
        for(size_t i = 0; i < path_plans.size(); ++i)
        {
            const PathPlan& plan = path_plans[i];
            double r = 0.1, g = 1.0, b = 0.1;
            pathColor(plan.uav_id, r, g, b);
            const std::string sdf = makeGazeboSphereSdf(gazebo_path_marker_radius_, r, g, b,
                                                        plan.used_fallback_straight_line ? 0.35 : 0.85, false);
            const int n = static_cast<int>(plan.waypoints.size());
            const int start_skip = std::max(0, gazebo_path_skip_start_points_);
            const int end_skip = std::max(0, gazebo_path_skip_goal_points_);
            for(int k = start_skip; k < n - end_skip; ++k)
            {
                Eigen::Vector3d p = plan.waypoints[k] + Eigen::Vector3d(0.0, 0.0, gazebo_path_marker_z_offset_);
                spawnGazeboModel("ta_astar_u" + std::to_string(plan.uav_id) + "_p" + std::to_string(k), sdf, p);
            }
        }
        gazebo_models_spawned_ = true;
    }

    std::string resultText(const std::vector<AssignmentResult>& results,
                           const std::vector<PathPlan>& path_plans,
                           const AssignmentMetrics& metrics,
                           const std::vector<int>& unassigned_tasks,
                           const std::vector<int>& idle_uavs) const
    {
        std::stringstream ss;
        ss << "================ MAPPO Sensor Risk + A* Task Assignment ================\n";
        ss << "sensor_obstacle_points: " << sensor_map_.getObstaclePoints(ros::Time::now()).size()
           << "  param_obstacles_for_assignment: " << (use_param_obstacles_for_assignment_ ? "true" : "false")
           << "  param_obstacles_for_astar: " << (use_param_obstacles_for_astar_ ? "true" : "false")
           << "  dispatch_started: " << (dispatch_started_ ? "true" : "false")
           << "  dispatch_frame: px4_local" << "\n";
        for(size_t i = 0; i < results.size(); ++i)
        {
            const AssignmentResult& r = results[i];
            double path_len = 0.0;
            bool astar_ok = false;
            bool fallback = false;
            for(size_t k = 0; k < path_plans.size(); ++k)
            {
                if(path_plans[k].uav_id == r.uav_id && path_plans[k].task_id == r.task_id)
                {
                    path_len = path_plans[k].length;
                    astar_ok = path_plans[k].success;
                    fallback = path_plans[k].used_fallback_straight_line;
                    break;
                }
            }
            ss << "UAV" << r.uav_id << " -> T" << r.task_id << " [" << taskTypeToChinese(r.task_type) << "]"
               << " score=" << std::fixed << std::setprecision(2) << r.score
               << " dist=" << r.distance << " m"
               << " risk=" << r.obstacle_risk
               << " astar_len=" << path_len << " m"
               << " astar=" << (astar_ok ? "success" : "fail") << "\n";
        }
        ss << "assigned: " << metrics.assigned_num << "/" << metrics.total_task
           << "  success_rate: " << std::fixed << std::setprecision(2) << metrics.success_rate * 100.0 << "%"
           << "  total_distance: " << metrics.total_distance << " m"
           << "  avg_distance: " << metrics.avg_distance << " m"
           << "  total_risk: " << metrics.total_obstacle_risk
           << "  avg_risk: " << metrics.avg_obstacle_risk
           << "  compute_time: " << metrics.compute_time_ms << " ms\n";
        ss << "unassigned_tasks: " << intVectorToString(unassigned_tasks) << "\n";
        ss << "idle_uavs: " << intVectorToString(idle_uavs) << "\n";
        ss << "RViz MarkerArray topic: /swarm_task_assignment_mappo/assignment_markers\n";
        ss << "RViz A* Path topics: /swarm_task_assignment_mappo/uavX_astar_path\n";
        ss << "path_dispatch: " << ((auto_dispatch_commands_ && path_dispatch_params_.enabled) ? "enabled" : "disabled")
           << "  sensor_stop_failsafe: " << (enable_sensor_stop_failsafe_ ? "enabled" : "disabled") << "\n";
        ss << "Gazebo markers: " << (enable_gazebo_visualization_ ? "enabled" : "disabled") << "\n";
        ss << "==================================================================";
        return ss.str();
    }

    void publishResultText(const std::vector<AssignmentResult>& results,
                           const std::vector<PathPlan>& path_plans,
                           const AssignmentMetrics& metrics,
                           const std::vector<int>& unassigned_tasks,
                           const std::vector<int>& idle_uavs)
    {
        std_msgs::String msg;
        msg.data = resultText(results, path_plans, metrics, unassigned_tasks, idle_uavs);
        result_text_pub_.publish(msg);
    }

    void printResult(const std::vector<AssignmentResult>& results,
                     const std::vector<PathPlan>& path_plans,
                     const AssignmentMetrics& metrics,
                     const std::vector<int>& unassigned_tasks,
                     const std::vector<int>& idle_uavs) const
    {
        ROS_INFO_STREAM("\n" << resultText(results, path_plans, metrics, unassigned_tasks, idle_uavs));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    SwarmTaskAssignmentMappoNode node;
    ros::spin();
    return 0;
}
