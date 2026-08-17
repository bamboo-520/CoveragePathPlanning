#include "task_allocate/path_command_dispatcher.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace task_allocate
{

PathCommandDispatcher::PathCommandDispatcher()
{
}

void PathCommandDispatcher::setParams(const PathDispatchParams& params)
{
    params_ = params;
    params_.waypoint_reach_threshold = std::max(0.1, params_.waypoint_reach_threshold);
    params_.min_waypoint_index = std::max(0, params_.min_waypoint_index);
    params_.offboard_prepare_time = std::max(0.5, params_.offboard_prepare_time);
    params_.takeoff_reach_height = std::max(0.2, params_.takeoff_reach_height);
    params_.takeoff_timeout = std::max(1.0, params_.takeoff_timeout);
}

void PathCommandDispatcher::reset(const std::vector<PathPlan>& plans)
{
    // This function is also used for online replanning.  Do not blindly clear
    // all flight stages, otherwise every new A* path will send the UAV back to
    // PREPARE_OFFBOARD/TAKEOFF and the vehicle will never follow a refreshed
    // obstacle-avoidance path stably.
    std::map<int, DispatchState> old_states = states_;
    states_.clear();

    for(size_t i = 0; i < plans.size(); ++i)
    {
        const PathPlan& p = plans[i];
        if(!p.success || p.waypoints.empty())
        {
            continue;
        }

        DispatchState st;
        st.plan = p;
        st.next_index = std::min(std::max(params_.min_waypoint_index, 0), static_cast<int>(p.waypoints.size()) - 1);
        st.finished = false;
        st.stage = params_.enable_preflight_sequence ? DispatchState::PREPARE_OFFBOARD : DispatchState::FOLLOW_PATH;
        st.stage_start_valid = false;

        std::map<int, DispatchState>::const_iterator old_it = old_states.find(p.uav_id);
        if(old_it != old_states.end())
        {
            const DispatchState& old = old_it->second;
            // If the UAV has already entered FOLLOW_PATH/HOLD, keep it in the
            // path-following stage and only replace the remaining path with the
            // newly planned one.  This is the key for sensor-based online
            // obstacle avoidance.
            if(old.stage == DispatchState::FOLLOW_PATH || old.stage == DispatchState::HOLD)
            {
                st.stage = DispatchState::FOLLOW_PATH;
                st.stage_start = old.stage_start;
                st.stage_start_valid = old.stage_start_valid;
            }
            else
            {
                // During OFFBOARD preparation or takeoff, keep the original
                // stage timing so repeated replanning does not restart the
                // preflight sequence.
                st.stage = old.stage;
                st.stage_start = old.stage_start;
                st.stage_start_valid = old.stage_start_valid;
            }
        }

        states_[p.uav_id] = st;
    }
}

bool PathCommandDispatcher::makeCommand(int uav_id,
                                        const UavInfo& uav,
                                        int& command_id,
                                        prometheus_msgs::SwarmCommand& cmd)
{
    std::map<int, DispatchState>::iterator it = states_.find(uav_id);
    if(it == states_.end())
    {
        return false;
    }

    DispatchState& st = it->second;
    if(st.finished || st.plan.waypoints.empty())
    {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if(!st.stage_start_valid)
    {
        st.stage_start = now;
        st.stage_start_valid = true;
    }

    if(params_.enable_preflight_sequence && st.stage == DispatchState::PREPARE_OFFBOARD)
    {
        const double elapsed = (now - st.stage_start).toSec();
        if(elapsed < params_.offboard_prepare_time)
        {
            cmd = makeIdleOffboardCommand(uav_id, command_id, "/prometheus_task_allocate/offboard_arm_prepare");
            return true;
        }
        st.stage = DispatchState::TAKEOFF;
        st.stage_start = now;
        st.stage_start_valid = true;
    }

    if(params_.enable_preflight_sequence && st.stage == DispatchState::TAKEOFF)
    {
        const double elapsed = (now - st.stage_start).toSec();
        if(uav.position.z() < params_.takeoff_reach_height && elapsed < params_.takeoff_timeout)
        {
            cmd = makeTakeoffCommand(uav_id, command_id, "/prometheus_task_allocate/takeoff_before_path");
            return true;
        }
        st.stage = DispatchState::FOLLOW_PATH;
        st.stage_start = now;
        st.stage_start_valid = true;
    }

    while(st.next_index < static_cast<int>(st.plan.waypoints.size()))
    {
        const Eigen::Vector3d target = st.plan.waypoints[st.next_index];
        const double dist = (uav.position - target).norm();
        if(dist > params_.waypoint_reach_threshold)
        {
            break;
        }
        st.next_index++;
    }

    if(st.next_index >= static_cast<int>(st.plan.waypoints.size()))
    {
        st.finished = true;
        st.stage = DispatchState::HOLD;
        cmd = makeHoldCommand(uav_id, uav, command_id, "/prometheus_task_allocate/path_finished_hold");
        return true;
    }

    const Eigen::Vector3d target = st.plan.waypoints[st.next_index];
    cmd.header.stamp = ros::Time::now();
    cmd.Mode = prometheus_msgs::SwarmCommand::Move;
    cmd.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
    cmd.Command_ID = ++command_id;
    cmd.source = "/prometheus_task_allocate/sensor_astar_path_follow";
    cmd.position_ref[0] = target.x();
    cmd.position_ref[1] = target.y();
    cmd.position_ref[2] = target.z();
    cmd.velocity_ref[0] = 0.0;
    cmd.velocity_ref[1] = 0.0;
    cmd.velocity_ref[2] = 0.0;
    cmd.acceleration_ref[0] = 0.0;
    cmd.acceleration_ref[1] = 0.0;
    cmd.acceleration_ref[2] = 0.0;
    cmd.yaw_ref = params_.yaw_ref;
    return true;
}

prometheus_msgs::SwarmCommand PathCommandDispatcher::makeHoldCommand(int uav_id,
                                                                     const UavInfo& uav,
                                                                     int& command_id,
                                                                     const std::string& source) const
{
    prometheus_msgs::SwarmCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.Mode = prometheus_msgs::SwarmCommand::Hold;
    cmd.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
    cmd.Command_ID = ++command_id;
    cmd.source = source;
    cmd.position_ref[0] = uav.position.x();
    cmd.position_ref[1] = uav.position.y();
    cmd.position_ref[2] = uav.position.z();
    cmd.velocity_ref[0] = 0.0;
    cmd.velocity_ref[1] = 0.0;
    cmd.velocity_ref[2] = 0.0;
    cmd.acceleration_ref[0] = 0.0;
    cmd.acceleration_ref[1] = 0.0;
    cmd.acceleration_ref[2] = 0.0;
    cmd.yaw_ref = params_.yaw_ref;
    return cmd;
}

prometheus_msgs::SwarmCommand PathCommandDispatcher::makeIdleOffboardCommand(int uav_id,
                                                                             int& command_id,
                                                                             const std::string& source) const
{
    prometheus_msgs::SwarmCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.Mode = prometheus_msgs::SwarmCommand::Idle;
    cmd.Command_ID = ++command_id;
    cmd.source = source;
    cmd.yaw_ref = 999.0; // prometheus_swarm_control uses this value to switch OFFBOARD and arm.
    return cmd;
}

prometheus_msgs::SwarmCommand PathCommandDispatcher::makeTakeoffCommand(int uav_id,
                                                                        int& command_id,
                                                                        const std::string& source) const
{
    prometheus_msgs::SwarmCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.Mode = prometheus_msgs::SwarmCommand::Takeoff;
    cmd.Command_ID = ++command_id;
    cmd.source = source;
    cmd.yaw_ref = params_.yaw_ref;
    return cmd;
}

bool PathCommandDispatcher::hasActivePath(int uav_id) const
{
    std::map<int, DispatchState>::const_iterator it = states_.find(uav_id);
    return it != states_.end() && !it->second.finished;
}

bool PathCommandDispatcher::finished(int uav_id) const
{
    std::map<int, DispatchState>::const_iterator it = states_.find(uav_id);
    return it != states_.end() && it->second.finished;
}

} // namespace task_allocate
