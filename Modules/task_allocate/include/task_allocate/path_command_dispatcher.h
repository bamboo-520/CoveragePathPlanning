#ifndef TASK_ALLOCATE_PATH_COMMAND_DISPATCHER_H
#define TASK_ALLOCATE_PATH_COMMAND_DISPATCHER_H

#include <map>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <prometheus_msgs/SwarmCommand.h>

#include "task_allocate/common_types.h"

namespace task_allocate
{

struct PathDispatchParams
{
    bool enabled;
    double waypoint_reach_threshold;
    double command_period;
    double yaw_ref;
    int min_waypoint_index;

    // When true, commands are sent in this order:
    // Idle(yaw_ref=999) -> Takeoff -> A* waypoint following -> Hold.
    // This matches the existing prometheus_swarm_control/swarm_controller logic.
    bool enable_preflight_sequence;
    double offboard_prepare_time;
    double takeoff_reach_height;
    double takeoff_timeout;

    PathDispatchParams()
        : enabled(false),
          waypoint_reach_threshold(0.8),
          command_period(0.2),
          yaw_ref(0.0),
          min_waypoint_index(1),
          enable_preflight_sequence(true),
          offboard_prepare_time(3.0),
          takeoff_reach_height(1.5),
          takeoff_timeout(8.0)
    {
    }
};

class PathCommandDispatcher
{
public:
    PathCommandDispatcher();

    void setParams(const PathDispatchParams& params);
    void reset(const std::vector<PathPlan>& plans);
    bool makeCommand(int uav_id,
                     const UavInfo& uav,
                     int& command_id,
                     prometheus_msgs::SwarmCommand& cmd);
    prometheus_msgs::SwarmCommand makeHoldCommand(int uav_id,
                                                  const UavInfo& uav,
                                                  int& command_id,
                                                  const std::string& source) const;
    prometheus_msgs::SwarmCommand makeIdleOffboardCommand(int uav_id,
                                                          int& command_id,
                                                          const std::string& source) const;
    prometheus_msgs::SwarmCommand makeTakeoffCommand(int uav_id,
                                                     int& command_id,
                                                     const std::string& source) const;
    bool hasActivePath(int uav_id) const;
    bool finished(int uav_id) const;

private:
    struct DispatchState
    {
        enum Stage
        {
            PREPARE_OFFBOARD = 0,
            TAKEOFF = 1,
            FOLLOW_PATH = 2,
            HOLD = 3
        };

        PathPlan plan;
        int next_index;
        bool finished;
        Stage stage;
        ros::Time stage_start;
        bool stage_start_valid;

        DispatchState()
            : next_index(0),
              finished(false),
              stage(PREPARE_OFFBOARD),
              stage_start_valid(false)
        {
        }
    };

private:
    PathDispatchParams params_;
    std::map<int, DispatchState> states_;
};

} // namespace task_allocate

#endif // TASK_ALLOCATE_PATH_COMMAND_DISPATCHER_H
