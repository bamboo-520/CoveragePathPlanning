#ifndef TASK_ALLOCATE_MAPPO_SEQUENTIAL_SOLVER_H
#define TASK_ALLOCATE_MAPPO_SEQUENTIAL_SOLVER_H

#include <vector>

#include "task_allocate/common_types.h"
#include "task_allocate/mappo_actor.h"

namespace task_allocate
{

struct AgentDecision
{
    int uav_id;
    int task_id;
    int task_index;
    double score;
    double idle_score;
    std::vector<double> task_scores;

    AgentDecision()
        : uav_id(0), task_id(-1), task_index(-1), score(-1e9), idle_score(-1e9)
    {
    }
};

class MappoSequentialSolver
{
public:
    MappoSequentialSolver();

    void setActor(const LinearMappoActor& actor);
    std::vector<AgentDecision> inferAgentDecisions(const std::vector<UavInfo>& uavs,
                                                    const std::vector<TaskInfo>& tasks) const;

    std::vector<AssignmentResult> solve(const std::vector<UavInfo>& uavs,
                                        const std::vector<TaskInfo>& tasks,
                                        AssignmentMetrics& metrics,
                                        std::vector<int>& unassigned_tasks,
                                        std::vector<int>& idle_uavs) const;

private:
    bool passesLookAheadMask(const std::vector<UavInfo>& uavs,
                             const std::vector<TaskInfo>& tasks,
                             size_t current_uav_index,
                             int selected_task_index,
                             const std::vector<int>& task_used) const;

    int maxFeasibleFutureAssignments(const std::vector<UavInfo>& uavs,
                                     const std::vector<TaskInfo>& tasks,
                                     const std::vector<int>& remaining_uavs,
                                     const std::vector<int>& remaining_tasks) const;

    void dfsFutureAssign(const std::vector<UavInfo>& uavs,
                         const std::vector<TaskInfo>& tasks,
                         const std::vector<int>& remaining_uavs,
                         const std::vector<int>& remaining_tasks,
                         size_t uav_pos,
                         std::vector<int>& task_taken,
                         int assigned_count,
                         int& best) const;

private:
    LinearMappoActor actor_;
};

std::string intVectorToString(const std::vector<int>& v);

} // namespace task_allocate

#endif // TASK_ALLOCATE_MAPPO_SEQUENTIAL_SOLVER_H
