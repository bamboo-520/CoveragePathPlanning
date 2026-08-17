#include "task_allocate/mappo_sequential_solver.h"

#include <algorithm>
#include <sstream>

namespace task_allocate
{

MappoSequentialSolver::MappoSequentialSolver()
{
}

void MappoSequentialSolver::setActor(const LinearMappoActor& actor)
{
    actor_ = actor;
}

std::vector<AgentDecision> MappoSequentialSolver::inferAgentDecisions(const std::vector<UavInfo>& uavs,
                                                                       const std::vector<TaskInfo>& tasks) const
{
    std::vector<AgentDecision> decisions;
    std::vector<int> task_used(tasks.size(), 0);

    for(size_t i = 0; i < uavs.size(); ++i)
    {
        AgentDecision decision;
        decision.uav_id = uavs[i].id;
        decision.idle_score = actor_.calcIdleLogit(uavs[i], tasks);
        decision.task_scores.resize(tasks.size(), -1e9);

        double best_score = -1e9;
        int best_task_index = -1;
        for(size_t j = 0; j < tasks.size(); ++j)
        {
            if(task_used[j])
            {
                continue;
            }
            if(!uavs[i].available || !LinearMappoActor::hasCapability(uavs[i], tasks[j].type))
            {
                continue;
            }

            const double score = actor_.calcTaskLogit(uavs[i], tasks[j]);
            decision.task_scores[j] = score;

            if(!passesLookAheadMask(uavs, tasks, i, static_cast<int>(j), task_used))
            {
                continue;
            }

            if(score > best_score)
            {
                best_score = score;
                best_task_index = static_cast<int>(j);
            }
        }

        if(best_task_index >= 0)
        {
            decision.task_index = best_task_index;
            decision.task_id = tasks[best_task_index].id;
            decision.score = best_score;
            task_used[best_task_index] = 1;
        }
        else
        {
            decision.task_index = -1;
            decision.task_id = -1;
            decision.score = decision.idle_score;
        }
        decisions.push_back(decision);
    }
    return decisions;
}

std::vector<AssignmentResult> MappoSequentialSolver::solve(const std::vector<UavInfo>& uavs,
                                                           const std::vector<TaskInfo>& tasks,
                                                           AssignmentMetrics& metrics,
                                                           std::vector<int>& unassigned_tasks,
                                                           std::vector<int>& idle_uavs) const
{
    std::vector<int> task_used(tasks.size(), 0);
    std::vector<int> uav_used(uavs.size(), 0);
    std::vector<AssignmentResult> results;

    for(size_t i = 0; i < uavs.size(); ++i)
    {
        const UavInfo& uav = uavs[i];
        if(!uav.available)
        {
            continue;
        }

        double best_score = -1e9;
        int best_task_index = -1;

        for(size_t j = 0; j < tasks.size(); ++j)
        {
            if(task_used[j])
            {
                continue;
            }
            if(!LinearMappoActor::hasCapability(uav, tasks[j].type))
            {
                continue;
            }
            if(!passesLookAheadMask(uavs, tasks, i, static_cast<int>(j), task_used))
            {
                continue;
            }

            const double score = actor_.calcTaskLogit(uav, tasks[j]);
            if(score > best_score)
            {
                best_score = score;
                best_task_index = static_cast<int>(j);
            }
        }

        // Idle is allowed only when the action mask has no feasible task.
        if(best_task_index < 0)
        {
            continue;
        }

        const TaskInfo& task = tasks[best_task_index];
        AssignmentResult r;
        r.uav_id = uav.id;
        r.task_id = task.id;
        r.task_index = best_task_index;
        r.task_type = task.type;
        r.uav_position = uav.position;
        r.task_position = task.position;
        r.score = best_score;
        r.distance = (uav.position - task.position).norm();
        r.obstacle_risk = actor_.calcObstacleRisk(uav, task);
        results.push_back(r);

        uav_used[i] = 1;
        task_used[best_task_index] = 1;
    }

    unassigned_tasks.clear();
    idle_uavs.clear();
    for(size_t j = 0; j < tasks.size(); ++j)
    {
        if(!task_used[j])
        {
            unassigned_tasks.push_back(tasks[j].id);
        }
    }
    for(size_t i = 0; i < uavs.size(); ++i)
    {
        if(!uav_used[i])
        {
            idle_uavs.push_back(uavs[i].id);
        }
    }

    metrics.total_uav = static_cast<int>(uavs.size());
    metrics.total_task = static_cast<int>(tasks.size());
    metrics.assigned_num = static_cast<int>(results.size());
    metrics.unassigned_task_num = static_cast<int>(unassigned_tasks.size());
    metrics.idle_uav_num = static_cast<int>(idle_uavs.size());
    metrics.total_distance = 0.0;
    metrics.total_obstacle_risk = 0.0;
    for(size_t i = 0; i < results.size(); ++i)
    {
        metrics.total_distance += results[i].distance;
        metrics.total_obstacle_risk += results[i].obstacle_risk;
    }
    metrics.avg_distance = results.empty() ? 0.0 : metrics.total_distance / results.size();
    metrics.avg_obstacle_risk = results.empty() ? 0.0 : metrics.total_obstacle_risk / results.size();
    metrics.success_rate = tasks.empty() ? 0.0 : static_cast<double>(results.size()) / tasks.size();
    return results;
}

bool MappoSequentialSolver::passesLookAheadMask(const std::vector<UavInfo>& uavs,
                                                const std::vector<TaskInfo>& tasks,
                                                size_t current_uav_index,
                                                int selected_task_index,
                                                const std::vector<int>& task_used) const
{
    if(selected_task_index < 0 || selected_task_index >= static_cast<int>(tasks.size()))
    {
        return false;
    }

    std::vector<int> next_task_used = task_used;
    next_task_used[selected_task_index] = 1;

    std::vector<int> remaining_tasks;
    for(size_t j = 0; j < tasks.size(); ++j)
    {
        if(!next_task_used[j])
        {
            remaining_tasks.push_back(static_cast<int>(j));
        }
    }

    std::vector<int> remaining_uavs;
    for(size_t i = current_uav_index + 1; i < uavs.size(); ++i)
    {
        if(uavs[i].available)
        {
            remaining_uavs.push_back(static_cast<int>(i));
        }
    }

    if(remaining_tasks.empty())
    {
        return true;
    }
    if(remaining_uavs.empty())
    {
        return false;
    }

    const int assign_need = std::min(static_cast<int>(remaining_tasks.size()),
                                     static_cast<int>(remaining_uavs.size()));
    return maxFeasibleFutureAssignments(uavs, tasks, remaining_uavs, remaining_tasks) >= assign_need;
}

int MappoSequentialSolver::maxFeasibleFutureAssignments(const std::vector<UavInfo>& uavs,
                                                        const std::vector<TaskInfo>& tasks,
                                                        const std::vector<int>& remaining_uavs,
                                                        const std::vector<int>& remaining_tasks) const
{
    std::vector<int> task_taken(remaining_tasks.size(), 0);
    int best = 0;
    dfsFutureAssign(uavs, tasks, remaining_uavs, remaining_tasks, 0, task_taken, 0, best);
    return best;
}

void MappoSequentialSolver::dfsFutureAssign(const std::vector<UavInfo>& uavs,
                                            const std::vector<TaskInfo>& tasks,
                                            const std::vector<int>& remaining_uavs,
                                            const std::vector<int>& remaining_tasks,
                                            size_t uav_pos,
                                            std::vector<int>& task_taken,
                                            int assigned_count,
                                            int& best) const
{
    if(assigned_count > best)
    {
        best = assigned_count;
    }
    if(uav_pos >= remaining_uavs.size())
    {
        return;
    }
    if(assigned_count + static_cast<int>(remaining_uavs.size() - uav_pos) <= best)
    {
        return;
    }

    const int uav_index = remaining_uavs[uav_pos];

    // Future idle branch.
    dfsFutureAssign(uavs, tasks, remaining_uavs, remaining_tasks,
                    uav_pos + 1, task_taken, assigned_count, best);

    for(size_t k = 0; k < remaining_tasks.size(); ++k)
    {
        if(task_taken[k])
        {
            continue;
        }
        const int task_index = remaining_tasks[k];
        if(!LinearMappoActor::hasCapability(uavs[uav_index], tasks[task_index].type))
        {
            continue;
        }
        task_taken[k] = 1;
        dfsFutureAssign(uavs, tasks, remaining_uavs, remaining_tasks,
                        uav_pos + 1, task_taken, assigned_count + 1, best);
        task_taken[k] = 0;
    }
}

std::string intVectorToString(const std::vector<int>& v)
{
    if(v.empty())
    {
        return "none";
    }
    std::stringstream ss;
    for(size_t i = 0; i < v.size(); ++i)
    {
        ss << v[i];
        if(i + 1 < v.size())
        {
            ss << ",";
        }
    }
    return ss.str();
}

} // namespace task_allocate
