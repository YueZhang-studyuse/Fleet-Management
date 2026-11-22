#include "TaskScheduler.h"
#include "const.h"
#include <numeric>

namespace {
void log_parking_traffic(const SharedEnvironment* env,
    const std::vector<DefaultPlanner::Double4>& delivery,
    const std::vector<DefaultPlanner::Double4>& parking, bool enabled)
{
    std::vector<double> vertex_traffic;
    const double mean_before = DefaultPlanner::calculate_mean_nonzero_traffic(env, delivery, vertex_traffic);
    const double delivery_total = std::accumulate(vertex_traffic.begin(), vertex_traffic.end(), 0.0);
    const double mean_after = DefaultPlanner::calculate_mean_nonzero_traffic(env, parking, vertex_traffic);
    const double parking_total = std::accumulate(vertex_traffic.begin(), vertex_traffic.end(), 0.0);
    std::cout << "[pickup-traffic] timestep=" << env->curr_timestep
              << " enabled=" << enabled << " delivery_flow=" << delivery_total
              << " added_pickup_flow=" << parking_total - delivery_total
              << " mean_positive_before=" << mean_before
              << " mean_positive_after=" << mean_after << std::endl;
}
}

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 * 
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit 
 * and adjust for a specified tolerance to account for potential timing errors. 
 * It ensures that initialization does not exceed the allocated time.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);    
}

/**
 * Plans a task schedule within a specified time limit.
 * 
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 * 
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    const double B = env->parking_strength;

    if (env->parking_model == 8)
    {
        // One flow chooses both task assignments and idle parking goals.
        DefaultPlanner::schedule_plan_flow_joint_task_parking(
            limit, proposed_schedule, env, background_flow, use_traffic, new_only,
            B, 2.0, 0.5);
        return;
    }

    const bool sequential_parking = env->parking_model >= 4 && env->parking_model <= 7;
    std::vector<DefaultPlanner::Double4> augmented_traffic;
    const bool augment = sequential_parking && pickup_traffic;
    DefaultPlanner::schedule_plan_flow(limit, proposed_schedule, env, background_flow, use_traffic, new_only,
                                      augment ? &augmented_traffic : nullptr);
    const auto& parking_traffic = augment ? augmented_traffic : background_flow;
    if (sequential_parking)
        log_parking_traffic(env, background_flow, parking_traffic, augment);
    if (env->parking_model == 4) 
    {
        DefaultPlanner::schedule_plan_flow_dummy(limit, env, parking_traffic, use_traffic, new_only);
    }
    else if (env->parking_model == 5) 
    {
        DefaultPlanner::schedule_plan_flow_dummy_combined(limit, env, parking_traffic, use_traffic, new_only, B, 2.0, 0.0);
    }
    else if (env->parking_model == 6) 
    {
        DefaultPlanner::schedule_plan_flow_dummy_combined(limit, env, parking_traffic, use_traffic, new_only, B, 0.0, 0.5);
    }
    else if (env->parking_model == 7)
    {
        // Tasks are fixed by the first flow; combine clearance and traffic
        // penalties when choosing parking goals for the remaining idle agents.
        DefaultPlanner::schedule_plan_flow_dummy_combined(limit, env, parking_traffic, use_traffic, new_only, B, 2.0, 0.5);
    }
}

void TaskScheduler::set_flow(std::vector<DefaultPlanner::Double4> flow)
{
    background_flow = flow;
}

void TaskScheduler::set_use_traffic(bool use_traffic)
{
    this->use_traffic = use_traffic;
}
void TaskScheduler::set_new_only(bool new_only)
{
    this->new_only = new_only;
}
void TaskScheduler::set_solver(int solver)
{
    this->solver = solver;
}
