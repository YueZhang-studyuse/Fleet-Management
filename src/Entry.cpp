#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit)
{
    env->past_waitings.resize(env->map.size()*5,{0,0});
    env->accu_waitings.resize(env->num_of_agents,0);
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function is called once per commit window.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a batch of joint actions, indexed by step and then by agent.
//NB: time_limit is the total window budget in milliseconds.
void Entry::compute(int time_limit, std::vector<std::vector<Action>> & plan, std::vector<int> & proposed_schedule)
{
    scheduler->set_flow(planner->get_flow());
    scheduler->plan(time_limit, proposed_schedule);
    update_goal_locations(proposed_schedule);
    planner->plan(time_limit, plan);
}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        while (i_loc < env->task_pool[t_id].locations.size())
        {
            env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
            i_loc++;
        }
    }

    return;
}
