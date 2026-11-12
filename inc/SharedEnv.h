#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"
#include <unordered_map>


typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;
typedef std::unordered_map<int, Task> TaskPool;

class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;

    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    int commit_window = 1; // number of joint action steps requested per planning call
    vector<State> curr_states;

    TaskPool task_pool; // task_id -> Task
    vector<int> new_tasks; // task ids revealed since the previous planning call
    vector<int> new_freeagents; // agents freed since the previous planning call
    vector<int> curr_task_schedule; // the current scheduler, agent_id -> task_id
    std::unordered_map<int, std::vector<int>> dummy_parking_allocation; // agent_id -> [parking_loc]

    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    SharedEnvironment(){}

    // vector<pair<int,int>> past_waitings;
    vector<pair<double,double>> past_waitings;
    vector<int> accu_waitings;

    //configurations
    // 1 random, 2 stay, 3 clearance preselection, 4 clearance parking flow,
    // 5 all-cell clearance, 6 traffic, 7 combined parking after task flow,
    // 8 joint task/parking flow.
    int parking_model = 1;
};
