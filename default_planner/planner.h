#ifndef PLANNER
#define PLANNER

#include "Types.h"
#include "TrajLNS.h"
#include <random>


namespace DefaultPlanner{

    
    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    void init_parking_random(SharedEnvironment* env);
    void init_parking_clearance(SharedEnvironment* env);

    // Compute env->commit_window joint PIBT moves: actions[step][agent].
    void plan(int time_limit, vector<vector<Action>>& actions, SharedEnvironment* env,
              const unordered_map<int, list<int>>& agent_guide_path);

    // Explicit single-step entry point.
    void plan(int time_limit, vector<Action>& actions, SharedEnvironment* env,
              const unordered_map<int, list<int>>& agent_guide_path);

    // std::vector<Int4> get_flow();

    std::vector<Double4> get_opened_flow(SharedEnvironment* env);
    void plan_pibt(int time_limit,vector<Action> & actions, SharedEnvironment* env);


}
#endif
