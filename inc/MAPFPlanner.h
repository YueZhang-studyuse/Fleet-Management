#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "planner.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // Explicit single-step interface; committed planning uses the overload below.
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Time-major committed actions: plan[step][agent]. time_limit is the
    // total budget in milliseconds for env->commit_window steps.
    // Delegates to DefaultPlanner's multi-step PIBT implementation.
    // Override this overload to supply a different committed planner.
    virtual void plan(int time_limit, std::vector<std::vector<Action>> & plan);

    std::vector<DefaultPlanner::Double4> get_flow();
    void plan_pibt(int time_limit,vector<Action> & actions); 

};
