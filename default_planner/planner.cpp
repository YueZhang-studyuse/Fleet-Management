#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include "scheduler.h"
#include <numeric>
#include <limits>
#include <tuple>


namespace DefaultPlanner{

    //default planner data
    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    std::mt19937 mt1;
    TrajLNS trajLNS;

    std::vector<Double4> get_opened_flow(SharedEnvironment* env)
    {
        double decay = 1;
        std::vector<Double4> background_flow(env->map.size(),Double4{0,0,0,0});
        //for (int i_task=0 ; i_task < env->task_pool.size() ;i_task++)
        for (auto task: env->task_pool)
        {
            if (task.second.idx_next_loc > 0) //task opened
            {
                int agent = task.second.agent_assigned;
                if (trajLNS.trajs[agent].empty())
                    continue;
                int loc, prev_loc, diff, d;
                double current_cost = 1;
                for (int j = 1; j < trajLNS.trajs[agent].size(); j++)
                {
                    loc = trajLNS.trajs[agent][j];
                    prev_loc = trajLNS.trajs[agent][j-1];
                    diff = loc - prev_loc;
                    d = get_d(diff, env);

                    background_flow[prev_loc].d[d] += current_cost;
                    current_cost *= decay;
                }
            }
        }
        return background_flow;
    }

    /**
     * @brief Default planner initialization
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
            return;
    };

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env, unordered_map<int,list<int>> agent_guide_path)
    {

        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        cout << "plan limit " << time_limit <<endl;

        // data sturcture for record the previous decision of each agent
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        decision.clear();
        decision.resize(env->map.size(), -1);

        if (env->curr_timestep == 0)
        {
            dummy_goals.assign(env->num_of_agents, -1);

            const std::vector<int>& sorted_cells = get_sorted_cell_ids_by_clearance();

            std::vector<int> traversable_cells;
            traversable_cells.reserve(env->map.size());
            for (int loc : sorted_cells)
            {
                if (loc >= 0 && loc < env->map.size() && env->map[loc] != 1)
                    traversable_cells.push_back(loc);
            }

            std::vector<int> agent_order;
            agent_order.resize(env->num_of_agents);
            std::iota(agent_order.begin(), agent_order.end(), 0);

            std::vector<bool> goal_used(env->map.size(), false);
            int cell_ptr = 0;
            for (int agent_id : agent_order)
            {
                while (cell_ptr < static_cast<int>(traversable_cells.size()) && goal_used[traversable_cells[cell_ptr]])
                    ++cell_ptr;

                if (cell_ptr < static_cast<int>(traversable_cells.size()))
                {
                    int goal = traversable_cells[cell_ptr++];
                    dummy_goals[agent_id] = goal;
                    goal_used[goal] = true;
                }
                else
                {
                    int fallback_goal = env->curr_states[agent_id].location;
                    if (fallback_goal >= 0 && fallback_goal < env->map.size() && env->map[fallback_goal] != 1 && !goal_used[fallback_goal])
                    {
                        dummy_goals[agent_id] = fallback_goal;
                        goal_used[fallback_goal] = true;
                    }
                }
            }
        }

        // update the status of each agent and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            //initialise the shortest distance heuristic table for the goal location of the agent
            if ( ( std::chrono::steady_clock::now() < end_time) )
            {
                for(int j=0; j<env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                        if (trajLNS.heuristics.at(goal_loc).empty()){
                            init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                            count++;
                        }
                }

                if (env->goal_locations[i].empty() && i < dummy_goals.size() && dummy_goals[i] != -1)
                {
                    int dummy_goal_loc = dummy_goals[i];
                    if (trajLNS.heuristics.at(dummy_goal_loc).empty())
                    {
                        init_heuristic(trajLNS.heuristics[dummy_goal_loc], env, dummy_goal_loc);
                        count++;
                    }
                }
            }

            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 

            // check if the agent need a guide path update, when the agent has no guide path or the guide path does not end at the goal location
            require_guide_path[i] = false;
            
            // // set the goal location of each agent
            // if (env->goal_locations[i].empty())
            // {
            //     // assign pre-selected high-clearance dummy goals when no real task goal exists
            //     if (i < dummy_goals.size() && dummy_goals[i] != -1)
            //         trajLNS.tasks[i] = dummy_goals[i];
            //     else
            //         trajLNS.tasks[i] = env->curr_states[i].location;
            //     //set the pririty to 0
            //     p[i] = 0;
            //     // //remove the trajectory of the agent 
            //     if (!trajLNS.trajs[i].empty())
            //         remove_traj(trajLNS, i);
            //     //add its current position as the trajectory of the agent
            //     trajLNS.trajs[i].clear();
            //     // trajLNS.trajs[i].push_back(env->curr_states[i].location);
            //     // add_traj(trajLNS,i);
            //     // update_dist_2_path(trajLNS,i);
            // }
            // else
            // {
            //     trajLNS.tasks[i] = env->goal_locations[i].front().first;
            //     if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
            //     {
            //         require_guide_path[i] = true;
            //         p[i] = p_copy[i];
            //     }
            //     else
            //     {
            //         p[i] = p[i]+1;
            //     }
            //     if (trajLNS.neighbors[env->curr_states[i].location].size() == 1){
            //         p[i] = p[i] + 10;
            //     }
            // }

            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                if (i < dummy_goals.size() && dummy_goals[i] != -1)
                    trajLNS.tasks[i] = dummy_goals[i];
                else
                    trajLNS.tasks[i] = env->curr_states[i].location;
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
            {
                require_guide_path[i] = true;
            }
            // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
            if(require_guide_path[i])
                p[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;
            if (env->goal_locations[i].empty())
            {
                p[i] = 0;
            }

            // give priority bonus to the agent if the agent is in a deadend location
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            
        }

        // compute the congestion minimised guide path for the agents that need guide path update
        for (int i = 0; i < env->num_of_agents;i++)
        {
            if (std::chrono::steady_clock::now() >end_time)
            {
                cout<<"compute initial stop until "<<i<<endl;
                break;
            }
            if (require_guide_path[i])
            {
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (agent_guide_path.find(i) != agent_guide_path.end())
                {
                    trajLNS.trajs[i].clear();
                    trajLNS.trajs[i].insert(trajLNS.trajs[i].end(), agent_guide_path[i].begin(), agent_guide_path[i].end());
                    add_traj(trajLNS,i);
                    update_dist_2_path(trajLNS,i);
                }
                else
                {
                    update_traj(trajLNS, i);
                }
            }
        }

        // iterate and recompute the guide path to optimise traffic flow
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated,end_time);

        // sort agents based on the current priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        //pibt
        for (int i : ids)
        {
            if (next_states[i].location==-1)
            {
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        
        // post processing the targeted next location to turning or moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids)
        {
            // post process the targeted next location to turning or moving actions
            actions.at(id) = getAction(prev_states.at(id),next_states.at(id).location, env);

        }
        prev_states = next_states;
        return;

    };

    void plan_pibt(int time_limit,vector<Action> & actions, SharedEnvironment* env)
    {

        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        cout << "plan limit " << time_limit <<endl;

        // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        // data sturcture for record the previous decision of each agent
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        // update the status of each agent and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            
            // check if the agent completed the action in the previous timestep
            // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 
            if (decided[i].loc == -1){
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            decided[i].state = DONE::DONE;

            // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
            if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            // give priority bonus to the agent if the agent is in a deadend location
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }
            
        }

        // sort agents based on the current priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        //pibt
        for (int i : ids)
        {
            if (env->goal_locations[i].empty())
            {
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                trajLNS.trajs[i].clear();
            }
            if (next_states[i].location==-1)
            {
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        
        // post processing the targeted next location to turning or moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids)
        {
            //clear the decision table based on which agent has next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;

            if (next_states.at(id).location >=0)
            {
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }

            // post process the targeted next location to turning or moving actions
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }



        prev_states = next_states;
        return;

    };
}