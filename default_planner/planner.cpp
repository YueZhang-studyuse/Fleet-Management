#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include "scheduler.h"
#include <numeric>
#include <limits>
#include <stdexcept>
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

    void init_parking_random(SharedEnvironment* env)
    {
        std::vector<int> available_locations;
        for (size_t loc = 0; loc < env->map.size(); ++loc)
        {
            if (env->map[loc] != 1)
                available_locations.push_back(static_cast<int>(loc));
        }

        std::shuffle(available_locations.begin(), available_locations.end(), mt1);
        dummy_goals.assign(available_locations.begin(), available_locations.begin() + env->num_of_agents);
    }

    void init_parking_clearance(SharedEnvironment* env)
    {
        std::vector<int> parking_locations;
        parking_locations.reserve(env->num_of_agents);
        for (int loc : get_sorted_cell_ids_by_clearance())
        {
            if (parking_locations.size() == static_cast<size_t>(env->num_of_agents))
                break;
            if (loc < 0 || static_cast<size_t>(loc) >= env->map.size())
                throw std::runtime_error("Invalid cell in parking ranking");
            if (env->map[loc] != 1)
                parking_locations.push_back(loc);
        }

        if (parking_locations.size() != static_cast<size_t>(env->num_of_agents))
            throw std::runtime_error("Not enough ranked free locations for parking goals");

        dummy_goals = std::move(parking_locations);
    }

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
            if (env->parking_model == 1)
            {
                init_parking_random(env);
            }
            else if (env->parking_model == 3)
            {
                init_parking_clearance(env);
            }
            return;
    };
    

    // Guide-path optimization runs once, followed by a PIBT rollout from the
    // observed states. Only planner-owned predictions advance inside the window.
    static void plan_commits(int time_limit, vector<vector<Action>>& actions,
                             SharedEnvironment* env,
                             const unordered_map<int, list<int>>& agent_guide_path,
                             int commit_steps)
    {
        if (commit_steps <= 0)
            throw std::invalid_argument("Commit window must be positive");

        const auto start_time = std::chrono::steady_clock::now();
        const auto pibt_time = static_cast<long long>(PIBT_RUNTIME_PER_100_AGENTS) *
                               env->num_of_agents / 100;
        const auto deadline = start_time + std::chrono::milliseconds(time_limit);
        const auto guide_deadline = deadline - std::chrono::milliseconds(
            pibt_time * commit_steps + TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
        cout << "plan limit " << time_limit << endl;

        actions.clear();
        actions.reserve(commit_steps);
        prev_states = env->curr_states;
        vector<size_t> next_goal(env->num_of_agents, 0);

        for (int step = 0; step < commit_steps; step++)
        {
            // Reservations belong to one joint move, not the entire window.
            prev_decision.assign(env->map.size(), -1);
            decision.assign(env->map.size(), -1);
            next_states.assign(env->num_of_agents, State());

            for (int i = 0; i < env->num_of_agents; i++)
            {
                prev_decision[prev_states[i].location] = i;
                const bool has_goal = next_goal[i] < env->goal_locations[i].size();
                int target = prev_states[i].location;
                if (has_goal)
                    target = env->goal_locations[i][next_goal[i]].first;
                else if (env->parking_model == 1 || env->parking_model == 3)
                    target = dummy_goals.at(i);
                else if (env->parking_model > 3)
                {
                    const auto parking = env->dummy_parking_allocation.find(i);
                    if (parking != env->dummy_parking_allocation.end() && !parking->second.empty())
                        target = parking->second.front();
                }
                trajLNS.tasks[i] = target;
                require_guide_path[i] = trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != target;
                if (require_guide_path[i])
                {
                    p[i] = p_copy[i];
                    // Even when no time remains to rebuild a guide, PIBT must
                    // not use distances or traffic flow for the previous goal.
                    if (!trajLNS.trajs[i].empty())
                        remove_traj(trajLNS, i);
                    trajLNS.trajs[i].clear();
                }
                else if (has_goal)
                    p[i]++;
                if (!has_goal)
                    p[i] = 0;
                else if (trajLNS.neighbors[prev_states[i].location].size() == 1)
                    p[i] += 10;

                // Keep a goal-distance fallback for errands reached mid-window
                // if their new guide cannot be computed within the budget.
                if (trajLNS.heuristics.at(target).empty())
                    init_heuristic(trajLNS.heuristics[target], env, target);
            }

            // Later steps only refresh changed goals from their predicted
            // positions; they do not rerun the full traffic optimization.
            const auto refresh_deadline = step == 0 ? guide_deadline :
                deadline - std::chrono::milliseconds(pibt_time * (commit_steps - step));
            for (int i = 0; i < env->num_of_agents; i++)
            {
                if (std::chrono::steady_clock::now() >= refresh_deadline)
                    break;
                if (!require_guide_path[i])
                    continue;

                const auto guide = agent_guide_path.find(i);
                if (step == 0 && guide != agent_guide_path.end() && !guide->second.empty() &&
                    guide->second.back() == trajLNS.tasks[i])
                {
                    trajLNS.trajs[i].assign(guide->second.begin(), guide->second.end());
                    add_traj(trajLNS, i);
                    update_dist_2_path(trajLNS, i);
                }
                else
                    update_traj(trajLNS, i, prev_states[i].location);
            }

            if (step == 0)
            {
                std::unordered_set<int> updated;
                frank_wolfe(trajLNS, updated, guide_deadline);
            }

            std::sort(ids.begin(), ids.end(), [&](int a, int b) { return p[a] > p[b]; });
            for (int i : ids)
            {
                if (next_states[i].location == -1)
                    causalPIBT(i, -1, prev_states, next_states,
                               prev_decision, decision, occupied, trajLNS);
            }

            // One action per agent per committed step: actions[step][agent].
            actions.emplace_back(env->num_of_agents, Action::WA);
            for (int i = 0; i < env->num_of_agents; i++)
            {
                actions.back()[i] = getAction(prev_states[i], next_states[i].location, env);
                next_states[i].timestep = prev_states[i].timestep + 1;
                // Match TaskManager: at most one errand is consumed per step,
                // including repeated locations and an errand at the start cell.
                if (next_goal[i] < env->goal_locations[i].size() &&
                    next_states[i].location == env->goal_locations[i][next_goal[i]].first)
                    next_goal[i]++;
            }
            prev_states = next_states;
        }
    }

    void plan(int time_limit, vector<vector<Action>>& actions, SharedEnvironment* env,
              const unordered_map<int, list<int>>& agent_guide_path)
    {
        plan_commits(time_limit, actions, env, agent_guide_path, env->commit_window);
    }

    void plan(int time_limit, vector<Action>& actions, SharedEnvironment* env,
              const unordered_map<int, list<int>>& agent_guide_path)
    {
        vector<vector<Action>> commits;
        plan_commits(time_limit, commits, env, agent_guide_path, 1);
        actions = std::move(commits.front());
    }

    void plan_pibt(int time_limit,vector<Action> & actions, SharedEnvironment* env)
    {

        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        cout << "plan limit " << time_limit <<endl;

        if (env->curr_timestep == 0)
            init_parking_random(env);

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
