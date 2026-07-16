


#include "pibt.h"





namespace DefaultPlanner{

int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
	{
        min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));	
		// if (lns.env->goal_locations[ai].empty())
		// {
		// 	std::cout<<"dist2path is not empty but goal location is empty for agent "<<ai<<std::endl;
		// 	std::cout<<"guid path size: "<<lns.trajs[ai].size()<<std::endl;
		// }
	}
    else if (!lns.heuristics[lns.tasks.at(ai)].empty())
	{
        min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
		// std::cout<<"heuristic table is not empty but dist2path is empty for agent "<<ai<<std::endl;
		// std::cout<<"target size: "<<lns.env->goal_locations[ai].size()<<std::endl;
	}
    else
	{
        min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
		// std::cout<<"heuristic table is empty for agent "<<ai<<std::endl;
		// std::cout<<"target size: "<<lns.env->goal_locations[ai].size()<<std::endl;
	}
    
    return min_heuristic;
}

bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, TrajLNS& lns)
	  {
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	//int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	// int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	
	//use regular guid path heuristic for agent that has goal
	if (!lns.env->goal_locations[curr_id].empty())
	{
		for (auto& neighbor: neighbors)
		{

			assert(validateMove(prev_loc, neighbor, lns.env));

			int min_heuristic = get_gp_h(lns, curr_id, neighbor);

			successors.emplace_back(neighbor,min_heuristic,0,rand());
		}

		int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);

		successors.emplace_back(prev_loc, wait_heuristic,0,rand());
	}
	else //evaluate where to push based on interference with other agents, for agent that has no goal
	//initial attempt: only check current cell
	{
		for (auto& neighbor: neighbors)
		{
			assert(validateMove(prev_loc, neighbor, lns.env));

			int min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(curr_id)], lns.env, target, &(lns.neighbors));
			int secondary_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(curr_id)], lns.env, neighbor, &(lns.neighbors));

			successors.emplace_back(neighbor,min_heuristic,secondary_heuristic,rand());
		}

		int wait_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(curr_id)], lns.env, target, &(lns.neighbors));
		int wait_secondary_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(curr_id)], lns.env, prev_loc, &(lns.neighbors));

		successors.emplace_back(prev_loc, wait_heuristic,wait_secondary_heuristic,rand());
	}


	std::sort(successors.begin(), successors.end(), 
		[&](PIBT_C& a, PIBT_C& b)
		{
			if (a.heuristic == b.heuristic)
			{
				if (a.secondary_heuristic == b.secondary_heuristic)
				{
					// random tie break
					return a.tie_breaker < b.tie_breaker;
				}
				return a.secondary_heuristic < b.secondary_heuristic;
			}
			return a.heuristic < b.heuristic; 
		});

    for (auto& next: successors)
	{
		
		if (next.location == -1)
			continue;
		if (decision[next.location] != -1){
			continue;
		}
		if (higher_id != -1 && prev_decision[next.location] == higher_id){
			continue;
		}
		next_states.at(curr_id) = State(next.location, -1, -1);
		decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 && 
			next_states.at(prev_decision.at(next.location)).location == -1){
            int lower_id = prev_decision.at(next.location);
            if (!causalPIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns)){
				continue;
            }
        }

        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);;
    decision.at(prev_loc) = curr_id;     

	#ifndef NDEBUG
		std::cout<<"false: "<< next_states[curr_id].location<<","<<next_states[curr_id].orientation <<std::endl;
	#endif   

    return false;
}


Action getAction(State& prev, int next_loc, SharedEnvironment* env)
{
	int diff = next_loc -prev.location;
	if (diff == 1)
		return Action::E;
	else if (diff == -1)
		return Action::WE;
	else if (diff > 0)
		return Action::S;
	else if (diff == 0)
		return Action::WA;
	else
		return Action::N;

}

}