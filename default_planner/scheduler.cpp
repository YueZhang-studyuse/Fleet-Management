#include "scheduler.h"
// #include "gurobi_c++.h"
#include <algorithm>
#include <boost/heap/pairing_heap.hpp>
#include <limits>
#include <numeric>
#include <queue>
#include <tuple>

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::vector<std::tuple<int,int>> map_clearance;
std::vector<int> sorted_agent_ids_by_clearance;

unordered_map<int,list<int>> agent_guide_path; //agent id, guide path from flow

struct Node
{
    int location;
    int value;

    Node() = default;
    Node(int location, int value) : location(location), value(value) {}
    // the following is used to compare nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const Node& n1, const Node& n2) const
        {
            return n1.value >= n2.value;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
};

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);

    const int map_size = static_cast<int>(env->map.size());
    std::vector<int> clearance_distance(map_size, std::numeric_limits<int>::max());

    std::queue<int> q;
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1)
        {
            clearance_distance[loc] = 0;
            q.push(loc);
        }
    }

    const int cols = env->cols;
    const int rows = env->rows;
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    if (q.empty())
    {
        clearance_distance.assign(map_size, rows + cols);
    }
    else
    {
        while (!q.empty())
        {
            int current = q.front();
            q.pop();

            int cur_r = current / cols;
            int cur_c = current % cols;

            for (int d = 0; d < 8; ++d)
            {
                int nr = cur_r + dr[d];
                int nc = cur_c + dc[d];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                    continue;

                int next = nr * cols + nc;

                if (clearance_distance[next] > clearance_distance[current] + 1)
                {
                    clearance_distance[next] = clearance_distance[current] + 1;
                    q.push(next);
                }
            }
        }
    }

    std::vector<int> obstacle_prefix((rows + 1) * (cols + 1), 0);
    auto pref = [&](int r, int c) -> int&
    {
        return obstacle_prefix[r * (cols + 1) + c];
    };

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int loc = r * cols + c;
            int obstacle = (env->map[loc] == 1) ? 1 : 0;
            pref(r + 1, c + 1) = obstacle + pref(r, c + 1) + pref(r + 1, c) - pref(r, c);
        }
    }

    map_clearance.assign(map_size, std::make_tuple(0, 0));
    for (int loc = 0; loc < map_size; ++loc)
    {
        int clearance = clearance_distance[loc];
        int r = loc / cols;
        int c = loc % cols;
        int top = std::max(0, r - clearance);
        int bottom = std::min(rows - 1, r + clearance);
        int left = std::max(0, c - clearance);
        int right = std::min(cols - 1, c + clearance);

        int obstacle_count = pref(bottom + 1, right + 1)
                           - pref(top, right + 1)
                           - pref(bottom + 1, left)
                           + pref(top, left);

        map_clearance[loc] = std::make_tuple(clearance, obstacle_count);
    }

    const int num_agents = static_cast<int>(env->curr_states.size());
    sorted_agent_ids_by_clearance.resize(num_agents);
    std::iota(sorted_agent_ids_by_clearance.begin(), sorted_agent_ids_by_clearance.end(), 0);
    std::sort(sorted_agent_ids_by_clearance.begin(), sorted_agent_ids_by_clearance.end(),
              [&](int a, int b)
              {
                  int loc_a = env->curr_states[a].location;
                  int loc_b = env->curr_states[b].location;

                  auto clearance_a = (loc_a >= 0 && loc_a < map_size) ? map_clearance[loc_a] : std::make_tuple(-1, std::numeric_limits<int>::max());
                  auto clearance_b = (loc_b >= 0 && loc_b < map_size) ? map_clearance[loc_b] : std::make_tuple(-1, std::numeric_limits<int>::max());

                  if (std::get<0>(clearance_a) != std::get<0>(clearance_b))
                      return std::get<0>(clearance_a) > std::get<0>(clearance_b);
                  if (std::get<1>(clearance_a) != std::get<1>(clearance_b))
                      return std::get<1>(clearance_a) < std::get<1>(clearance_b);
                  return a < b;
              });

    cout << "[DEBUG] Clearance table:" << endl;
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int loc = r * cols + c;
            cout << std::get<0>(map_clearance[loc]) << "(" << std::get<1>(map_clearance[loc]) << ") ";
        }
        cout << endl;
    }

    cout << "[DEBUG] Agents sorted by clearance (high->low): ";
    for (int agent_id : sorted_agent_ids_by_clearance)
    {
        int loc = env->curr_states[agent_id].location;
        auto clearance = (loc >= 0 && loc < map_size) ? map_clearance[loc] : std::make_tuple(-1, -1);
        cout << agent_id << "(" << std::get<0>(clearance) << "," << std::get<1>(clearance) << ") ";
    }
    cout << endl;
    return;
}

void schedule_plan_raw(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        // //keep assigning until timeout
        // if (std::chrono::steady_clock::now() > endtime)
        // {
        //     break;
        // }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            // //check for timeout every 10 task evaluations
            // if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            // {
            //     break;
            // }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc : env->task_pool[t_id].locations){
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
                break; // only consider the first location of the task for the makespan
            }

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
    #ifndef NDEBUG
    cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    #endif
    return;

}


void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    agent_guide_path.clear();

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids; //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign
    unordered_map<int,list<int>> task_loc_ids;

    for (int agent = 0; agent < env->num_of_agents; agent++)
    {
        if (env->curr_task_schedule[agent] == -1)
            flexible_agent_ids.push_back(agent);
    }

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            if (new_only)
            {
                if (task.second.agent_assigned == -1)
                {
                    flexible_task_ids.push_back(task.first);
                    task_loc_ids[task.second.locations[0]].push_back(task.first);
                }
            }
            else
            {
                flexible_task_ids.push_back(task.first);
                task_loc_ids[task.second.locations[0]].push_back(task.first);
                if (task.second.agent_assigned != -1)
                    flexible_agent_ids.push_back(task.second.agent_assigned);
            }
        }
    }

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    if (num_workers == 0 || num_tasks == 0)
    {
        return;
    }

    // Start timing
    start_time = std::chrono::high_resolution_clock::now();
    
    // Create the graph
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g); // Store the flow for warm start

    vector<ListDigraph::Node> map_nodes(env->map.size());

    ListDigraph::Node source = g.addNode(); // Source node
    ListDigraph::Node sink = g.addNode();   // Sink node

    unordered_map<int, int> node_to_maploc; // map graph node id to env->map index
    unordered_map<int, int> maploc_to_node; // reverse

    // Create worker and task nodes
    for (int i = 0 ; i < env->map.size(); ++i)
    {
        map_nodes[i] = g.addNode();
        int id = lemon::ListDigraphBase::id(map_nodes[i]);
        node_to_maploc[id] = i;
        maploc_to_node[i] = id;
    } 

    // Set supply/demand values
    supply[source] = num_workers; // Source supplies workers
    supply[sink] = -num_workers;  // Sink absorbs tasks

    if (num_workers > num_tasks)
    {
        supply[source] = num_tasks; // Source supplies tasks
        supply[sink] = -num_tasks;  // Sink absorbs tasks
    }

    for (int i = 0; i < num_workers; ++i) supply[map_nodes[i]] = 0;

    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, map_nodes[env->curr_states[flexible_agent_ids[i]].location]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers
    }

    unordered_map<int, int> node_to_task_id;


    for (auto task: task_loc_ids)
    {
        int loc = task.first;
        ListDigraph::Arc a = g.addArc(map_nodes[loc], sink);
        node_to_task_id[lemon::ListDigraphBase::id(map_nodes[loc])] = loc;
        capacity[a] = task.second.size();
        cost[a] = 0;
    }

    vector<int> neighbor = {-env->cols, 1, env->cols, -1};

    int  diff, d, op_flow, all_vertex_flow,vertex_flow;
    int temp_op, temp_vertex;

    for (int loc = 0; loc < env->map.size(); loc++)
    {
        if (env->map[loc] == 1) continue;
        //try four directions
        for (int i = 0; i < 4; i++)
        {
            int neighbor_loc = loc + neighbor[i];
            if (neighbor_loc < 0 || neighbor_loc >= env->map.size() || env->map[neighbor_loc] == 1)
                continue;
            ListDigraph::Arc a = g.addArc(map_nodes[loc], map_nodes[neighbor_loc]);

            if (use_traffic)
            {
                op_flow = 0;
                all_vertex_flow = 0;
                diff = loc-neighbor_loc;
                d = get_d(diff,env);
                temp_op = ( (background_flow[loc].d[d]+1) * background_flow[neighbor_loc].d[(d+2)%4]);
                temp_vertex = 1;
                for (int j=0; j<4; j++)
                {
                    temp_vertex += background_flow[neighbor_loc].d[j];                
                }
                op_flow += temp_op;
                all_vertex_flow+= (temp_vertex-1) /2;
                cost[a] = 1 + op_flow + all_vertex_flow;
            }
            else
            {
                cost[a] = 1;
            }

            capacity[a] = num_workers;
        }
    }

    unordered_map<int,int> edge_flows; //arc id, flow count

    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL) 
    {
        int cnt = 0;

        // cout << "Optimal assignment with minimum cost:" << endl;
        // Iterate over all worker nodes
        for (int i = 0; i < num_workers; i++) 
        {
            ListDigraph::Node current = map_nodes[env->curr_states[flexible_agent_ids[i]].location];

            list<int> path;

            while (node_to_task_id.find(lemon::ListDigraphBase::id(current)) == node_to_task_id.end()) 
            {
                // Check if the current node is a task node
                if (current == sink) break; // Reached sink, no task node found

                int loc = node_to_maploc[lemon::ListDigraphBase::id(current)];
                path.push_back(loc);

                // Follow the flow to the next node
                // Find the next node in the path
                bool found = false;
                for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc) 
                {
                    if (ns.flow(arc) > 0) 
                    { // Follow the flow
                        if (edge_flows.find(lemon::ListDigraphBase::id(arc)) == edge_flows.end())
                        {
                            edge_flows[lemon::ListDigraphBase::id(arc)] = ns.flow(arc);
                        }
                        if (edge_flows[lemon::ListDigraphBase::id(arc)] <= 0)
                            continue;
                        current = g.target(arc);
                        edge_flows[lemon::ListDigraphBase::id(arc)]--;
                        found = true;
                        break;
                    }
                }
                if (!found) break;  // No path found
            }
            // Now `current` should be a task node
            if (node_to_task_id.find(lemon::ListDigraphBase::id(current)) != node_to_task_id.end()) 
            {
                int task_loc = node_to_task_id[lemon::ListDigraphBase::id(current)];
                int task_id = task_loc_ids[task_loc].front();
                // node_to_task_id[current].pop_front();
                path.push_back(task_loc);
                //cout << "Worker " << flexible_agent_ids[i] << " is assigned to Task " << task_id  << " through intermediate nodes." << endl;
                proposed_schedule[flexible_agent_ids[i]] = task_id;
                if (use_traffic)
                    agent_guide_path[flexible_agent_ids[i]] = path;
                task_loc_ids[task_loc].pop_front();
                if (task_loc_ids[task_loc].empty())
                {
                    task_loc_ids.erase(task_loc);
                    node_to_task_id.erase(lemon::ListDigraphBase::id(current));
                }
            }
            else 
            {
                proposed_schedule[flexible_agent_ids[i]] = -1;
                //cout << "No solution found." << endl;
            }
        }
    }
    else 
    {
        cout << "No optimal solution found." << endl;
    }
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    cout << "Solving time: " << elapsed_time << " seconds" << endl;

}

void schedule_plan_flow_hist(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<pair<double,double>>& background_flow, bool new_only)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    agent_guide_path.clear();

    proposed_schedule.resize(env->num_of_agents, -1);

    vector<int>flexible_agent_ids; //storing the agents not doing a opened task
    vector<int>flexible_task_ids; //storing the tasks we consider to swap/assign
    unordered_map<int,list<int>> task_loc_ids;

    for (auto task: env->task_pool)
    {
        if (task.second.idx_next_loc > 0) //task opened
        {
            proposed_schedule[task.second.agent_assigned] = task.first;
        }
        else
        {
            if (new_only)
            {
                if (task.second.agent_assigned == -1)
                {
                    flexible_task_ids.push_back(task.first);
                    task_loc_ids[task.second.locations[0]].push_back(task.first);
                }
            }
            else
            {
                flexible_task_ids.push_back(task.first);
                task_loc_ids[task.second.locations[0]].push_back(task.first);
                if (task.second.agent_assigned != -1)
                    flexible_agent_ids.push_back(task.second.agent_assigned);
            }
        }
    }

    cout<<"num of flexible agents: "<<flexible_agent_ids.size()<<endl;
    cout<<"num of flexible tasks: "<<flexible_task_ids.size()<<endl;

    int num_workers = flexible_agent_ids.size();
    int num_tasks = flexible_task_ids.size();

    // Start timing
    start_time = std::chrono::high_resolution_clock::now();
    
    // Create the graph
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g); // Store the flow for warm start

    vector<ListDigraph::Node> map_nodes(env->map.size());

    ListDigraph::Node source = g.addNode(); // Source node
    ListDigraph::Node sink = g.addNode();   // Sink node

    unordered_map<int, int> node_to_maploc; // map graph node id to env->map index
    unordered_map<int, int> maploc_to_node; // reverse

    // Create worker and task nodes
    for (int i = 0 ; i < env->map.size(); ++i)
    {
        map_nodes[i] = g.addNode();
        int id = lemon::ListDigraphBase::id(map_nodes[i]);
        node_to_maploc[id] = i;
        maploc_to_node[i] = id;
    } 

    // Set supply/demand values
    supply[source] = num_workers; // Source supplies workers
    supply[sink] = -num_workers;  // Sink absorbs tasks

    if (num_workers > num_tasks)
    {
        supply[source] = num_tasks; // Source supplies tasks
        supply[sink] = -num_tasks;  // Sink absorbs tasks
    }

    for (int i = 0; i < num_workers; ++i) supply[map_nodes[i]] = 0;

    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, map_nodes[env->curr_states[flexible_agent_ids[i]].location]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers
    }

    unordered_map<int, int> node_to_task_id;


    for (auto task: task_loc_ids)
    {
        int loc = task.first;
        ListDigraph::Arc a = g.addArc(map_nodes[loc], sink);
        node_to_task_id[lemon::ListDigraphBase::id(map_nodes[loc])] = loc;
        capacity[a] = task.second.size();
        cost[a] = 0;
    }

    vector<int> neighbor = {-env->cols, 1, env->cols, -1};
    double vertex_flow = 0;
    double edge_flow = 0;

    for (int loc = 0; loc < env->map.size(); loc++)
    {
        if (env->map[loc] == 1) continue;
        //try four directions
        for (int i = 0; i < 4; i++)
        {
            int neighbor_loc = loc + neighbor[i];
            if (neighbor_loc < 0 || neighbor_loc >= env->map.size() || env->map[neighbor_loc] == 1)
                continue;
            ListDigraph::Arc a = g.addArc(map_nodes[loc], map_nodes[neighbor_loc]);

            // if (background_flow[loc*5].second != 0)
            //     vertex_flow = background_flow[loc*5].first/background_flow[loc*5].second;

            if (background_flow[neighbor_loc*5+i].second != 0)
                edge_flow = (double)background_flow[neighbor_loc*5+i].first/(double)background_flow[neighbor_loc*5+i].second;

            cost[a] = 1 + edge_flow;

            capacity[a] = num_workers;
        }
    }

    unordered_map<int,int> edge_flows; //arc id, flow count

    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL) 
    {
        int cnt = 0;

        cout << "Optimal assignment with minimum cost:" << endl;
        // Iterate over all worker nodes
        for (int i = 0; i < num_workers; i++) 
        {
            ListDigraph::Node current = map_nodes[env->curr_states[flexible_agent_ids[i]].location];

            list<int> path;

            while (node_to_task_id.find(lemon::ListDigraphBase::id(current)) == node_to_task_id.end()) 
            {
                // Check if the current node is a task node
                if (current == sink) break; // Reached sink, no task node found

                int loc = node_to_maploc[lemon::ListDigraphBase::id(current)];
                path.push_back(loc);

                // Follow the flow to the next node
                // Find the next node in the path
                bool found = false;
                for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc) 
                {
                    if (ns.flow(arc) > 0) 
                    { // Follow the flow
                        if (edge_flows.find(lemon::ListDigraphBase::id(arc)) == edge_flows.end())
                        {
                            edge_flows[lemon::ListDigraphBase::id(arc)] = ns.flow(arc);
                        }
                        if (edge_flows[lemon::ListDigraphBase::id(arc)] <= 0)
                            continue;
                        current = g.target(arc);
                        edge_flows[lemon::ListDigraphBase::id(arc)]--;
                        found = true;
                        break;
                    }
                }
                if (!found) break;  // No path found
            }
            // Now `current` should be a task node
            if (node_to_task_id.find(lemon::ListDigraphBase::id(current)) != node_to_task_id.end()) 
            {
                int task_loc = node_to_task_id[lemon::ListDigraphBase::id(current)];
                int task_id = task_loc_ids[task_loc].front();
                // node_to_task_id[current].pop_front();
                path.push_back(task_loc);
                // cout << "Worker " << i << " is assigned to Task " << task_id  << " through intermediate nodes." << endl;
                proposed_schedule[flexible_agent_ids[i]] = task_id;
                agent_guide_path[flexible_agent_ids[i]] = path;
                task_loc_ids[task_loc].pop_front();
                if (task_loc_ids[task_loc].empty())
                {
                    task_loc_ids.erase(task_loc);
                    node_to_task_id.erase(lemon::ListDigraphBase::id(current));
                }
            }
            else 
            {
                cout << "No solution found." << endl;
            }
        }
    }
    else 
    {
        cout << "No optimal solution found." << endl;
    }
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    cout << "Solving time: " << elapsed_time << " seconds" << endl;

}



void printDIMACS(ListDigraph& g, 
                 ListDigraph::Node source, 
                 ListDigraph::Node sink, 
                 vector<ListDigraph::Node>& workers, 
                 vector<ListDigraph::Node>& tasks, 
                 ListDigraph::ArcMap<int>& capacity, 
                 ListDigraph::ArcMap<double>& cost) 
{
    int num_workers = workers.size();
    int num_tasks = tasks.size();
    int num_nodes = 2 + num_workers + num_tasks; // source, sink, workers, tasks
    int num_arcs = num_workers + num_tasks + (num_workers * num_tasks); // source→workers + tasks→sink + workers→tasks

    // Print problem line
    cout << "p min " << num_nodes << " " << num_arcs << endl;

    // Print node descriptors
    cout << "n 1 " << num_workers << "   c Source (supplies workers)" << endl;
    cout << "n " << num_nodes << " -" << num_workers << "   c Sink (absorbs workers)" << endl;

    // Print arcs (Source → Workers)
    for (int i = 0; i < num_workers; ++i) {
        cout << "a 1 " << (i + 2) << " 0 1 0   c Source to Worker " << (i + 1) << endl;
    }

    // Print arcs (Workers → Tasks)
    for (int i = 0; i < num_workers; ++i) {
        for (int j = 0; j < num_tasks; ++j) {
            ListDigraph::Arc a = findArc(g, workers[i], tasks[j]);
            if (a == INVALID) continue; // Skip if no valid edge

            cout << "a " << (i + 2) << " " << (num_workers + j + 2) << " 0 " 
                 << capacity[a] << " " << cost[a] 
                 << "   c Worker " << (i + 1) << " to Task " << (j + 1) << endl;
        }
    }

    // Print arcs (Tasks → Sink)
    for (int j = 0; j < num_tasks; ++j) {
        cout << "a " << (num_workers + j + 2) << " " << num_nodes << " 0 1 0   c Task " << (j + 1) << " to Sink" << endl;
    }
}


bool isTaskNode(ListDigraph::Node node, ListDigraph& g, ListDigraph::Node sink) 
{
    for (ListDigraph::OutArcIt outArc(g, node); outArc != INVALID; ++outArc) {
        ListDigraph::Arc arc = outArc;
        if (g.target(arc) == sink) {
            return true;  // Node is connected to sink (task node)
        }
    }
    return false;
}

unordered_map<int,list<int>> get_guide_path()
{ 
    return agent_guide_path; 
}

const std::vector<std::tuple<int,int>>& get_map_clearance()
{
    return map_clearance;
}

const std::vector<int>& get_sorted_agent_ids_by_clearance()
{
    return sorted_agent_ids_by_clearance;
}

};
