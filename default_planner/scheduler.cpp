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
std::vector<int> sorted_cell_ids_by_clearance;
std::vector<int> sorted_cell_clearance_level_indices;
std::vector<int> sorted_cell_clearance_levels;

unordered_map<int,list<int>> agent_guide_path; //agent id, guide path from flow

static double avg_task_step_cost_for_parking = 1.0;

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

static std::vector<std::tuple<int, int>> calculate_map_clearance(const std::vector<int>& map, int rows, int cols)
{
    const int map_size = static_cast<int>(map.size());
    std::vector<int> clearance_distance(map_size, std::numeric_limits<int>::max());

    // Multi-source BFS from all obstacle cells to get each cell's nearest-obstacle distance.
    std::queue<int> q;
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (map[loc] == 1)
        {
            clearance_distance[loc] = 0;
            q.push(loc);
        }
    }

    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    if (q.empty())
    {
        // Degenerate case: no obstacles on map, use a large uniform clearance baseline.
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

    // Prefix-sum grid enables O(1) obstacle counting inside each cell's clearance window.
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
            int obstacle = (map[loc] == 1) ? 1 : 0;
            pref(r + 1, c + 1) = obstacle + pref(r, c + 1) + pref(r + 1, c) - pref(r, c);
        }
    }

    // Store for each cell: (clearance distance, obstacles in clearance bounding box).
    std::vector<std::tuple<int, int>> clearance(map_size, std::make_tuple(0, 0));
    for (int loc = 0; loc < map_size; ++loc)
    {
        int cell_clearance = clearance_distance[loc];
        int r = loc / cols;
        int c = loc % cols;
        int top = std::max(0, r - cell_clearance);
        int bottom = std::min(rows - 1, r + cell_clearance);
        int left = std::max(0, c - cell_clearance);
        int right = std::min(cols - 1, c + cell_clearance);

        int obstacle_count = pref(bottom + 1, right + 1)
                           - pref(top, right + 1)
                           - pref(bottom + 1, left)
                           + pref(top, left);

        clearance[loc] = std::make_tuple(cell_clearance, obstacle_count);
    }

    return clearance;
}

void refresh_sorted_cells_by_clearance(const SharedEnvironment* env, int min_clearance_threshold)
{
    const int map_size = static_cast<int>(env->map.size());

    sorted_cell_ids_by_clearance.resize(map_size);
    std::iota(sorted_cell_ids_by_clearance.begin(), sorted_cell_ids_by_clearance.end(), 0);
    // Primary key: larger clearance first; secondary: fewer nearby obstacles; tertiary: lower id.
    std::sort(sorted_cell_ids_by_clearance.begin(), sorted_cell_ids_by_clearance.end(),
              [&](int a, int b)
              {
                  auto clearance_a = (a >= 0 && a < map_size) ? map_clearance[a] : std::make_tuple(-1, std::numeric_limits<int>::max());
                  auto clearance_b = (b >= 0 && b < map_size) ? map_clearance[b] : std::make_tuple(-1, std::numeric_limits<int>::max());

                  if (std::get<0>(clearance_a) != std::get<0>(clearance_b))
                      return std::get<0>(clearance_a) > std::get<0>(clearance_b);
                  if (std::get<1>(clearance_a) != std::get<1>(clearance_b))
                      return std::get<1>(clearance_a) < std::get<1>(clearance_b);
                  return a < b;
              });

    // Sorted by clearance descending, so cells below the threshold form the tail.
    sorted_cell_ids_by_clearance.erase(
        std::remove_if(sorted_cell_ids_by_clearance.begin(), sorted_cell_ids_by_clearance.end(),
                       [&](int id) { return std::get<0>(map_clearance[id]) < min_clearance_threshold; }),
        sorted_cell_ids_by_clearance.end());

    sorted_cell_clearance_level_indices.clear();
    sorted_cell_clearance_levels.clear();
    int max_clearance = -1;
    int min_clearance = -1;
    // Capture the clearance range represented in the sorted order.
    if (!sorted_cell_ids_by_clearance.empty())
    {
        int top_cell = sorted_cell_ids_by_clearance.front();
        if (top_cell >= 0 && top_cell < map_size)
            max_clearance = std::get<0>(map_clearance[top_cell]);

        int last_cell = sorted_cell_ids_by_clearance.back();
        if (last_cell >= 0 && last_cell < map_size)
            min_clearance = std::get<0>(map_clearance[last_cell]);
    }

    auto find_first_index_for_clearance = [&](int target_clearance) -> int
    {
        // Map a clearance value to its first position in the sorted cell array.
        for (int idx = 0; idx < static_cast<int>(sorted_cell_ids_by_clearance.size()); ++idx)
        {
            int cell_id = sorted_cell_ids_by_clearance[idx];
            if (cell_id < 0 || cell_id >= map_size)
                continue;
            if (std::get<0>(map_clearance[cell_id]) == target_clearance)
                return idx;
        }
        return -1;
    };

    if (max_clearance >= 0 && min_clearance >= 0)
    {
        for (int c = max_clearance; c >= min_clearance; --c)
        {
            sorted_cell_clearance_levels.push_back(c);
            sorted_cell_clearance_level_indices.push_back(find_first_index_for_clearance(c));
        }
    }
}

static void build_sort_clearance(const SharedEnvironment* env, int rows, int cols)
{
    map_clearance = calculate_map_clearance(env->map, rows, cols);
    refresh_sorted_cells_by_clearance(env);
}

static void build_sort_clearance_with_update(const SharedEnvironment* env, int rows, int cols)
{
    const int map_size = static_cast<int>(env->map.size());

    std::vector<int> temp_map = env->map;
    std::vector<bool> selected(map_size, false);
    map_clearance.assign(map_size, std::make_tuple(0, 0));

    sorted_cell_ids_by_clearance.clear();
    sorted_cell_ids_by_clearance.reserve(map_size);

    std::vector<std::tuple<int, int>> last_temp_clearance;

    // Iteratively pick the current best cell and then block it in the temporary map.
    // This makes each next selection reflect previously chosen dummy parking cells.
    for (int step = 0; step < map_size; ++step)
    {
        // Recompute clearance after each virtual placement so ranking adapts over time.
        std::vector<std::tuple<int, int>> temp_clearance = calculate_map_clearance(temp_map, rows, cols);
        last_temp_clearance = temp_clearance;

        int best_loc = -1;
        for (int loc = 0; loc < map_size; ++loc)
        {
            if (selected[loc])
                continue;

            if (best_loc == -1)
            {
                best_loc = loc;
                continue;
            }

            const auto& current = temp_clearance[loc];
            const auto& best = temp_clearance[best_loc];
            if (std::get<0>(current) > std::get<0>(best) ||
                (std::get<0>(current) == std::get<0>(best) && std::get<1>(current) < std::get<1>(best)) ||
                (std::get<0>(current) == std::get<0>(best) && std::get<1>(current) == std::get<1>(best) && loc < best_loc))
            {
                best_loc = loc;
            }
        }

        if (best_loc == -1)
            break;

        // Mark selected location as obstacle for the next iteration.
        // Persist the clearance that was actually used when this location was selected.
        sorted_cell_ids_by_clearance.push_back(best_loc);
        map_clearance[best_loc] = temp_clearance[best_loc];
        selected[best_loc] = true;
        temp_map[best_loc] = 1;
    }

    // Safety fallback: ensure sorted_cell_ids_by_clearance always contains all map cells.
    if (static_cast<int>(sorted_cell_ids_by_clearance.size()) < map_size)
    {
        for (int loc = 0; loc < map_size; ++loc)
        {
            if (!selected[loc])
            {
                sorted_cell_ids_by_clearance.push_back(loc);
                if (!last_temp_clearance.empty())
                    map_clearance[loc] = last_temp_clearance[loc];
            }
        }
    }

    sorted_cell_clearance_level_indices.clear();
    sorted_cell_clearance_levels.clear();
    int max_clearance = -1;
    int min_clearance = -1;
    // Build clearance-level lookup metadata from the final ranking.
    if (!sorted_cell_ids_by_clearance.empty())
    {
        int top_cell = sorted_cell_ids_by_clearance.front();
        if (top_cell >= 0 && top_cell < map_size)
            max_clearance = std::get<0>(map_clearance[top_cell]);

        int last_cell = sorted_cell_ids_by_clearance.back();
        if (last_cell >= 0 && last_cell < map_size)
            min_clearance = std::get<0>(map_clearance[last_cell]);
    }

    auto find_first_index_for_clearance = [&](int target_clearance) -> int
    {
        for (int idx = 0; idx < static_cast<int>(sorted_cell_ids_by_clearance.size()); ++idx)
        {
            int cell_id = sorted_cell_ids_by_clearance[idx];
            if (cell_id < 0 || cell_id >= map_size)
                continue;
            if (std::get<0>(map_clearance[cell_id]) == target_clearance)
                return idx;
        }
        return -1;
    };

    if (max_clearance >= 0 && min_clearance >= 0)
    {
        for (int c = max_clearance; c >= min_clearance; --c)
        {
            sorted_cell_clearance_levels.push_back(c);
            sorted_cell_clearance_level_indices.push_back(find_first_index_for_clearance(c));
        }
    }
}

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);

    const int rows = env->rows;
    const int cols = env->cols;
    const int map_size = static_cast<int>(env->map.size());

    // build_sort_clearance(env, rows, cols);
    // Use iterative update mode: each selected top-clearance cell is blocked before
    // computing the next one, producing a spread-out clearance ranking.
    build_sort_clearance_with_update(env, rows, cols);

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

    cout << "[DEBUG] Cells sorted by clearance (high->low): ";
    for (int cell_id : sorted_cell_ids_by_clearance)
    {
        auto clearance = (cell_id >= 0 && cell_id < map_size) ? map_clearance[cell_id] : std::make_tuple(-1, -1);
        cout << cell_id << "(" << std::get<0>(clearance) << "," << std::get<1>(clearance) << ") ";
    }
    cout << endl;

    cout << "[DEBUG] Clearance-index in sorted cells from max to min: ";
    for (int i = 0; i < static_cast<int>(sorted_cell_clearance_levels.size()); ++i)
    {
        cout << "[" << sorted_cell_clearance_levels[i] << "->" << sorted_cell_clearance_level_indices[i] << "] ";
    }
    cout << endl;
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
            flexible_task_ids.push_back(task.first);
            task_loc_ids[task.second.locations[0]].push_back(task.first);
            if (task.second.agent_assigned != -1)
                flexible_agent_ids.push_back(task.second.agent_assigned);
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

    // Create worker and task nodes
    for (int i = 0 ; i < env->map.size(); ++i)
    {
        map_nodes[i] = g.addNode();
        int id = lemon::ListDigraphBase::id(map_nodes[i]);
        node_to_maploc[id] = i;
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

    // Keep source arcs to identify which agents are selected by the flow.
    std::vector<ListDigraph::Arc> worker_source_arcs;
    worker_source_arcs.reserve(num_workers);
    // Connect source to workers
    for (int i = 0; i < num_workers; ++i) 
    {
        ListDigraph::Arc a = g.addArc(source, map_nodes[env->curr_states[flexible_agent_ids[i]].location]);
        capacity[a] = 1;
        cost[a] = 0; // No cost for assigning workers

        worker_source_arcs.push_back(a);
    }


    for (auto task: task_loc_ids)
    {
        int loc = task.first;
        ListDigraph::Arc a = g.addArc(map_nodes[loc], sink);
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
            // Prevent right movement from wrapping to the next row.
            if (i == 1 && loc % env->cols == env->cols - 1)
                continue;
            // Prevent left movement from wrapping to the previous row.
            if (i == 3 && loc % env->cols == 0)
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

    // NetworkSimplex setup
    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow); // Use the initial flow (warm start)
    
    
    if (ns.run() == NetworkSimplex<ListDigraph>::OPTIMAL)
    {
        double total_task_movement_cost = 0.0;
        double total_task_movement_steps = 0.0;
        // Copy the solution flow so that we can consume it during extraction.
        ListDigraph::ArcMap<int> remaining_flow(g);

        for (ListDigraph::ArcIt arc(g); arc != INVALID; ++arc)
        {
            remaining_flow[arc] = ns.flow(arc);
        }

        int assigned_count = 0;

        for (int i = 0; i < num_workers; ++i)
        {
            int agent_id = flexible_agent_ids[i];

            // Only agents with positive source flow are selected.
            if (remaining_flow[worker_source_arcs[i]] == 0)
            {
                proposed_schedule[agent_id] = -1;
                env->curr_task_schedule[agent_id] = -1;
                continue;
            }

            // Source-to-worker arcs have capacity 1.
            assert(remaining_flow[worker_source_arcs[i]] == 1);
            --remaining_flow[worker_source_arcs[i]];

            int start_loc = env->curr_states[agent_id].location;
            ListDigraph::Node current = map_nodes[start_loc];

            std::list<int> path;
            int task_loc = -1;
            size_t steps = 0;

            // Follow positive remaining flow until reaching an arc to sink.
            while (task_loc == -1)
            {
                int node_id = lemon::ListDigraphBase::id(current);
                int loc = node_to_maploc.at(node_id);

                path.push_back(loc);

                bool found = false;

                for (ListDigraph::OutArcIt arc(g, current);
                    arc != INVALID; ++arc)
                {
                    if (remaining_flow[arc] <= 0)
                        continue;

                    ListDigraph::Node next = g.target(arc);

                    --remaining_flow[arc];
                    found = true;

                    if (next == sink)
                    {
                        // The flow actually terminates at this task location.
                        task_loc = loc;
                    }
                    else
                    {
                        current = next;
                        total_task_movement_cost += cost[arc];
                        total_task_movement_steps+=1.0;
                    }

                    break;
                }

                if (!found)
                {
                    throw std::runtime_error(
                        "Flow extraction failed for agent " +
                        std::to_string(agent_id)
                    );
                }

                // Positive movement costs imply that an optimal flow
                // contains no circulation cycles.
                if (++steps > env->map.size() + 1)
                {
                    throw std::runtime_error(
                        "Cycle detected during flow extraction"
                    );
                }
            }

            // Find a task associated with the selected pickup location.
            auto it = task_loc_ids.find(task_loc);

            if (it == task_loc_ids.end() || it->second.empty())
            {
                throw std::runtime_error(
                    "No available task at flow destination " +
                    std::to_string(task_loc)
                );
            }

            int task_id = it->second.front();
            it->second.pop_front();

            if (it->second.empty())
            {
                task_loc_ids.erase(it);
            }

            // Update task assignments.
            proposed_schedule[agent_id] = task_id;

            // Preserve the original guide-path behaviour.
            if (use_traffic)
            {
                agent_guide_path[agent_id] = path;
            }

            ++assigned_count;
        }

        // Every unit of source flow should produce one assignment.
        const int expected = std::min(num_workers, num_tasks);

        if (assigned_count != expected)
        {
            throw std::runtime_error(
                "Incorrect number of extracted task assignments"
            );
        }

        // All flow entering sink must have been consumed.
        for (ListDigraph::InArcIt arc(g, sink);
            arc != INVALID; ++arc)
        {
            if (remaining_flow[arc] != 0)
            {
                throw std::runtime_error(
                    "Unconsumed task flow remains at sink"
                );
            }
        }
        if (!use_traffic)
        {
            avg_task_step_cost_for_parking = 1.0;
        }
        else if (total_task_movement_steps > 0)
        {
            avg_task_step_cost_for_parking = total_task_movement_cost / static_cast<double>(total_task_movement_steps);
        }
    }
    else
    {
        std::cout << "No optimal solution found." << std::endl;
    }

    env->curr_task_schedule = proposed_schedule;

    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    cout << "Solving time: " << elapsed_time << " seconds" << endl;

}

void schedule_plan_flow_dummy(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only)
{

    cout<<"run schedule_plan_flow_dummy"<<endl;

    const auto start_time = std::chrono::high_resolution_clock::now();
    const int map_size = static_cast<int>(env->map.size());

    std::vector<int> dummy_agent_ids;
    dummy_agent_ids.reserve(env->num_of_agents);

    assert(static_cast<int>(env->curr_task_schedule.size()) == env->num_of_agents);
    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id)
    {
        if (env->curr_task_schedule[agent_id] == -1)
            dummy_agent_ids.push_back(agent_id);
    }

    const int num_workers = static_cast<int>(dummy_agent_ids.size());
    if (num_workers == 0)
    {
        env->dummy_parking_allocation.clear();
        return;
    }

    // Identify task locations that are currently protected by assigned agents.
    std::unordered_set<int> protected_task_locs;

    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id)
    {
        int task_id = env->curr_task_schedule[agent_id];

        if (task_id == -1)
            continue;

        auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end())
            continue;

        const auto& task = it->second;
        int next_idx = task.idx_next_loc;

        if (next_idx >= 0 &&
            next_idx < static_cast<int>(task.locations.size()))
        {
            protected_task_locs.insert(task.locations[next_idx]);
        }
    }

    // Stage 1: select the top-m locations in the existing clearance ranking.
    // The candidate set can be larger than the number of idle agents.
    constexpr double candidate_ratio = 1.1;
    const int requested_targets = static_cast<int>(std::ceil(candidate_ratio * num_workers));

    std::vector<int> selected_targets;
    selected_targets.reserve(std::min(requested_targets, map_size));

    for (int cell_id : sorted_cell_ids_by_clearance)
    {
        if (cell_id < 0 || cell_id >= map_size)
            throw std::runtime_error("Invalid cell in parking ranking");
        if (env->map[cell_id] == 1)
            continue;

        //do not park on the cell that is currently a task location for an assigned agent
        if (protected_task_locs.find(cell_id) != protected_task_locs.end())
            continue;

        selected_targets.push_back(cell_id);
        if (static_cast<int>(selected_targets.size()) >= requested_targets)
            break;
    }

    // Stage 2: min-cost flow from idle agents to the chosen candidates.
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();

    std::vector<ListDigraph::Node> map_nodes(map_size);
    std::unordered_map<int, int> node_to_maploc;
    node_to_maploc.reserve(map_size);

    supply[source] = num_workers;
    supply[sink] = -num_workers;
    for (int loc = 0; loc < map_size; ++loc)
    {
        map_nodes[loc] = g.addNode();
        supply[map_nodes[loc]] = 0;
        node_to_maploc[lemon::ListDigraphBase::id(map_nodes[loc])] = loc;
    }

    // Keep the source arc for each idle agent (including agents sharing a cell).
    std::vector<ListDigraph::Arc> worker_source_arcs;
    worker_source_arcs.reserve(num_workers);
    for (int agent_id : dummy_agent_ids)
    {
        const int start_loc = env->curr_states[agent_id].location;
        if (start_loc < 0 || start_loc >= map_size || env->map[start_loc] == 1)
            throw std::runtime_error("Invalid idle agent position");

        const ListDigraph::Arc arc = g.addArc(source, map_nodes[start_loc]);
        capacity[arc] = 1;
        cost[arc] = 0;
        worker_source_arcs.push_back(arc);
    }

    // Connect ALL selected candidates to the sink, not only num_workers.
    // Parking quality is a filter here; the sink cost remains zero.
    for (int target_loc : selected_targets)
    {
        const ListDigraph::Arc arc = g.addArc(map_nodes[target_loc], sink);
        capacity[arc] = 1;
        cost[arc] = 0;
    }

    const int offsets[4] = {-env->cols, 1, env->cols, -1};
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1)
            continue;

        for (int dir = 0; dir < 4; ++dir)
        {
            // A linear index check alone does not prevent row wrapping.
            if (dir == 1 && loc % env->cols == env->cols - 1)
                continue;
            if (dir == 3 && loc % env->cols == 0)
                continue;

            const int neighbor_loc = loc + offsets[dir];
            if (neighbor_loc < 0 || neighbor_loc >= map_size ||
                env->map[neighbor_loc] == 1)
                continue;

            const ListDigraph::Arc arc =
                g.addArc(map_nodes[loc], map_nodes[neighbor_loc]);

            if (use_traffic)
            {
                // Preserve the traffic-cost calculation from the original
                // function so the rewrite does not change the objective.
                const int d = get_d(loc - neighbor_loc, env);
                const int temp_op =
                    (background_flow[loc].d[d] + 1) *
                    background_flow[neighbor_loc].d[(d + 2) % 4];

                int temp_vertex = 1;
                for (int j = 0; j < 4; ++j)
                    temp_vertex += background_flow[neighbor_loc].d[j];

                cost[arc] = 1 + temp_op + (temp_vertex - 1) / 2;
            }
            else
            {
                cost[arc] = 1;
            }
            capacity[arc] = num_workers;
        }
    }

    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);

    if (ns.run() != NetworkSimplex<ListDigraph>::OPTIMAL)
        throw std::runtime_error("No optimal dummy parking flow solution");

    // Stage 3: decompose actual positive flow all the way to the sink.
    ListDigraph::ArcMap<int> remaining_flow(g);
    for (ListDigraph::ArcIt arc(g); arc != INVALID; ++arc)
        remaining_flow[arc] = ns.flow(arc);

    std::vector<std::pair<int, int>> new_allocations;
    std::vector<std::pair<int, std::list<int>>> new_guide_paths;
    std::unordered_set<int> used_targets;
    new_allocations.reserve(num_workers);
    new_guide_paths.reserve(num_workers);
    used_targets.reserve(num_workers);

    for (int i = 0; i < num_workers; ++i)
    {
        const int agent_id = dummy_agent_ids[i];
        const ListDigraph::Arc source_arc = worker_source_arcs[i];

        // Unlike task assignment, every idle agent must be assigned.
        if (remaining_flow[source_arc] != 1)
            throw std::runtime_error("Parking flow did not select every idle agent");
        --remaining_flow[source_arc];

        ListDigraph::Node current =
            map_nodes[env->curr_states[agent_id].location];
        std::list<int> path;
        int target_loc = -1;
        int steps = 0;

        while (target_loc == -1)
        {
            const int loc = node_to_maploc.at(
                lemon::ListDigraphBase::id(current));
            path.push_back(loc);

            bool found = false;
            for (ListDigraph::OutArcIt arc(g, current);
                 arc != INVALID; ++arc)
            {
                if (remaining_flow[arc] <= 0)
                    continue;

                const ListDigraph::Node next = g.target(arc);
                --remaining_flow[arc];
                found = true;

                if (next == sink)
                    target_loc = loc;
                else
                    current = next;
                break;
            }

            if (!found)
                throw std::runtime_error("Could not extract complete parking flow");
            if (++steps > map_size + 1)
                throw std::runtime_error("Cycle detected in parking flow extraction");
        }

        if (!used_targets.insert(target_loc).second)
            throw std::runtime_error("Duplicate parking destination extracted");

        new_allocations.emplace_back(agent_id, target_loc);
        new_guide_paths.emplace_back(agent_id, std::move(path));
    }

    if (static_cast<int>(new_allocations.size()) != num_workers)
        throw std::runtime_error("Wrong number of extracted parking assignments");

    for (ListDigraph::InArcIt arc(g, sink); arc != INVALID; ++arc)
    {
        if (remaining_flow[arc] != 0)
            throw std::runtime_error("Unconsumed parking flow remains at sink");
    }

    // Commit only after successfully extracting the entire solution.
    // Do not clear agent_guide_path: it also holds paths for working agents.
    env->dummy_parking_allocation.clear();
    for (const auto& allocation : new_allocations)
        env->dummy_parking_allocation[allocation.first] = {allocation.second};
    for (const auto& entry : new_guide_paths)
        agent_guide_path[entry.first] = entry.second;

    const auto end_time = std::chrono::high_resolution_clock::now();
    const double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    std::cout << "Dummy parking flow solving time: "<< elapsed_time << " seconds" << std::endl;
}


// void schedule_plan_flow_dummy_all(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only)
// {

//     std::vector<int> dummy_agent_ids;
//     dummy_agent_ids.reserve(env->num_of_agents);
//     assert(static_cast<int>(env->curr_task_schedule.size()) == env->num_of_agents);
//     for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id)
//     {
//         if (env->curr_task_schedule[agent_id] == -1)
//             dummy_agent_ids.push_back(agent_id);
//     }

//     env->dummy_parking_allocation.clear();
//     for (int agent_id : dummy_agent_ids)
//     {
//         env->dummy_parking_allocation[agent_id] = {};
//     }

//     auto start_time = std::chrono::high_resolution_clock::now();

//     assert(!dummy_agent_ids.empty());
//     assert(!sorted_cell_ids_by_clearance.empty());
//     if (dummy_agent_ids.empty() || sorted_cell_ids_by_clearance.empty())
//     {
//         return;
//     }

//     assert(static_cast<int>(sorted_cell_ids_by_clearance.size()) == static_cast<int>(env->map.size()));
//     refresh_sorted_cells_by_clearance(env, 1);

//     std::vector<int> selected_targets;
//     int selected_target_size = static_cast<int>(sorted_cell_ids_by_clearance.size());
//     selected_targets.reserve(selected_target_size);
//     for (int cell_id : sorted_cell_ids_by_clearance)
//     {
//         assert(cell_id >= 0 && cell_id < static_cast<int>(env->map.size()));
//         if (env->map[cell_id] == 1)
//             continue;
//         selected_targets.push_back(cell_id);
//         if (selected_targets.size() >= selected_target_size)
//             break;
//     }

//     assert(selected_targets.size() >= dummy_agent_ids.size());
//     if (selected_targets.size() < dummy_agent_ids.size())
//     {
//         return;
//     }

//     const int num_workers = static_cast<int>(dummy_agent_ids.size());

//     ListDigraph g;
//     ListDigraph::NodeMap<int> supply(g);
//     ListDigraph::ArcMap<double> cost(g);
//     ListDigraph::ArcMap<int> capacity(g);
//     ListDigraph::ArcMap<int> flow(g);

//     std::vector<ListDigraph::Node> map_nodes(env->map.size());
//     ListDigraph::Node source = g.addNode();
//     ListDigraph::Node sink = g.addNode();

//     std::unordered_map<int, int> node_to_maploc;
//     for (int i = 0; i < static_cast<int>(env->map.size()); ++i)
//     {
//         map_nodes[i] = g.addNode();
//         node_to_maploc[lemon::ListDigraphBase::id(map_nodes[i])] = i;
//     }

//     supply[source] = num_workers;
//     supply[sink] = -num_workers;

//     for (int i = 0; i < static_cast<int>(env->map.size()); ++i)
//     {
//         supply[map_nodes[i]] = 0;
//     }

//     for (int agent_id : dummy_agent_ids)
//     {
//         int start_loc = env->curr_states[agent_id].location;
//         assert(start_loc >= 0 && start_loc < static_cast<int>(env->map.size()));
//         ListDigraph::Arc a = g.addArc(source, map_nodes[start_loc]);
//         capacity[a] = 1;
//         cost[a] = 0;
//     }

//     std::unordered_map<int, int> node_to_target_loc;
//     for (int i = 0; i < num_workers; ++i)
//     {
//         int target_loc = selected_targets[i];
//         ListDigraph::Arc a = g.addArc(map_nodes[target_loc], sink);
//         node_to_target_loc[lemon::ListDigraphBase::id(map_nodes[target_loc])] = target_loc;
//         capacity[a] = 1;
//         // Clearance priority >=4: 0; >=3: 2; >=2: 4; >= 1: 16
//         int target_clearance = std::get<0>(map_clearance[target_loc]);
//         if (target_clearance >= 4)
//             cost[a] = 0;
//         else if (target_clearance == 3)
//             cost[a] = 2;
//         else if (target_clearance == 2)
//             cost[a] = 4;
//         else
//             cost[a] = 16;
//     }

//     std::vector<int> neighbor = {-env->cols, 1, env->cols, -1};
//     int diff, d, op_flow, all_vertex_flow;
//     int temp_op, temp_vertex;

//     for (int loc = 0; loc < static_cast<int>(env->map.size()); ++loc)
//     {
//         if (env->map[loc] == 1)
//             continue;

//         for (int i = 0; i < 4; ++i)
//         {
//             int neighbor_loc = loc + neighbor[i];
//             if (neighbor_loc < 0 || neighbor_loc >= static_cast<int>(env->map.size()) || env->map[neighbor_loc] == 1)
//                 continue;

//             ListDigraph::Arc a = g.addArc(map_nodes[loc], map_nodes[neighbor_loc]);

//             if (use_traffic)
//             {
//                 op_flow = 0;
//                 all_vertex_flow = 0;
//                 diff = loc - neighbor_loc;
//                 d = get_d(diff, env);
//                 temp_op = ((background_flow[loc].d[d] + 1) * background_flow[neighbor_loc].d[(d + 2) % 4]);
//                 temp_vertex = 1;
//                 for (int j = 0; j < 4; ++j)
//                 {
//                     temp_vertex += background_flow[neighbor_loc].d[j];
//                 }
//                 op_flow += temp_op;
//                 all_vertex_flow += (temp_vertex - 1) / 2;
//                 cost[a] = 1 + op_flow + all_vertex_flow;
//             }
//             else
//             {
//                 cost[a] = 1;
//             }

//             capacity[a] = num_workers;
//         }
//     }

//     NetworkSimplex<ListDigraph> ns(g);
//     ns.costMap(cost);
//     ns.upperMap(capacity);
//     ns.supplyMap(supply);
//     ns.flowMap(flow);

//     if (ns.run() != NetworkSimplex<ListDigraph>::OPTIMAL)
//     {
//         cout << "No optimal dummy parking solution found." << endl;
//         return;
//     }

//     std::unordered_map<int, int> edge_flows;
//     for (int agent_id : dummy_agent_ids)
//     {
//         int start_loc = env->curr_states[agent_id].location;
//         assert(start_loc >= 0 && start_loc < static_cast<int>(env->map.size()));
//         ListDigraph::Node current = map_nodes[start_loc];
//         list<int> path;
//         int cost = 0;

//         while (node_to_target_loc.find(lemon::ListDigraphBase::id(current)) == node_to_target_loc.end())
//         {
//             if (current == sink)
//                 break;

//             int loc = node_to_maploc[lemon::ListDigraphBase::id(current)];
//             path.push_back(loc);
//             cost++;

//             bool found = false;
//             for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc)
//             {
//                 if (ns.flow(arc) > 0)
//                 {
//                     int arc_id = lemon::ListDigraphBase::id(arc);
//                     if (edge_flows.find(arc_id) == edge_flows.end())
//                     {
//                         edge_flows[arc_id] = ns.flow(arc);
//                     }
//                     if (edge_flows[arc_id] <= 0)
//                         continue;

//                     current = g.target(arc);
//                     edge_flows[arc_id]--;
//                     found = true;
//                     break;
//                 }
//             }

//             if (!found)
//                 break;
//         }

//         if (node_to_target_loc.find(lemon::ListDigraphBase::id(current)) != node_to_target_loc.end())
//         {
//             int target_loc = node_to_target_loc[lemon::ListDigraphBase::id(current)];
//             path.push_back(target_loc);
//             agent_guide_path[agent_id] = path;
//             env->dummy_parking_allocation[agent_id] = {target_loc};
//             cost++;
//             //cout << "Agent " << agent_id << " is assigned to dummy parking location " << target_loc << " with clearance of " << std::get<0>(map_clearance[target_loc]) << " and cost of " << cost << endl;
//         }
//     }

//     auto end_time = std::chrono::high_resolution_clock::now();
//     double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
//     cout << "Dummy parking flow solving time: " << elapsed_time << " seconds" << endl;

// }
void schedule_plan_flow_dummy_all(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only)
{

    cout<<"run schedule_plan_flow_dummy_all"<<endl;

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const int map_size = static_cast<int>(env->map.size());

    constexpr double B = 16.0;
    constexpr double gamma = 2.0;
    constexpr long long COST_SCALE = 1000LL;
    using ParkingFlow = NetworkSimplex<ListDigraph, int, long long>;

    // Stage 1: identify idle agents and protect the next location of each
    // working agent's currently assigned task. Those protected locations
    // remain traversable but cannot be parking destinations.
    std::vector<int> idle_agents;
    idle_agents.reserve(env->num_of_agents);
    std::unordered_set<int> protected_task_locs;

    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id)
    {
        const int task_id = env->curr_task_schedule[agent_id];
        if (task_id == -1)
        {
            idle_agents.push_back(agent_id);
            continue;
        }

        const auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end())
            throw std::runtime_error("Current task assignment not found in task_pool");

        const auto& task = it->second;
        const int next_idx = task.idx_next_loc;
        if (next_idx >= 0 &&
            next_idx < static_cast<int>(task.locations.size()))
        {
            protected_task_locs.insert(task.locations[next_idx]);
        }
    }

    const int num_workers = static_cast<int>(idle_agents.size());
    if (num_workers == 0)
    {
        env->dummy_parking_allocation.clear();
        return;
    }

    // One average-cost-step estimate for all destinations. The task flow
    // updates this during successful assignment; unit-cost parking uses 1.
    const double avg_step_cost = use_traffic ? avg_task_step_cost_for_parking : 1.0;

    // Stage 2: build the complete spatial flow graph.
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::NodeMap<int> node_to_maploc(g, -1);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<long long> cost(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();
    supply[source] = num_workers;
    supply[sink] = -num_workers;

    std::vector<ListDigraph::Node> map_nodes(map_size);
    for (int loc = 0; loc < map_size; ++loc)
    {
        const ListDigraph::Node node = g.addNode();
        map_nodes[loc] = node;
        node_to_maploc[node] = loc;
        supply[node] = 0;
    }

    std::vector<ListDigraph::Arc> worker_source_arcs;
    worker_source_arcs.reserve(num_workers);

    for (int agent_id : idle_agents)
    {
        const int start_loc = env->curr_states[agent_id].location;
        const ListDigraph::Arc arc = g.addArc(source, map_nodes[start_loc]);
        capacity[arc] = 1;
        cost[arc] = 0;
        worker_source_arcs.push_back(arc);
    }

    // Allow ALL traversable, unprotected cells to be selected.
    int candidate_count = 0;
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1 || protected_task_locs.count(loc))
            continue;

        const int c_int = std::get<0>(map_clearance[loc]);
        const int o_int = std::get<1>(map_clearance[loc]);
        if (c_int < 1 || o_int < 0)
            throw std::runtime_error("Invalid static clearance score");

        const double c = static_cast<double>(c_int);
        const double o = static_cast<double>(o_int);
        const double base = std::pow(c, -gamma);
        const double next = std::pow(c + 1.0, -gamma);
        const double obstacle_fraction = o / (8.0 * c);

        if (obstacle_fraction < 0.0 || obstacle_fraction > 1.0 + 1e-9)
            throw std::runtime_error("Invalid clearance obstacle fraction");

        const double penalty = B * avg_step_cost *
            (base + obstacle_fraction * (base - next));
        const double scaled = penalty * static_cast<double>(COST_SCALE);
        if (!std::isfinite(scaled) ||
            scaled >= static_cast<double>(std::numeric_limits<long long>::max()) / 8.0)
            throw std::runtime_error("Parking penalty is too large");

        const ListDigraph::Arc arc = g.addArc(map_nodes[loc], sink);
        capacity[arc] = 1;
        cost[arc] = std::llround(scaled);
        ++candidate_count;
    }

    const int offsets[4] = {-env->cols, 1, env->cols, -1};
    int movement_arc_count = 0;

    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1)
            continue;

        for (int dir = 0; dir < 4; ++dir)
        {
            // Avoid accidental horizontal wrap between consecutive rows.
            if (dir == 1 && loc % env->cols == env->cols - 1)
                continue;
            if (dir == 3 && loc % env->cols == 0)
                continue;

            const int next_loc = loc + offsets[dir];
            if (next_loc < 0 || next_loc >= map_size || env->map[next_loc] == 1)
                continue;

            // Intentionally preserve the integer traffic-cost arithmetic used
            // in the existing task-flow and dummy-flow implementations.
            int movement_cost = 1;
            if (use_traffic)
            {
                const int d = get_d(loc - next_loc, env);
                const int temp_op =
                    ((background_flow[loc].d[d] + 1) *
                      background_flow[next_loc].d[(d + 2) % 4]);
                int temp_vertex = 1;
                for (int j = 0; j < 4; ++j)
                    temp_vertex += background_flow[next_loc].d[j];

                movement_cost += temp_op + (temp_vertex - 1) / 2;
            }

            if (movement_cost <= 0)
                throw std::runtime_error("Non-positive movement arc cost");

            const ListDigraph::Arc arc =
                g.addArc(map_nodes[loc], map_nodes[next_loc]);
            capacity[arc] = num_workers;
            cost[arc] = COST_SCALE * static_cast<long long>(movement_cost);
            ++movement_arc_count;
        }
    }

    const auto t_graph_end = Clock::now();

    // Stage 3: solve with integer-scaled 64-bit costs. NetworkSimplex's
    // default integer cost type would truncate fractional sink penalties.
    ParkingFlow ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    const auto t_solver_start = Clock::now();
    const auto status = ns.run();
    const auto t_solver_end = Clock::now();

    const auto ms = [](Clock::time_point from, Clock::time_point to)
    {
        return std::chrono::duration<double, std::milli>(to - from).count();
    };

    if (status != ParkingFlow::OPTIMAL)
    {
        std::cout << "[parking-all] solver status=" << static_cast<int>(status)
                  << " idle=" << num_workers
                  << " candidates=" << candidate_count
                  << " graph_ms=" << ms(t0, t_graph_end)
                  << " solver_ms=" << ms(t_solver_start, t_solver_end)
                  << std::endl;
        throw std::runtime_error("No optimal full-candidate parking flow solution");
    }

    // Stage 4: extract actual positive-flow paths through parking-to-sink
    // arcs. Do not stop merely because a path passes a candidate node.
    ListDigraph::ArcMap<int> remaining_flow(g);
    ns.flowMap(remaining_flow);

    std::vector<std::pair<int, int>> allocations;
    std::vector<std::pair<int, std::list<int>>> guide_paths;
    std::unordered_set<int> used_targets;
    allocations.reserve(num_workers);
    guide_paths.reserve(num_workers);
    used_targets.reserve(num_workers);

    long long total_relocation_steps = 0;
    int moved_agents = 0;
    double total_selected_clearance = 0.0;

    for (int i = 0; i < num_workers; ++i)
    {
        const int agent_id = idle_agents[i];
        const ListDigraph::Arc source_arc = worker_source_arcs[i];

        if (remaining_flow[source_arc] != 1)
            throw std::runtime_error("Parking flow did not select every idle agent");
        --remaining_flow[source_arc];

        const int start_loc = env->curr_states[agent_id].location;
        ListDigraph::Node current = map_nodes[start_loc];
        std::list<int> path;
        int target_loc = -1;
        int steps = 0;

        while (target_loc == -1)
        {
            const int loc = node_to_maploc[current];
            if (loc < 0 || loc >= map_size)
                throw std::runtime_error("Unexpected node in flow extraction");
            path.push_back(loc);

            bool found = false;
            for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc)
            {
                if (remaining_flow[arc] <= 0)
                    continue;

                const ListDigraph::Node next = g.target(arc);
                --remaining_flow[arc];
                found = true;

                if (next == sink)
                    target_loc = loc;
                else
                    current = next;
                break;
            }

            if (!found)
                throw std::runtime_error("Could not extract a complete parking path");
            if (++steps > map_size + 1)
                throw std::runtime_error("Unexpected cycle in parking flow decomposition");
        }

        if (!used_targets.insert(target_loc).second)
            throw std::runtime_error("Duplicate parking destination");
        if (protected_task_locs.count(target_loc))
            throw std::runtime_error("Parking assigned to a protected task location");

        total_relocation_steps += static_cast<long long>(path.size()) - 1;
        moved_agents += (target_loc != start_loc);
        total_selected_clearance +=
            static_cast<double>(std::get<0>(map_clearance[target_loc]));

        allocations.emplace_back(agent_id, target_loc);
        guide_paths.emplace_back(agent_id, std::move(path));
    }

    for (ListDigraph::InArcIt arc(g, sink); arc != INVALID; ++arc)
    {
        if (remaining_flow[arc] != 0)
            throw std::runtime_error("Unconsumed parking-to-sink flow");
    }
    const auto t_extract_end = Clock::now();

    // Atomic commit: do not modify task-agent guide paths and do not clear
    // the previous parking allocation if decomposition fails.
    env->dummy_parking_allocation.clear();
    for (const auto& entry : allocations)
        env->dummy_parking_allocation[entry.first] = {entry.second};
    for (const auto& entry : guide_paths)
        agent_guide_path[entry.first] = entry.second;

    const auto t_end = Clock::now();

    std::cout << "[parking-all]"
              << " idle=" << num_workers
              << " candidates=" << candidate_count
              << " map_nodes=" << map_size
              << " avg_step_cost=" << avg_step_cost
              << " graph_ms=" << ms(t0, t_graph_end)
              << " setup_ms=" << ms(t_graph_end, t_solver_start)
              << " solve_ms=" << ms(t_solver_start, t_solver_end)
              << " extract_ms=" << ms(t_solver_end, t_extract_end)
              << " commit_ms=" << ms(t_extract_end, t_end)
              << " total_ms=" << ms(t0, t_end)
              << " moved=" << moved_agents
              << " relocation_steps=" << total_relocation_steps
              << " avg_clearance=" << total_selected_clearance / num_workers
              << std::endl;
}

// Traffic-only full-candidate parking using min-cost flow.
//
// Signature adds double B_T as last argument. Add matching declaration to
// scheduler.h and pass B_T from the experiment configuration.

void schedule_plan_flow_dummy_traffic(
    int time_limit,
    SharedEnvironment* env,
    std::vector<Double4> background_flow,
    bool use_traffic,
    bool new_only,
    double B_T)
{

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const int map_size = static_cast<int>(env->map.size());

    constexpr long long COST_SCALE = 1000LL;
    using ParkingFlow = NetworkSimplex<ListDigraph, int, long long>;

    // Stage 1: identify idle agents and protect the next location of each
    // working agent's currently assigned task. Those protected locations
    // remain traversable but cannot be parking destinations.
    std::vector<int> idle_agents;
    idle_agents.reserve(env->num_of_agents);
    std::unordered_set<int> protected_task_locs;

    for (int agent_id = 0; agent_id < env->num_of_agents; ++agent_id)
    {
        const int task_id = env->curr_task_schedule[agent_id];
        if (task_id == -1)
        {
            idle_agents.push_back(agent_id);
            continue;
        }

        const auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end())
            throw std::runtime_error("Current task assignment not found in task_pool");

        const auto& task = it->second;
        const int next_idx = task.idx_next_loc;
        if (next_idx >= 0 &&
            next_idx < static_cast<int>(task.locations.size()))
        {
            protected_task_locs.insert(task.locations[next_idx]);
        }
    }

    const int num_workers = static_cast<int>(idle_agents.size());
    if (num_workers == 0)
    {
        env->dummy_parking_allocation.clear();
        return;
    }

    // One average-cost-step estimate for all destinations. The task flow
    // updates this during successful assignment; unit-cost parking uses 1.
    const double avg_step_cost =
        use_traffic ? avg_task_step_cost_for_parking : 1.0;
    if (!std::isfinite(avg_step_cost) || avg_step_cost <= 0.0)
        throw std::runtime_error("Invalid average task-flow step cost");

    // Stage 2: build the complete spatial flow graph.
    // Traffic-only parking penalty: P_T(loc) = B_T * avg_step_cost * T(loc)^2.
    // Traffic T(loc) sums all four directional components of background_flow.
    // We do NOT need the static or iteratively updated clearance map.
    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::NodeMap<int> node_to_maploc(g, -1);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<long long> cost(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();
    supply[source] = num_workers;
    supply[sink] = -num_workers;

    std::vector<ListDigraph::Node> map_nodes(map_size);
    for (int loc = 0; loc < map_size; ++loc)
    {
        const ListDigraph::Node node = g.addNode();
        map_nodes[loc] = node;
        node_to_maploc[node] = loc;
        supply[node] = 0;
    }

    std::vector<ListDigraph::Arc> worker_source_arcs;
    worker_source_arcs.reserve(num_workers);

    for (int agent_id : idle_agents)
    {
        const int start_loc = env->curr_states[agent_id].location;
        if (start_loc < 0 || start_loc >= map_size || env->map[start_loc] == 1)
            throw std::runtime_error("Invalid idle agent position");

        const ListDigraph::Arc arc = g.addArc(source, map_nodes[start_loc]);
        capacity[arc] = 1;
        cost[arc] = 0;
        worker_source_arcs.push_back(arc);
    }

    // Precompute vertex traffic once, outside the candidate and arc loops.
    // This is spatial path-usage intensity, not simultaneous robot occupancy.
    std::vector<double> vertex_traffic(map_size, 0.0);
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1)
            continue;
        for (int d = 0; d < 4; ++d)
            vertex_traffic[loc] += background_flow[loc].d[d];
        if (!std::isfinite(vertex_traffic[loc]) || vertex_traffic[loc] < 0.0)
            throw std::runtime_error("Invalid background vertex traffic");
    }

    // Allow ALL traversable, unprotected cells to be selected.
    int candidate_count = 0;
    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1 || protected_task_locs.count(loc))
            continue;

        const double T = vertex_traffic[loc];
        const double penalty = B_T * avg_step_cost * T * T;
        const double scaled = penalty * static_cast<double>(COST_SCALE);
        if (!std::isfinite(scaled) ||
            scaled >= static_cast<double>(std::numeric_limits<long long>::max()) / 8.0)
            throw std::runtime_error("Parking traffic penalty is too large");

        const ListDigraph::Arc arc = g.addArc(map_nodes[loc], sink);
        capacity[arc] = 1;
        cost[arc] = std::llround(scaled);
        ++candidate_count;
    }

    if (candidate_count < num_workers)
        throw std::runtime_error("Not enough eligible parking destinations");

    const int offsets[4] = {-env->cols, 1, env->cols, -1};
    int movement_arc_count = 0;

    for (int loc = 0; loc < map_size; ++loc)
    {
        if (env->map[loc] == 1)
            continue;

        for (int dir = 0; dir < 4; ++dir)
        {
            // Avoid accidental horizontal wrap between consecutive rows.
            if (dir == 1 && loc % env->cols == env->cols - 1)
                continue;
            if (dir == 3 && loc % env->cols == 0)
                continue;

            const int next_loc = loc + offsets[dir];
            if (next_loc < 0 || next_loc >= map_size || env->map[next_loc] == 1)
                continue;

            // Intentionally preserve the integer traffic-cost arithmetic used
            // in the existing task-flow and dummy-flow implementations.
            int movement_cost = 1;
            if (use_traffic)
            {
                const int d = get_d(loc - next_loc, env);
                const int temp_op =
                    ((background_flow[loc].d[d] + 1) *
                      background_flow[next_loc].d[(d + 2) % 4]);
                int temp_vertex = 1;
                for (int j = 0; j < 4; ++j)
                    temp_vertex += background_flow[next_loc].d[j];

                movement_cost += temp_op + (temp_vertex - 1) / 2;
            }

            if (movement_cost <= 0)
                throw std::runtime_error("Non-positive movement arc cost");

            const ListDigraph::Arc arc =
                g.addArc(map_nodes[loc], map_nodes[next_loc]);
            capacity[arc] = num_workers;
            cost[arc] = COST_SCALE * static_cast<long long>(movement_cost);
            ++movement_arc_count;
        }
    }

    const auto t_graph_end = Clock::now();

    // Stage 3: solve with integer-scaled 64-bit costs. NetworkSimplex's
    // default integer cost type would truncate fractional sink penalties.
    ParkingFlow ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    const auto t_solver_start = Clock::now();
    const auto status = ns.run();
    const auto t_solver_end = Clock::now();

    const auto ms = [](Clock::time_point from, Clock::time_point to)
    {
        return std::chrono::duration<double, std::milli>(to - from).count();
    };

    if (status != ParkingFlow::OPTIMAL)
    {
        std::cout << "[parking-traffic] solver status=" << static_cast<int>(status)
                  << " idle=" << num_workers
                  << " candidates=" << candidate_count
                  << " graph_ms=" << ms(t0, t_graph_end)
                  << " solver_ms=" << ms(t_solver_start, t_solver_end)
                  << std::endl;
        throw std::runtime_error("No optimal traffic-only parking flow solution");
    }

    // Stage 4: extract actual positive-flow paths through parking-to-sink
    // arcs. Do not stop merely because a path passes a candidate node.
    ListDigraph::ArcMap<int> remaining_flow(g);
    ns.flowMap(remaining_flow);

    std::vector<std::pair<int, int>> allocations;
    std::vector<std::pair<int, std::list<int>>> guide_paths;
    std::unordered_set<int> used_targets;
    allocations.reserve(num_workers);
    guide_paths.reserve(num_workers);
    used_targets.reserve(num_workers);

    long long total_relocation_steps = 0;
    int moved_agents = 0;
    double total_selected_traffic = 0.0;

    for (int i = 0; i < num_workers; ++i)
    {
        const int agent_id = idle_agents[i];
        const ListDigraph::Arc source_arc = worker_source_arcs[i];

        if (remaining_flow[source_arc] != 1)
            throw std::runtime_error("Parking flow did not select every idle agent");
        --remaining_flow[source_arc];

        const int start_loc = env->curr_states[agent_id].location;
        ListDigraph::Node current = map_nodes[start_loc];
        std::list<int> path;
        int target_loc = -1;
        int steps = 0;

        while (target_loc == -1)
        {
            const int loc = node_to_maploc[current];
            if (loc < 0 || loc >= map_size)
                throw std::runtime_error("Unexpected node in flow extraction");
            path.push_back(loc);

            bool found = false;
            for (ListDigraph::OutArcIt arc(g, current); arc != INVALID; ++arc)
            {
                if (remaining_flow[arc] <= 0)
                    continue;

                const ListDigraph::Node next = g.target(arc);
                --remaining_flow[arc];
                found = true;

                if (next == sink)
                    target_loc = loc;
                else
                    current = next;
                break;
            }

            if (!found)
                throw std::runtime_error("Could not extract a complete parking path");
            if (++steps > map_size + 1)
                throw std::runtime_error("Unexpected cycle in parking flow decomposition");
        }

        if (!used_targets.insert(target_loc).second)
            throw std::runtime_error("Duplicate parking destination");
        if (protected_task_locs.count(target_loc))
            throw std::runtime_error("Parking assigned to a protected task location");

        total_relocation_steps += static_cast<long long>(path.size()) - 1;
        moved_agents += (target_loc != start_loc);
        total_selected_traffic += vertex_traffic[target_loc];

        allocations.emplace_back(agent_id, target_loc);
        guide_paths.emplace_back(agent_id, std::move(path));
    }

    for (ListDigraph::InArcIt arc(g, sink); arc != INVALID; ++arc)
    {
        if (remaining_flow[arc] != 0)
            throw std::runtime_error("Unconsumed parking-to-sink flow");
    }
    const auto t_extract_end = Clock::now();

    // Atomic commit: do not modify task-agent guide paths and do not clear
    // the previous parking allocation if decomposition fails.
    env->dummy_parking_allocation.clear();
    for (const auto& entry : allocations)
        env->dummy_parking_allocation[entry.first] = {entry.second};
    for (const auto& entry : guide_paths)
        agent_guide_path[entry.first] = entry.second;

    const auto t_end = Clock::now();

    std::cout << "[parking-traffic]"
              << " idle=" << num_workers
              << " candidates=" << candidate_count
              << " map_nodes=" << map_size
              << " movement_arcs=" << movement_arc_count
              << " total_arcs=" << (num_workers + candidate_count + movement_arc_count)
              << " avg_step_cost=" << avg_step_cost
              << " B_T=" << B_T
              << " graph_ms=" << ms(t0, t_graph_end)
              << " setup_ms=" << ms(t_graph_end, t_solver_start)
              << " solve_ms=" << ms(t_solver_start, t_solver_end)
              << " extract_ms=" << ms(t_solver_end, t_extract_end)
              << " commit_ms=" << ms(t_extract_end, t_end)
              << " total_ms=" << ms(t0, t_end)
              << " moved=" << moved_agents
              << " relocation_steps=" << total_relocation_steps
              << " avg_target_traffic=" << total_selected_traffic / num_workers
              << std::endl;
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

const std::vector<int>& get_sorted_cell_ids_by_clearance()
{
    return sorted_cell_ids_by_clearance;
}

};
