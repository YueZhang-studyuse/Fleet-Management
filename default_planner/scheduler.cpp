#include "scheduler.h"
// #include "gurobi_c++.h"
#include <algorithm>
#include <boost/heap/pairing_heap.hpp>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
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

// Represent fractional parking costs to the nearest 0.01 cost unit.
constexpr long long kParkingCostScale = 100LL;
constexpr double kParkingGamma = 2.0;

struct ParkingWeights {
    double B;
    double b_c;
    double b_t;
};

// avg_task_step_cost_for_parking is the current mean for secondary parking,
// or the last known mean for one-flow parking. Declared once above.

// One immutable map per simulation. Initialized eagerly for clearance modes,
// or on first use when a combined parking call enables its clearance term.
static bool parking_clearance_ready = false;

// Check a numeric cost before converting to a 64-bit LEMON arc cost.
// Conservative bound protects network-simplex's accumulation of costs in
// large instances. Exceeding it is a configuration error, not an invitation
// to silently clamp a penalty (which would change the objective).
long long scale_parking_arc_cost(double unscaled, double safe_scaled_cost)
{
    if (!std::isfinite(unscaled) || unscaled < 0.0)
        throw std::invalid_argument("Parking arc cost must be finite and non-negative");
    if (!std::isfinite(safe_scaled_cost) || safe_scaled_cost <= 0.0)
        throw std::invalid_argument("Parking arc cost bound must be finite and positive");

    const double scaled = unscaled * static_cast<double>(kParkingCostScale);
    if (!std::isfinite(scaled))
        throw std::overflow_error("Scaled parking arc cost is not finite");

    // Round in floating point so out-of-range values never reach an integer
    // conversion. Check the rounded value too: rounding can cross the bound.
    const double rounded = std::round(scaled);
    if (rounded >= safe_scaled_cost ||
        rounded >= static_cast<double>(std::numeric_limits<long long>::max()))
        throw std::overflow_error("Parking arc cost exceeds safe int64 range");
    return static_cast<long long>(rounded);
}

static void validate_parking_weights(const ParkingWeights& weights)
{
    for (double value : {weights.B, weights.b_c, weights.b_t})
        if (!std::isfinite(value) || value < 0.0)
            throw std::invalid_argument("Parking weights must be finite and non-negative");
}

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

static void ensure_parking_clearance(const SharedEnvironment* env)
{
    if (parking_clearance_ready)
        return;

    const int rows = env->rows;
    const int cols = env->cols;
    const int map_size = static_cast<int>(env->map.size());

    // Use iterative update mode: each selected top-clearance cell is blocked before
    // computing the next one, producing a spread-out clearance ranking.
    build_sort_clearance_with_update(env, rows, cols);
    parking_clearance_ready = true;

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

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    DefaultPlanner::init_heuristics(env);
    mt.seed(0);

    parking_clearance_ready = false;
    map_clearance.clear();
    sorted_cell_ids_by_clearance.clear();
    sorted_cell_clearance_level_indices.clear();
    sorted_cell_clearance_levels.clear();

    // Random/stay/traffic-only parking needs no geometric clearance table.
    // Combined APIs decide from their weights when a parking call is made.
    if (env->parking_model == 3 || env->parking_model == 4 || env->parking_model == 5)
        ensure_parking_clearance(env);
}

// Original task-flow movement cost. Directional flow values are treated as in
// the existing code: temp_op and temp_vertex are *integer* intermediates.
long long parking_movement_cost(int loc, int next_loc,
    SharedEnvironment* env, const std::vector<Double4>& background_flow,
    bool use_traffic)
{
    if (!use_traffic)
        return 1;
    const int d = get_d(loc - next_loc, env);
    const int op_flow = static_cast<int>(
        (background_flow[loc].d[d] + 1) *
        background_flow[next_loc].d[(d + 2) % 4]);
    int vertex_flow = 1;
    for (int j = 0; j < 4; ++j)
        vertex_flow += background_flow[next_loc].d[j];
    const long long movement = 1LL + op_flow + (vertex_flow - 1) / 2;
    if (movement < 1)
        throw std::runtime_error("Non-positive traffic movement cost");
    return movement;
}

struct FlowParkingResult {
    std::vector<int> flexible_agents;
    std::vector<int> assigned_tasks;   // -1 => parking, otherwise task id
    std::vector<int> parking_goals;    // -1 => assigned a task, otherwise cell id
    std::vector<std::list<int>> paths;
    long long task_movement_scaled = 0;
    long long task_movement_steps = 0;
    int candidate_count = 0;
    double graph_ms = 0.0;
    double solve_ms = 0.0;
    double extract_ms = 0.0;
};


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
    ensure_parking_clearance(env);

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

void schedule_plan_flow_dummy_all(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only, double B)
{

    cout<<"run schedule_plan_flow_dummy_all"<<endl;

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const int map_size = static_cast<int>(env->map.size());

    constexpr double gamma = 2.0;
    constexpr long long COST_SCALE = kParkingCostScale;
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
    ensure_parking_clearance(env);

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
        const ListDigraph::Arc arc = g.addArc(map_nodes[loc], sink);
        capacity[arc] = 1;
        cost[arc] = scale_parking_arc_cost(
            penalty, static_cast<double>(std::numeric_limits<long long>::max()) / 8.0);
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

    constexpr long long COST_SCALE = kParkingCostScale;
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
        const ListDigraph::Arc arc = g.addArc(map_nodes[loc], sink);
        capacity[arc] = 1;
        cost[arc] = scale_parking_arc_cost(
            penalty, static_cast<double>(std::numeric_limits<long long>::max()) / 8.0);
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

FlowParkingResult solve_parking_flow(
    SharedEnvironment* env,
    const std::vector<Double4>& background_flow,
    bool use_traffic,
    const ParkingWeights& w,
    const std::vector<int>& flexible_agents,
    const std::map<int, std::deque<int>>& task_ids_by_loc,
    const std::unordered_set<int>& protected_locations,
    bool combined)
{
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const int map_size = static_cast<int>(env->map.size());
    const int N = static_cast<int>(flexible_agents.size());
    int tasks_count = 0;
    for (const auto& entry : task_ids_by_loc)
        tasks_count += static_cast<int>(entry.second.size());
    const int task_quota = combined ? std::min(N, tasks_count) : 0;
    const int parking_quota = N - task_quota;

    FlowParkingResult result;
    result.flexible_agents = flexible_agents;
    result.assigned_tasks.assign(N, -1);
    result.parking_goals.assign(N, -1);
    result.paths.resize(N);
    if (!N)
        return result;
    // Each unit of flow can traverse at most |V|-1 movement arcs on an
    // optimal positive-cost path, but incurs at most ONE terminal penalty.
    // Hence sink penalties need not be restricted by an extra factor |V|.
    const double safe_terminal_scaled_cost =
        static_cast<double>(std::numeric_limits<long long>::max()) /
        (32.0 * std::max(1, N));
    const double safe_movement_scaled_cost =
        safe_terminal_scaled_cost / std::max(1, map_size);

    double avg_step_cost = use_traffic ? avg_task_step_cost_for_parking : 1.0;
    if (!std::isfinite(avg_step_cost) || avg_step_cost < 1.0)
        avg_step_cost = 1.0;

    // No clearance computation at all when b_c==0 (or when nobody parks).
    const std::vector<std::tuple<int,int>>* clearance = nullptr;
    if (parking_quota && w.B > 0.0 && w.b_c > 0.0)
    {
        ensure_parking_clearance(env);
        clearance = &map_clearance;
    }

    // No destination-traffic array when b_t==0. Traffic-dependent *movement*
    // costs are intentionally separate, controlled by use_traffic.
    std::vector<double> vertex_traffic;
    if (parking_quota && w.B > 0.0 && w.b_t > 0.0) {
        vertex_traffic.assign(map_size, 0.0);
        for (int loc = 0; loc < map_size; ++loc) {
            if (env->map[loc] == 1)
                continue;
            for (int d = 0; d < 4; ++d)
                vertex_traffic[loc] += background_flow[loc].d[d];
        }
    }

    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::NodeMap<int> node_loc(g, -1);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<long long> cost(g);
    const auto source = g.addNode();
    const auto task_sink = g.addNode();
    const auto parking_sink = g.addNode();
    supply[source] = N;
    supply[task_sink] = -task_quota;
    supply[parking_sink] = -parking_quota;

    std::vector<ListDigraph::Node> nodes(map_size);
    for (int loc = 0; loc < map_size; ++loc) {
        nodes[loc] = g.addNode();
        node_loc[nodes[loc]] = loc;
        supply[nodes[loc]] = 0;
    }
    std::vector<ListDigraph::Arc> source_arcs;
    source_arcs.reserve(N);
    for (int a : flexible_agents) {
        int loc = env->curr_states[a].location;
        if (loc < 0 || loc >= map_size || env->map[loc] == 1)
            throw std::runtime_error("Invalid flexible agent position");
        const auto arc = g.addArc(source, nodes[loc]);
        capacity[arc] = 1;
        cost[arc] = 0;
        source_arcs.push_back(arc);
    }

    // Every task is a distinct unit of required demand. Same-pickup tasks
    // share a task-to-sink arc of corresponding capacity.
    if (task_quota) {
        for (const auto& entry : task_ids_by_loc) {
            const int loc = entry.first;
            if (loc < 0 || loc >= map_size || env->map[loc] == 1)
                throw std::runtime_error("Invalid pickup cell");
            const auto arc = g.addArc(nodes[loc], task_sink);
            capacity[arc] = static_cast<int>(entry.second.size());
            cost[arc] = 0;
        }
    }

    // All traversable cells except protected task targets are eligible.
    // This includes the current cell of each idle agent if it is not protected,
    // enabling Stay as a genuine choice without using a dummy zero-cost option.
    if (parking_quota) {
        for (int loc = 0; loc < map_size; ++loc) {
            if (env->map[loc] == 1 || protected_locations.count(loc))
                continue;
            double score = 0.0;
            if (clearance) {
                const int c_int = std::get<0>((*clearance)[loc]);
                const int o_int = std::get<1>((*clearance)[loc]);
                if (c_int < 1 || o_int < 0)
                    throw std::runtime_error("Invalid static clearance");
                const double c = static_cast<double>(c_int);
                const double o = static_cast<double>(o_int);
                const double base = 1.0 / (c*c);
                const double next = 1.0 / ((c+1.0)*(c+1.0));
                score += w.b_c * (base + (o / (8.0*c))*(base - next));
            }
            if (!vertex_traffic.empty()) {
                const double T = vertex_traffic[loc];
                score += w.b_t * (T*T);
            }
            const auto arc = g.addArc(nodes[loc], parking_sink);
            capacity[arc] = 1;
            cost[arc] = scale_parking_arc_cost(w.B * avg_step_cost * score,
                                               safe_terminal_scaled_cost);
            ++result.candidate_count;
        }
        if (result.candidate_count < parking_quota)
            throw std::runtime_error("Not enough eligible parking cells");
    }

    // Directed movement graph: costs for traversing to a cell are the same
    // as in schedule_plan_flow(), including the no-horizontal-wrap fix.
    const int offsets[4] = {-env->cols, 1, env->cols, -1};
    for (int loc = 0; loc < map_size; ++loc) {
        if (env->map[loc] == 1)
            continue;
        for (int dir = 0; dir < 4; ++dir) {
            if (dir == 1 && loc % env->cols == env->cols - 1)
                continue;
            if (dir == 3 && loc % env->cols == 0)
                continue;
            const int next = loc + offsets[dir];
            if (next < 0 || next >= map_size || env->map[next] == 1)
                continue;
            const long long w_edge = parking_movement_cost(
                loc, next, env, background_flow, use_traffic);
            const auto arc = g.addArc(nodes[loc], nodes[next]);
            capacity[arc] = N;
            const double scaled_edge = static_cast<double>(kParkingCostScale) * w_edge;
            if (!std::isfinite(scaled_edge) || scaled_edge >= safe_movement_scaled_cost)
                throw std::runtime_error("Movement arc cost exceeds safe int64 range");
            cost[arc] = kParkingCostScale * w_edge;
        }
    }
    const auto t_graph_end = Clock::now();

    using ParkingNS = NetworkSimplex<ListDigraph, int, long long>;
    ParkingNS ns(g);
    ns.costMap(cost).upperMap(capacity).supplyMap(supply);
    const auto status = ns.run();
    const auto t_solve_end = Clock::now();
    if (status != ParkingNS::OPTIMAL)
        throw std::runtime_error("Task/parking flow infeasible or unbounded");

    ListDigraph::ArcMap<int> remaining(g);
    ns.flowMap(remaining);
    std::map<int, std::deque<int>> remaining_tasks = task_ids_by_loc;
    std::unordered_set<int> used_parking_cells;
    int assigned_count = 0;
    int parked_count = 0;
    for (int i = 0; i < N; ++i) {
        if (remaining[source_arcs[i]] != 1)
            throw std::runtime_error("Source arc did not select agent");
        --remaining[source_arcs[i]];
        const int start = env->curr_states[flexible_agents[i]].location;
        auto cur = nodes[start];
        int traversed = 0;
        int moved = 0;
        long long path_cost = 0;
        bool done = false;
        while (!done) {
            const int loc = node_loc[cur];
            if (loc < 0 || loc >= map_size)
                throw std::runtime_error("Unexpected flow node during decomposition");
            result.paths[i].push_back(loc);
            bool found = false;
            for (ListDigraph::OutArcIt arc(g, cur); arc != lemon::INVALID; ++arc) {
                if (remaining[arc] <= 0)
                    continue;
                const auto nxt = g.target(arc);
                --remaining[arc];
                if (nxt == task_sink) {
                    auto it = remaining_tasks.find(loc);
                    if (it == remaining_tasks.end() || it->second.empty())
                        throw std::runtime_error("No task at selected task sink location");
                    result.assigned_tasks[i] = it->second.front();
                    it->second.pop_front();
                    result.task_movement_scaled += path_cost;
                    result.task_movement_steps += moved;
                    ++assigned_count;
                    done = true;
                } else if (nxt == parking_sink) {
                    if (!used_parking_cells.insert(loc).second)
                        throw std::runtime_error("Duplicate parking destination");
                    if (protected_locations.count(loc))
                        throw std::runtime_error("Parking on protected task cell");
                    result.parking_goals[i] = loc;
                    ++parked_count;
                    done = true;
                } else {
                    path_cost += cost[arc];
                    ++moved;
                    cur = nxt;
                }
                found = true;
                break;
            }
            if (!found)
                throw std::runtime_error("Incomplete task/parking flow path");
            if (++traversed > map_size + 1)
                throw std::runtime_error("Unexpected flow cycle during extraction");
        }
    }
    if (assigned_count != task_quota || parked_count != parking_quota)
        throw std::runtime_error("Wrong task/parking quotas after extraction");
    for (ListDigraph::InArcIt arc(g, task_sink); arc != lemon::INVALID; ++arc)
        if (remaining[arc] != 0)
            throw std::runtime_error("Unconsumed task-sink flow");
    for (ListDigraph::InArcIt arc(g, parking_sink); arc != lemon::INVALID; ++arc)
        if (remaining[arc] != 0)
            throw std::runtime_error("Unconsumed parking-sink flow");
    const auto t_extract_end = Clock::now();

    auto elapsed_ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b-a).count();
    };
    result.graph_ms = elapsed_ms(t0,t_graph_end);
    result.solve_ms = elapsed_ms(t_graph_end,t_solve_end);
    result.extract_ms = elapsed_ms(t_solve_end,t_extract_end);
    return result;
}

void log_parking_flow_result(const char* label, const FlowParkingResult& r,
    const ParkingWeights& w)
{
    const int tasks = static_cast<int>(std::count_if(
        r.assigned_tasks.begin(), r.assigned_tasks.end(), [](int t){return t!=-1;}));
    const int idle = static_cast<int>(r.flexible_agents.size()) - tasks;
    int staying = 0;
    for (size_t i = 0; i < r.flexible_agents.size(); ++i)
        if (r.parking_goals[i] >= 0 && r.paths[i].size()==1)
            ++staying;
    std::cout << "[" << label << "] agents=" << r.flexible_agents.size()
              << " tasks=" << tasks << " parking=" << idle
              << " candidates=" << r.candidate_count
              << " stay=" << staying
              << " B=" << w.B << " b_c=" << w.b_c << " b_t=" << w.b_t
              << " graph_ms=" << r.graph_ms
              << " solver_ms=" << r.solve_ms
              << " extraction_ms=" << r.extract_ms << std::endl;
}

// Call once from schedule_initialize() when starting a new simulation to
// invalidate clearance data and the previous task-step cost reference.
void reset_combined_parking_model()
{
    avg_task_step_cost_for_parking = 1.0;
}

// -------------------------------------------------------------------------
// Function 1. SECOND parking flow: call AFTER schedule_plan_flow() has updated
// env->curr_task_schedule. Task-assignment decisions remain completely fixed.
// -------------------------------------------------------------------------
// b_c == 0 skips all static clearance work for the parking cost.
// b_t == 0 skips the destination vertex-traffic score (but movement edges
// still use traffic if use_traffic==true).
void schedule_plan_flow_dummy_combined(
    int time_limit, SharedEnvironment* env,
    const std::vector<Double4>& background_flow,
    bool use_traffic, bool new_only,
    double B, double b_c, double b_t)
{
    const ParkingWeights weights{B,b_c,b_t};
    validate_parking_weights(weights);
    if (static_cast<int>(env->curr_task_schedule.size()) != env->num_of_agents)
        throw std::runtime_error("curr_task_schedule size mismatch");

    std::vector<int> idle;
    std::unordered_set<int> protected_locs;
    for (int a = 0; a < env->num_of_agents; ++a) {
        const int task_id = env->curr_task_schedule[a];
        if (task_id == -1) {
            idle.push_back(a);
            continue;
        }
        const auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end())
            throw std::runtime_error("Working agent's task not found");
        const auto& t = it->second;
        if (t.idx_next_loc >= 0 && t.idx_next_loc < static_cast<int>(t.locations.size()))
            protected_locs.insert(t.locations[t.idx_next_loc]);
    }
    if (idle.empty()) {
        env->dummy_parking_allocation.clear();
        return;
    }
    const auto r = solve_parking_flow(env,background_flow,use_traffic,weights,
                                      idle,{},protected_locs,false);
    // Commit only after the entire flow has been successfully extracted.
    env->dummy_parking_allocation.clear();
    for (size_t i = 0; i < r.flexible_agents.size(); ++i) {
        const int a = r.flexible_agents[i];
        env->dummy_parking_allocation[a] = {r.parking_goals[i]};
        agent_guide_path[a] = r.paths[i];
    }
    log_parking_flow_result("parking-second-flow",r,weights);
}

// -------------------------------------------------------------------------
// Function 2. ONE combined task + parking flow: call INSTEAD of both
// schedule_plan_flow() and schedule_plan_flow_dummy_*().
// Guarantees the number of assigned tasks min(flexible_agents, flexible_tasks),
// but the joint objective may trade away some task-assignment distance to
// improve parking. See integration notes for this important distinction.
// -------------------------------------------------------------------------
void schedule_plan_flow_joint_task_parking(
    int time_limit, std::vector<int>& proposed_schedule,
    SharedEnvironment* env, const std::vector<Double4>& background_flow,
    bool use_traffic, bool new_only,
    double B, double b_c, double b_t)
{
    (void)time_limit; // No interruptible LEMON timeout in this implementation.
    const ParkingWeights weights{B,b_c,b_t};
    validate_parking_weights(weights);
    const int n = env->num_of_agents;
    if (static_cast<int>(env->curr_task_schedule.size()) != n)
        throw std::runtime_error("curr_task_schedule size mismatch");

    // Preserve assignments for tasks already opened. new_only is reserved
    // for future use; unopened assignments can currently be reconsidered.
    std::vector<int> next_schedule = env->curr_task_schedule;
    std::vector<int> flexible;
    std::vector<unsigned char> is_flexible(n,0);
    auto add_flexible = [&](int a) {
        if (a < 0 || a >= n)
            throw std::runtime_error("Invalid task-assigned agent id");
        if (!is_flexible[a]) {
            flexible.push_back(a);
            is_flexible[a] = 1;
        }
    };
    for (int a=0; a<n; ++a)
        if (env->curr_task_schedule[a] == -1)
            add_flexible(a);

    std::map<int,std::deque<int>> task_ids_by_loc;
    std::unordered_set<int> protected_locs;
    std::vector<unsigned char> fixed_agent(n,0);
    for (const auto& entry : env->task_pool) {
        const int task_id = entry.first;
        const auto& task = entry.second;
        if (task.locations.empty())
            throw std::runtime_error("Task without pickup/delivery locations");
        const int assigned_agent = task.agent_assigned;
        if (task.idx_next_loc > 0) {
            if (assigned_agent < 0 || assigned_agent >= n)
                throw std::runtime_error("Committed task missing assigned agent");
            next_schedule[assigned_agent] = task_id;
            fixed_agent[assigned_agent] = 1;
            if (task.idx_next_loc >= 0 &&
                task.idx_next_loc < static_cast<int>(task.locations.size()))
                protected_locs.insert(task.locations[task.idx_next_loc]);
            continue;
        }
        const int pickup = task.locations[0];
        task_ids_by_loc[pickup].push_back(task_id);
        protected_locs.insert(pickup); // Every considered task must be served if N>=M.
        if (assigned_agent != -1)
            add_flexible(assigned_agent);
    }
    // A locked agent cannot also enter the flexible flow (even if the
    // environment gave inconsistent curr_task_schedule information).
    flexible.erase(std::remove_if(flexible.begin(), flexible.end(),
        [&](int a){ return fixed_agent[a] != 0; }), flexible.end());
    // Stable order within each task pickup group.
    for (auto& entry:task_ids_by_loc)
        std::sort(entry.second.begin(),entry.second.end());
    std::sort(flexible.begin(),flexible.end());
    for (int a : flexible)
        next_schedule[a] = -1;

    const auto r = solve_parking_flow(env,background_flow,use_traffic,weights,
                                      flexible,task_ids_by_loc,protected_locs,true);
    std::vector<std::pair<int,int>> parking_allocs;
    parking_allocs.reserve(flexible.size());
    for (size_t i = 0; i < r.flexible_agents.size(); ++i) {
        const int a = r.flexible_agents[i];
        if (r.assigned_tasks[i] >= 0)
            next_schedule[a] = r.assigned_tasks[i];
        else
            parking_allocs.emplace_back(a,r.parking_goals[i]);
    }

    // Atomic schedule and goal commit. Do not modify the simulator on failure.
    proposed_schedule = next_schedule;
    env->curr_task_schedule = next_schedule;
    env->dummy_parking_allocation.clear();
    agent_guide_path.clear();
    for (size_t i = 0; i < r.flexible_agents.size(); ++i) {
        if (r.assigned_tasks[i] >= 0) {
            // Match the old task-flow guidance convention.
            if (use_traffic)
                agent_guide_path[r.flexible_agents[i]] = r.paths[i];
        } else {
            const int a = r.flexible_agents[i];
            env->dummy_parking_allocation[a] = {r.parking_goals[i]};
            agent_guide_path[a] = r.paths[i];
        }
    }

    // For the *next* planning cycle, the combined solver can use the mean
    // movement cost of the task paths chosen in this planning cycle.
    if (use_traffic && r.task_movement_steps > 0)
        avg_task_step_cost_for_parking =
            static_cast<double>(r.task_movement_scaled) /
            (static_cast<double>(kParkingCostScale) * r.task_movement_steps);
    if (!use_traffic)
        avg_task_step_cost_for_parking = 1.0;
    log_parking_flow_result("task-and-parking-one-flow",r,weights);
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
