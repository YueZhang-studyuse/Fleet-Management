#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>
#include <thread>
#include <future>
#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>
#include <tuple>

using namespace lemon;

namespace DefaultPlanner{

using ClearanceTable = std::vector<std::tuple<double,int>>;

// Precompute iterative and neighbourhood clearance for parking models 3, 4, 5, 7 and 8.
// Models 7 and 8 need only the table, so skip the sorted-cell ranking.
// Load/save a validated disk cache beside env->map_path when provided.
// Base clearance includes outside-map walls when env->boundary_aware_clearance
// is true (default); smoothing still uses only in-map traversable cells.
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
double calculate_parking_strength(int rows, int cols, double beta = 0.75);

// new_only is reserved for future use; it currently has no effect in these APIs.
// Optional output starts as a fresh delivery-only snapshot, then receives the
// solved pickup movement flow. The task flow itself always uses background_flow.
void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env,
    std::vector<Double4> background_flow, bool use_traffic, bool new_only,
    std::vector<Double4>* parking_background_flow = nullptr);
// node_to_maploc contains grid nodes only; terminals and non-grid arcs are ignored.
// parking_traffic must initially contain delivery-only traffic to avoid double-counting.
void add_pickup_flow_to_background_traffic(const ListDigraph& graph,
    const ListDigraph::ArcMap<int>& flow, const boost::unordered_map<int, int>& node_to_maploc,
    const SharedEnvironment* env, std::vector<Double4>& parking_traffic);
double calculate_mean_nonzero_traffic(const SharedEnvironment* env,
    const std::vector<Double4>& background_flow, std::vector<double>& vertex_traffic);
void schedule_plan_flow_dummy(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
// Reset parking-related preprocessing between simulations.
void reset_combined_parking_model();

// Parking scores: B * avg_step_cost * (b_c * F_C + b_t * T / mean_nonzero_T).
// An all-zero traffic field contributes zero. Disabled terms skip preprocessing.
// Models 5/6/7 share this solver: (b_c,b_t) = (1,0)/(0,0.5)/(1,0.5).
// Two-flow approach:
// Run task assignment first, then assign parking goals to idle agents.
void schedule_plan_flow_dummy_combined( int time_limit, SharedEnvironment* env, const std::vector<Double4>& background_flow, bool use_traffic, bool new_only, double B, double b_c = 1.0, double b_t = 0.5);

// One-flow approach:
// Jointly assign tasks and parking goals.
void schedule_plan_flow_joint_task_parking(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env, const std::vector<Double4>& background_flow, bool use_traffic, bool new_only, double B, double b_c = 1.0, double b_t = 0.5);

// Default calls use B computed by schedule_initialize. The overloads above
// retain an explicit B for controlled experiments, including B=0.
inline void schedule_plan_flow_dummy_combined(int time_limit, SharedEnvironment* env,
    const std::vector<Double4>& background_flow, bool use_traffic, bool new_only)
{
    schedule_plan_flow_dummy_combined(time_limit, env, background_flow,
                                     use_traffic, new_only, env->parking_strength);
}
inline void schedule_plan_flow_joint_task_parking(int time_limit,
    std::vector<int>& proposed_schedule, SharedEnvironment* env,
    const std::vector<Double4>& background_flow, bool use_traffic, bool new_only)
{
    schedule_plan_flow_joint_task_parking(time_limit, proposed_schedule, env,
        background_flow, use_traffic, new_only, env->parking_strength);
}

unordered_map<int,unordered_map<int,int>> compute_heuristics(SharedEnvironment* env, std::vector<Int4> background_flow, vector<pair<int,int>> current_id_assignment, unordered_map<int,list<int>> task_loc_ids, int max_num_tasks);
void printDIMACS(ListDigraph& g, ListDigraph::Node source, ListDigraph::Node sink, vector<ListDigraph::Node>& workers, vector<ListDigraph::Node>& tasks, ListDigraph::ArcMap<int>& capacity, ListDigraph::ArcMap<double>& cost);
bool isTaskNode(ListDigraph::Node node, ListDigraph& g, ListDigraph::Node sink);

unordered_map<int,list<int>> get_guide_path();
// Empty until clearance preprocessing has been requested for this simulation.
const ClearanceTable& get_map_clearance();
// Smooth a completed iterative table without modifying it. Chebyshev rings
// contain only in-map traversable cells; empty rings have no weight.
// Returns (fractional clearance, 0), or (0, 0) for obstacles. Defaults include
// radii 1..6; rings with weight below min_weight are omitted.
// Requires positive free-cell clearances, 0 <= decay < 1, 0 < min_weight <= 1.
// Uses O(R * map.size()) time and O(map.size()) auxiliary storage. Does not
// populate caches or change the clearance used by parking assignment.
ClearanceTable calculate_neighbourhood_clearance(
    const SharedEnvironment* env,
    const ClearanceTable& iterative_clearance,
    double decay = 0.5, double min_weight = 0.01);
// Replace the shared table with neighbourhood scores and refresh its ranking.
// Initialization uses kUseNeighbourhoodClearance to select the cached baseline.
void calculate_neighbourhood_clearance(
    const SharedEnvironment* env, double decay = 0.5, double min_weight = 0.01);
const std::vector<int>& get_sorted_cell_ids_by_clearance();
void refresh_sorted_cells_by_clearance(const SharedEnvironment* env, double min_clearance_threshold = 1.0);

}

#endif
