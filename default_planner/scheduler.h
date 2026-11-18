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

// Precompute iterative and neighbourhood clearance for parking models 3, 4 and 5.
// Combined parking APIs prepare it lazily if their active weights need it.
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

// new_only is reserved for future use; it currently has no effect in these APIs.
void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
void schedule_plan_flow_dummy(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
void schedule_plan_flow_dummy_all(int time_limit, SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only, double B);
// Traffic-only uses B * avg_step_cost * b_t * T / mean_nonzero_T.
void schedule_plan_flow_dummy_traffic(int time_limit, SharedEnvironment* env,
    std::vector<Double4> background_flow, bool use_traffic, bool new_only,
    double B = 16.0, double b_t = 0.1);
// Reset parking-related preprocessing between simulations.
void reset_combined_parking_model();

// Parking scores: B * avg_step_cost * (b_c * F_C + b_t * T / mean_nonzero_T).
// An all-zero traffic field contributes zero. Disabled terms skip preprocessing.
// Two-flow approach:
// Run task assignment first, then assign parking goals to idle agents.
void schedule_plan_flow_dummy_combined( int time_limit, SharedEnvironment* env, const std::vector<Double4>& background_flow, bool use_traffic, bool new_only, double B = 16.0, double b_c = 1.0, double b_t = 0.1);

// One-flow approach:
// Jointly assign tasks and parking goals.
void schedule_plan_flow_joint_task_parking(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env, const std::vector<Double4>& background_flow, bool use_traffic, bool new_only, double B = 16.0, double b_c = 1.0, double b_t = 0.1);

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
// Comment out this overload's call after the iterative build for the baseline.
void calculate_neighbourhood_clearance(
    const SharedEnvironment* env, double decay = 0.5, double min_weight = 0.01);
const std::vector<int>& get_sorted_cell_ids_by_clearance();
void refresh_sorted_cells_by_clearance(const SharedEnvironment* env, double min_clearance_threshold = 1.0);

}

#endif
