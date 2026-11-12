#include "CompetitionSystem.h"
#include <climits>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>

// Exercise the internal conversion boundary without exposing it in the public
// scheduler API or constructing a huge flow graph.
namespace DefaultPlanner {
long long scale_parking_arc_cost(double unscaled, double safe_scaled_cost);
}

namespace {
void check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

struct ScriptedEntry : Entry
{
    std::function<void(ScriptedEntry&, vector<vector<Action>>&, vector<int>&)> script;
    vector<int> timesteps;
    vector<int> budgets;
    vector<vector<int>> new_tasks_seen;
    vector<vector<int>> free_agents_seen;
    std::thread::id caller = std::this_thread::get_id();

    void initialize(int) override {}
    void compute(int budget, vector<vector<Action>>& actions, vector<int>& schedule) override
    {
        check(std::this_thread::get_id() == caller, "planning must run on the simulation thread");
        timesteps.push_back(env->curr_timestep);
        budgets.push_back(budget);
        new_tasks_seen.push_back(env->new_tasks);
        free_agents_seen.push_back(env->new_freeagents);
        script(*this, actions, schedule);
    }
};

struct InspectableSystem : BaseSystem
{
    using BaseSystem::BaseSystem;
    int timestep() { return simulator.get_curr_timestep(); }
    int timeouts() { return total_timetous; }
    int finished() { return task_manager.num_of_task_finish; }
    int schedule_errors() { return task_manager.get_number_errors(); }
    int action_errors() { return simulator.get_number_errors(); }
    const vector<int>& costs() { return solution_costs; }
    nlohmann::ordered_json paths() { return simulator.actual_path_to_json(); }
    nlohmann::ordered_json planned_paths() { return simulator.planned_path_to_json(); }
    size_t timing_count() { return planner_times.size(); }
};

struct Fixture
{
    std::filesystem::path directory;
    Grid grid;
    vector<int> starts;
    vector<list<int>> tasks;
    Logger logger;
    ActionModel model;
    ScriptedEntry* entry;
    InspectableSystem system;

    static std::filesystem::path make_directory()
    {
        auto directory = std::filesystem::temp_directory_path() /
            ("fleet-commit-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directory(directory);
        std::ofstream(directory / "map") << "type octile\nheight 3\nwidth 6\nmap\n......\n......\n......\n";
        return directory;
    }

    Fixture(int window, vector<int> starts = {0}, vector<list<int>> tasks = {{17}}):
        directory(make_directory()), grid((directory / "map").string()),
        starts(std::move(starts)), tasks(std::move(tasks)), logger("", 5), model(grid),
        entry(new ScriptedEntry()), system(grid, entry, this->starts, this->tasks, &model)
    {
        entry->commit_window = window;
        model.set_logger(&logger);
        system.set_logger(&logger);
        system.set_plan_time_limit(1000);
        system.set_preprocess_time_limit(1000);
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
};

void test_batches_and_horizon()
{
    Fixture f(3);
    f.entry->script = [](auto&, auto& actions, auto& schedule)
    {
        actions = {{Action::E}, {Action::E}, {Action::WE}};
        schedule = {0};
    };
    f.system.simulate(7);
    check(f.entry->timesteps == vector<int>({0, 3, 6}), "planner called inside a commit window");
    check(f.entry->budgets == vector<int>({3000, 3000, 3000}), "window budget not scaled");
    check(f.system.paths()[0] == "R,R,L,R,R,L,R", "commits executed out of order");
    check(f.system.planned_paths() == f.system.paths(), "planned paths not recorded per step");
    check(f.system.timestep() == 7 && f.system.timeouts() == 0, "horizon overshot");
    check(f.system.timing_count() == 3, "timings must be per planner call");
}

void test_single_step()
{
    Fixture f(1);
    f.entry->script = [](auto&, auto& actions, auto&) { actions = {{Action::WA}}; };
    f.system.simulate(3);
    check(f.entry->timesteps == vector<int>({0, 1, 2}), "single-step behavior changed");
    check(f.entry->budgets == vector<int>({1000, 1000, 1000}), "single-step budget changed");
}

void test_short_empty_and_long_batches()
{
    Fixture f(3);
    f.entry->script = [](auto& self, auto& actions, auto&)
    {
        if (self.timesteps.size() == 1)
            actions = {{Action::E}};
        else if (self.timesteps.size() == 3)
            actions = {{Action::E}, {Action::E}, {Action::E}, {Action::E}};
        // The second call leaves an empty batch; previous actions must not persist.
    };
    f.system.simulate(9);
    check(f.system.paths()[0] == "R,W,W,W,W,W,R,R,R", "batch padding or truncation failed");
    check(f.system.timeouts() == 0, "missing actions are not timeout penalties");
}

void test_mid_window_tasks()
{
    Fixture f(3, {0}, {{1, 2}, {3}});
    f.entry->script = [](auto& self, auto& actions, auto& schedule)
    {
        if (self.timesteps.size() == 1)
        {
            schedule = {0};
            actions = {{Action::E}, {Action::E}, {Action::E}};
        }
        else
        {
            check(self.env->curr_task_schedule == vector<int>({-1}), "finished assignment persisted");
            check(self.env->task_pool.count(0) == 0, "completed task remains in pool");
            check(self.env->task_pool.at(1).t_revealed == 2, "replacement task revealed too late");
            schedule = {1};
            actions = {{Action::WA}};
        }
    };
    f.system.simulate(4);
    check(f.system.finished() == 2, "tasks not progressed at each step");
    check(f.system.schedule_errors() == 0, "completed assignment reapplied");
    check(f.entry->new_tasks_seen[1] == vector<int>({1}), "new task notification lost");
    check(f.entry->free_agents_seen[1] == vector<int>({0}), "free agent notification lost");
    check(f.system.costs() == vector<int>({3}), "cost includes steps after task completion");
}

void test_timeout_after_commits()
{
    Fixture f(3, {0}, {{0, 0, 0, 0}});
    f.system.set_plan_time_limit(100);
    f.entry->script = [](auto&, auto& actions, auto& schedule)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        actions = {{Action::WA}, {Action::WA}, {Action::WA}};
        schedule = {0};
    };
    f.system.simulate(4);
    check(f.entry->budgets == vector<int>({300}), "timeout call got wrong budget");
    check(f.system.paths()[0] == "W,W,W,T", "timeout wait must follow all commits");
    check(f.system.timeouts() == 1, "timeout count must count executed wait steps");
    check(f.system.finished() == 1, "timeout wait must advance task errands");
}

void test_timeout_horizon()
{
    Fixture f(2);
    f.system.set_plan_time_limit(10);
    f.entry->script = [](auto&, auto& actions, auto&)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(55));
        actions = {{Action::E}, {Action::WE}};
    };
    f.system.simulate(3);
    check(f.system.paths()[0] == "R,L,T", "timeout advanced beyond horizon");
    check(f.system.timeouts() == 1 && f.system.timestep() == 3, "timeout count exceeds simulated waits");
}

void test_replanning_after_multiple_timeout_waits()
{
    Fixture f(2);
    f.system.set_plan_time_limit(100);
    f.entry->script = [](auto& self, auto& actions, auto&)
    {
        if (self.timesteps.size() == 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(450));
        actions = {{Action::E}, {Action::WE}};
    };
    f.system.simulate(8);
    check(f.system.paths()[0] == "R,L,T,T,T,R,L,R", "overrun intervals not rounded up correctly");
    check(f.system.timeouts() == 3, "wrong number of timeout waits");
    check(f.entry->timesteps == vector<int>({0, 5, 7}), "planner did not resume after timeout waits");
}

void test_invalid_commits()
{
    Fixture f(3, {0, 2});
    f.entry->script = [](auto&, auto& actions, auto&)
    {
        actions = {{Action::E, Action::WE}, {Action::S, Action::S}, {Action::S, Action::S}};
    };
    f.system.simulate(3);
    check(f.system.action_errors() == 1, "conflicting joint action was not rejected");
    check(f.system.paths()[0] == "W,W,W" && f.system.paths()[1] == "W,W,W",
          "invalid plan suffix executed from an unexpected state");

    Fixture wrong_size(2);
    wrong_size.entry->script = [](auto&, auto& actions, auto&) { actions = {{}, {Action::E}}; };
    wrong_size.system.simulate(2);
    check(wrong_size.system.action_errors() == 1 && wrong_size.system.paths()[0] == "W,W",
          "malformed joint action not handled safely");
}

void test_waiting_history()
{
    Fixture f(3);
    f.entry->script = [](auto&, auto& actions, auto&) { actions = {{Action::WA}, {Action::WA}, {Action::E}}; };
    f.system.simulate(3);
    check(f.entry->env->accu_waitings[0] == 0, "wait streak not reset by movement");
    check(f.entry->env->past_waitings[1 * 5 + 2] == pair<double, double>(2, 1),
          "waiting history skipped intermediate committed steps");
}

// The default batch wrapper must reach planner.cpp without adapting the
// single-step virtual API.
struct DefaultBatchPlanner : MAPFPlanner
{
    using MAPFPlanner::MAPFPlanner;
    using MAPFPlanner::plan;
    void plan(int, vector<Action>&) override
    {
        throw std::runtime_error("default batch planner called the single-step wrapper");
    }
};

void test_default_pibt_commits()
{
    Fixture f(5, {0, 17}, {{1, 2, 2, 3}, {16, 15, 15, 14}});
    auto* env = f.entry->env;
    env->num_of_agents = 2;
    env->rows = f.grid.rows;
    env->cols = f.grid.cols;
    env->map = f.grid.map;
    env->curr_timestep = 7;
    env->curr_states = {State(0, 7, 0), State(17, 7, 0)};
    env->goal_locations = {{{1, 0}, {2, 0}, {2, 0}, {3, 0}},
                           {{16, 0}, {15, 0}, {15, 0}, {14, 0}}};
    env->curr_task_schedule = {0, 1};
    env->task_pool.emplace(0, Task(0, {1, 2, 2, 3}, 0));
    env->task_pool.emplace(1, Task(1, {16, 15, 15, 14}, 0));
    env->commit_window = 5;
    env->parking_model = 2;
    env->plan_start_time = std::chrono::steady_clock::now();
    auto* planner = new DefaultBatchPlanner(env);
    f.entry->planner = planner;
    planner->initialize(1000);

    const auto states = env->curr_states;
    const auto goals = env->goal_locations;
    auto check_environment = [&]()
    {
        check(env->curr_states == states && env->curr_timestep == 7,
              "PIBT rollout changed observed simulation states");
        check(env->goal_locations == goals && env->curr_task_schedule == vector<int>({0, 1}) &&
              env->task_pool.at(0).idx_next_loc == 0 && env->task_pool.at(1).idx_next_loc == 0,
              "PIBT rollout changed observed task progress");
    };
    const vector<vector<Action>> expected = {
        {Action::E, Action::WE}, {Action::E, Action::WE},
        {Action::WA, Action::WA}, {Action::E, Action::WE}, {Action::WA, Action::WA}
    };
    vector<vector<Action>> actions;
    // Exercise both guide refreshes and the fallback when the budget is gone.
    const unordered_map<int, list<int>> guides = {{0, {0, 1}}, {1, {17, 16}}};
    for (int budget : {100, 0})
    {
        DefaultPlanner::plan(budget, actions, env, guides);
        check(actions == expected, "PIBT did not advance errands or handle repeated goal locations");
        auto predicted = states;
        for (auto& joint : actions)
        {
            check(f.model.is_valid(predicted, joint, predicted[0].timestep), "default PIBT produced an invalid commit");
            predicted = f.model.result_states(predicted, joint);
        }
        check_environment();
    }

    env->plan_start_time = std::chrono::steady_clock::now();
    planner->plan(100, actions);
    check(actions == expected, "thin batch wrapper did not return the default planner's commits");
    check_environment();

    // The explicit single-step API produces one move even when W > 1.
    vector<Action> single;
    DefaultPlanner::plan(0, single, env, {});
    check(single == expected.front(), "explicit single-step planner returned the wrong move");
    check_environment();

    env->commit_window = 1;
    DefaultPlanner::plan(0, actions, env, {});
    check(actions.size() == 1 && actions.front() == single, "W=1 batch differs from one-step planning");

    // Competing agents must get fresh reservations for every predicted step.
    env->commit_window = 8;
    env->curr_states = {State(0, 7, 0), State(2, 7, 0)};
    env->goal_locations = {{{2, 0}}, {{0, 0}}};
    DefaultPlanner::plan(100, actions, env, {});
    check(actions.size() == 8, "PIBT did not return the requested number of commits");
    auto predicted = env->curr_states;
    vector<bool> reached(2, false);
    for (auto& joint : actions)
    {
        check(joint.size() == 2 && f.model.is_valid(predicted, joint, predicted[0].timestep),
              "PIBT batch contains a vertex or edge conflict");
        predicted = f.model.result_states(predicted, joint);
        for (int i = 0; i < 2; i++)
            reached[i] = reached[i] || predicted[i].location == env->goal_locations[i][0].first;
    }
    check(reached[0] && reached[1], "PIBT did not resolve competing agents within the batch");
    check(env->curr_states[0] == State(0, 7, 0) && env->curr_states[1] == State(2, 7, 0),
          "collision handling changed observed positions");
}

struct BatchPlanner : MAPFPlanner
{
    using MAPFPlanner::MAPFPlanner;
    int calls = 0;
    void plan(int budget, vector<vector<Action>>& actions) override
    {
        check(budget == 3000 && env->commit_window == 3, "native planner got wrong window budget");
        check(env->goal_locations[0].front().first == 1, "scheduler goals not available to native planner");
        calls++;
        actions = {{Action::E}, {Action::E}, {Action::WA}};
    }
    void plan(int, vector<Action>&) override
    {
        throw std::runtime_error("single-step API called for native batch planner");
    }
};

struct CountingScheduler : TaskScheduler
{
    using TaskScheduler::TaskScheduler;
    int calls = 0;
    void plan(int budget, vector<int>& schedule) override
    {
        check(budget == 3000, "scheduler did not receive full window budget");
        calls++;
        schedule = {0};
    }
};

void test_native_batch_dispatch()
{
    Fixture f(3, {0}, {{1, 2}});
    auto* planner = new BatchPlanner(f.entry->env);
    auto* scheduler = new CountingScheduler(f.entry->env);
    f.entry->planner = planner;
    f.entry->scheduler = scheduler;
    f.entry->script = [](auto& self, auto& actions, auto& schedule)
    {
        self.Entry::compute(self.budgets.back(), actions, schedule);
    };
    f.system.simulate(3);
    check(planner->calls == 1 && scheduler->calls == 1, "native planner or scheduler ran more than once per batch");
    check(f.system.finished() == 1 && f.system.paths()[0] == "R,R,W", "native batch not simulated");
}

void test_invalid_parameters()
{
    for (const auto config : vector<pair<int, int>>{{0, 100}, {-1, 100}, {1, 0}, {2, INT_MAX}})
    {
        Fixture f(config.first);
        f.system.set_plan_time_limit(config.second);
        bool threw = false;
        try { f.system.simulate(1); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid window/budget accepted");
    }
}

void test_clearance_preprocessing()
{
    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.num_of_agents = 2;
    env.curr_states = {State(0, 0, 0), State(5, 0, 0)};
    env.curr_task_schedule = {0, -1};
    env.task_pool.emplace(0, Task(0, {1}, 0));
    env.task_pool.at(0).agent_assigned = 0;
    const vector<DefaultPlanner::Double4> flow(18, DefaultPlanner::Double4{0, 0, 0, 0});

    // Legacy clearance modes must still be ready before planner initialization.
    vector<std::tuple<int, int>> expected_clearance;
    vector<int> expected_ranking;
    for (int model : {3, 4, 5})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance().size() == env.map.size(),
              "clearance mode was not preprocessed");
        check(DefaultPlanner::get_sorted_cell_ids_by_clearance().size() == env.map.size(),
              "clearance parking ranking is missing");
        if (model == 3)
        {
            expected_clearance = DefaultPlanner::get_map_clearance();
            expected_ranking = DefaultPlanner::get_sorted_cell_ids_by_clearance();
        }
    }

    for (int model : {1, 2, 6})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance().empty() &&
              DefaultPlanner::get_sorted_cell_ids_by_clearance().empty(),
              "non-clearance mode still builds a clearance table");
    }
    DefaultPlanner::schedule_plan_flow_dummy_traffic(100, &env, flow, false, false, 0.1);
    check(DefaultPlanner::get_map_clearance().empty(), "traffic-only parking computes clearance");
    check(env.dummy_parking_allocation.at(1) == vector<int>({5}), "zero-traffic parking should allow staying");

    DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 0, 0.01);
    DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 0, 1, 0.01);
    check(DefaultPlanner::get_map_clearance().empty(), "zero clearance weight still builds the table");
    check(env.dummy_parking_allocation.at(1) == vector<int>({5}), "zero destination penalty should allow staying");

    DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 1, 0.01);
    check(DefaultPlanner::get_map_clearance() == expected_clearance &&
          DefaultPlanner::get_sorted_cell_ids_by_clearance() == expected_ranking,
          "lazy clearance differs from upfront preprocessing");
    check(env.curr_task_schedule == vector<int>({0, -1}) && env.dummy_parking_allocation.at(1)[0] != 1,
          "lazy parking changed task assignments or parked on a protected goal");

    // A joint flow assigning every flexible agent to a task needs no parking scores.
    DefaultPlanner::schedule_initialize(1000, &env);
    env.curr_task_schedule = {-1, -1};
    env.task_pool.at(0).agent_assigned = -1;
    env.task_pool.emplace(1, Task(1, {4}, 0));
    vector<int> schedule;
    DefaultPlanner::schedule_plan_flow_joint_task_parking(100, schedule, &env, flow, false, false);
    check(schedule[0] >= 0 && schedule[1] >= 0 && DefaultPlanner::get_map_clearance().empty(),
          "joint task-only assignment unnecessarily computes clearance");

    // Removing a task introduces a parking demand, which activates lazy clearance.
    env.task_pool.erase(1);
    env.curr_task_schedule = {-1, -1};
    DefaultPlanner::schedule_plan_flow_joint_task_parking(100, schedule, &env, flow, false, false);
    check(DefaultPlanner::get_map_clearance() == expected_clearance &&
          env.dummy_parking_allocation.size() == 1,
          "joint parking did not initialize clearance on first use");
}
void test_parking_cost_validation()
{
    using DefaultPlanner::scale_parking_arc_cost;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double largest = std::numeric_limits<double>::max();
    check(scale_parking_arc_cost(0, 1000) == 0 &&
          scale_parking_arc_cost(1, 1000) == 100 &&
          scale_parking_arc_cost(1.234, 1000) == 123 &&
          scale_parking_arc_cost(1.236, 1000) == 124 &&
          scale_parking_arc_cost(9.99, 1000) == 999,
          "parking cost scaling does not use nearest-hundredth precision");
    for (auto values : vector<pair<double, double>>{
             {-1, 1000}, {inf, 1000}, {nan, 1000}, {largest, 1000},
             {10, 1000}, {9.996, 1000}, // Exact bound and rounding up to it.
             {1, 0}, {1, -1}, {1, inf}, {1, nan},
             {static_cast<double>(LLONG_MAX) / 100, largest}})
    {
        bool rejected = false;
        try { scale_parking_arc_cost(values.first, values.second); }
        catch (const std::invalid_argument&) { rejected = true; }
        catch (const std::overflow_error&) { rejected = true; }
        check(rejected, "invalid or oversized parking cost accepted");
    }

    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.num_of_agents = 1;
    env.curr_states = {State(0, 0, 0)};
    env.curr_task_schedule = {-1};
    env.dummy_parking_allocation = {{0, {7}}};
    const vector<DefaultPlanner::Double4> flow(18, DefaultPlanner::Double4{1, 0, 0, 0});
    const auto original_parking = env.dummy_parking_allocation;
    const auto original_guides = DefaultPlanner::get_guide_path();
    for (bool joint : {false, true})
    for (auto weights : vector<vector<double>>{
             {-1, 1, 0.01}, {nan, 1, 0.01}, {inf, 1, 0.01},
             {16, -1, 0.01}, {16, nan, 0.01}, {16, inf, 0.01},
             {16, 1, -1}, {16, 1, nan}, {16, 1, inf},
             {1e16, 0, 1}, // Finite and representable, but exceeds solver bound.
             {largest, 0, 1}}) // Overflow during scaling.
    {
        vector<int> schedule = {-42};
        bool rejected = false;
        try
        {
            if (joint)
                DefaultPlanner::schedule_plan_flow_joint_task_parking(
                    100, schedule, &env, flow, false, false, weights[0], weights[1], weights[2]);
            else
                DefaultPlanner::schedule_plan_flow_dummy_combined(
                    100, &env, flow, false, false, weights[0], weights[1], weights[2]);
        }
        catch (const std::invalid_argument&) { rejected = true; }
        catch (const std::overflow_error&) { rejected = true; }
        check(rejected, "parking API accepted invalid weights or excessive penalties");
        check(schedule == vector<int>({-42}) && env.curr_task_schedule == vector<int>({-1}) &&
              env.dummy_parking_allocation == original_parking &&
              DefaultPlanner::get_guide_path() == original_guides,
              "rejected parking costs partially changed assignments or guides");
    }
}

void test_parking_model_dispatch()
{
    for (int model : {7, 8})
    for (bool use_traffic : {false, true})
    {
        TaskScheduler scheduler;
        auto& env = *scheduler.env;
        env.rows = 3;
        env.cols = 6;
        env.map.assign(18, 0);
        env.num_of_agents = 3;
        env.parking_model = model;
        env.curr_states = {State(0, 0, 0), State(5, 0, 0), State(17, 0, 0)};
        env.curr_task_schedule = {0, -1, 1};
        env.task_pool.emplace(0, Task(0, {5, 4}, 0));
        env.task_pool.at(0).agent_assigned = 0;
        env.task_pool.emplace(1, Task(1, {17, 16}, 0));
        env.task_pool.at(1).agent_assigned = 2;
        env.task_pool.at(1).idx_next_loc = 1;
        scheduler.set_flow(vector<DefaultPlanner::Double4>(18, DefaultPlanner::Double4{0, 0, 0, 0}));
        scheduler.set_use_traffic(use_traffic);
        scheduler.initialize(1000);

        vector<int> schedule;
        scheduler.plan(1000, schedule);
        check(schedule == vector<int>({-1, 0, 1}) && env.curr_task_schedule == schedule,
              "new parking mode did not reassign an unopened task or preserve the opened task");
        check(env.dummy_parking_allocation.size() == 1 && env.dummy_parking_allocation.count(0),
              "new parking mode did not allocate parking to the idle agent");
        const int parking = env.dummy_parking_allocation.at(0).at(0);
        check(parking != 5 && parking != 16 &&
              DefaultPlanner::get_guide_path().at(0).back() == parking,
              "parking blocked a task target or its guide missed the assigned goal");

        // All agents working: remove obsolete parking allocations.
        env.curr_task_schedule = {-1, 0, 1};
        env.task_pool.at(0).agent_assigned = 1;
        env.task_pool.emplace(2, Task(2, {0, 1}, 0));
        scheduler.plan(1000, schedule);
        check(std::count(schedule.begin(), schedule.end(), -1) == 0 && schedule[2] == 1 &&
              env.dummy_parking_allocation.empty(),
              "new parking mode left an agent idle or retained stale parking with enough tasks");

        // No tasks: every agent must receive a distinct parking destination.
        env.task_pool.clear();
        env.curr_task_schedule.assign(3, -1);
        schedule.clear();
        scheduler.plan(1000, schedule);
        std::set<int> destinations;
        for (const auto& allocation : env.dummy_parking_allocation)
            destinations.insert(allocation.second.at(0));
        check(schedule == vector<int>({-1, -1, -1}) && destinations.size() == 3,
              "new parking mode failed when every agent was idle");
    }
}
} // namespace

int main()
{
    try
    {
        test_batches_and_horizon();
        test_single_step();
        test_short_empty_and_long_batches();
        test_mid_window_tasks();
        test_timeout_after_commits();
        test_timeout_horizon();
        test_replanning_after_multiple_timeout_waits();
        test_invalid_commits();
        test_waiting_history();
        test_default_pibt_commits();
        test_native_batch_dispatch();
        test_invalid_parameters();
        test_clearance_preprocessing();
        test_parking_cost_validation();
        test_parking_model_dispatch();
        std::cout << "All commit-window tests passed.\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
