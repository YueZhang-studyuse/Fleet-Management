#include "CompetitionSystem.h"
#include <climits>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>
#include <zlib.h>

// Exercise the internal conversion boundary without exposing it in the public
// scheduler API or constructing a huge flow graph.
namespace DefaultPlanner {
long long scale_parking_arc_cost(double unscaled, double safe_scaled_cost);
double calculate_mean_nonzero_traffic(const SharedEnvironment* env,
    const std::vector<Double4>& background_flow, std::vector<double>& vertex_traffic);
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

nlohmann::ordered_json saved_results(Fixture& fixture, int detail = 1)
{
    const auto path = fixture.directory / "results.json";
    fixture.system.saveResults(path.string(), detail);
    std::ifstream input(path);
    nlohmann::ordered_json result;
    input >> result;
    return result;
}

void test_parking_history()
{
    Fixture f(3, {0, 5}, {{1}, {4}});
    f.system.set_num_tasks_reveal(2);
    f.entry->script = [](auto& self, auto& actions, auto& schedule)
    {
        if (self.env->curr_timestep == 0)
        {
            schedule = {0, -1};
            actions = {{Action::E, Action::WA}, {Action::WA, Action::WE},
                       {Action::WA, Action::WA}};
        }
        else
            schedule = {-1, 1}; // Empty batch exercises padded waits.
        // Planner-local state must not override the accepted assignments.
        self.env->curr_task_schedule = {-1, -1};
    };
    f.system.simulate(5);
    check(f.system.finished() == 2 && f.system.schedule_errors() == 0,
          "parking history scenario did not complete its tasks");
    const auto expected = nlohmann::ordered_json::parse(
        R"([{"t":0,"agents":[1]},{"t":1,"agents":[0,1]},{"t":3,"agents":[0]},{"t":4,"agents":[0,1]}])");
    for (int detail : {1, 2, 3})
    {
        const auto result = saved_results(f, detail);
        check(result.at("parking") == expected && result.at("simulationSteps") == 5,
              "parking JSON must track changes and duration at every detail level");
    }

    Fixture rejected(1);
    rejected.entry->script = [](auto& self, auto& actions, auto& schedule)
    {
        schedule = {self.env->curr_timestep == 0 ? 0 : 999};
        actions = {{Action::WA}};
        self.env->curr_task_schedule = {-1};
    };
    rejected.system.simulate(2);
    check(rejected.system.schedule_errors() == 1, "invalid test schedule was not rejected");
    check(saved_results(rejected).at("parking") ==
              nlohmann::ordered_json::parse(R"([{"t":0,"agents":[]}])"),
          "rejected schedule must retain the actual task assignment in parking history");

    Fixture empty(3);
    empty.system.simulate(0);
    check(saved_results(empty).at("parking") == nlohmann::ordered_json::array(),
          "zero simulated steps must produce an empty parking history");
    check(saved_results(empty).at("simulationSteps") == 0,
          "zero-step output must report its duration");
}

string read_gzip(const std::filesystem::path& path)
{
    gzFile input = gzopen(path.string().c_str(), "rb");
    check(input != nullptr, "gzip result file was not created");
    string decoded;
    char chunk[4096];
    int count;
    while ((count = gzread(input, chunk, sizeof(chunk))) > 0)
        decoded.append(chunk, count);
    const int status = gzclose(input);
    check(count == 0 && status == Z_OK, "gzip stream was corrupt or incomplete");
    return decoded;
}

void test_gzip_results()
{
    vector<int> starts(16);
    std::iota(starts.begin(), starts.end(), 0);
    Fixture f(2000, starts);
    f.entry->script = [](auto&, auto&, auto&) {}; // Padded waits, unchanged idle set.
    f.system.simulate(2000);
    for (int detail : {1, 2, 3})
    {
        const auto expected = saved_results(f, detail);
        check(expected.at("parking").size() == 1 && expected.at("simulationSteps") == 2000,
              "unchanged parking must be retained as one change point");
        const auto path = f.directory / "compressed.json";
        f.system.saveResults(path.string(), detail, true);
        const auto compressed_path = std::filesystem::path(path.string() + ".gz");
        std::ifstream encoded(compressed_path, std::ios::binary);
        check(encoded.get() == 0x1f && encoded.get() == 0x8b, "result lacks a gzip header");
        const string decoded = read_gzip(compressed_path);
        check(decoded == expected.dump(), "gzip output differs from compact plain JSON");
        check(!std::filesystem::exists(path), "gzip output created an intermediate plain file");
        if (detail == 1)
        {
            check(decoded.size() > 64 * 1024, "gzip test did not exercise buffer flushing");
            check(std::filesystem::file_size(compressed_path) < decoded.size(),
                  "repetitive simulation output was not compressed");
        }
        const auto explicit_path = f.directory / "explicit.json.gz";
        f.system.saveResults(explicit_path.string(), detail, true);
        check(read_gzip(explicit_path) == decoded &&
              !std::filesystem::exists(explicit_path.string() + ".gz"),
              "explicit gzip suffix was duplicated or changed output");
    }
    for (bool gzip : {false, true})
    {
        bool rejected = false;
        try { f.system.saveResults((f.directory / "missing" / "result.json").string(), 1, gzip); }
        catch (const std::exception&) { rejected = true; }
        check(rejected, "output open failure was silently ignored");
    }
}

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
    check(saved_results(f).at("parking") ==
              nlohmann::ordered_json::parse(R"([{"t":0,"agents":[0]}])") &&
          saved_results(f).at("simulationSteps") == 3,
          "timeout waits must extend parking duration without redundant changes");
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

    // Clearance modes must be ready before planner initialization.
    DefaultPlanner::ClearanceTable expected_clearance;
    vector<int> expected_ranking;
    for (int model : {3, 4, 5})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance().size() == env.map.size(),
              "clearance mode was not preprocessed");
        check(DefaultPlanner::get_sorted_cell_ids_by_clearance().size() == env.map.size(),
              "clearance parking ranking is missing");
        const auto& table = DefaultPlanner::get_map_clearance();
        bool fractional = false;
        for (const auto& cell : table)
        {
            check(std::get<1>(cell) == 0, "neighbourhood clearance retained obstacle tie-breakers");
            fractional = fractional || std::get<0>(cell) != std::floor(std::get<0>(cell));
        }
        check(fractional, "shared table discarded fractional neighbourhood scores");
        const auto& ranking = DefaultPlanner::get_sorted_cell_ids_by_clearance();
        for (size_t i = 1; i < ranking.size(); ++i)
            check(std::get<0>(table[ranking[i-1]]) >= std::get<0>(table[ranking[i]]),
                  "shared ranking was not refreshed after smoothing");
        if (model == 3)
        {
            expected_clearance = DefaultPlanner::get_map_clearance();
            expected_ranking = DefaultPlanner::get_sorted_cell_ids_by_clearance();
        }
    }

    for (int model : {7, 8})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance() == expected_clearance,
              "combined/joint initialization changed or omitted clearance values");
        check(DefaultPlanner::get_sorted_cell_ids_by_clearance().empty(),
              "combined/joint initialization unnecessarily built a sorted ranking");
        // Zero weights must not undo the table prepared during initialization.
        DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 0, 0, 0);
        DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 1, 0.1);
        check(DefaultPlanner::get_map_clearance() == expected_clearance &&
              DefaultPlanner::get_sorted_cell_ids_by_clearance().empty(),
              "online parking changed the preprocessed table or built an unused ranking");
    }

    for (int model : {1, 2, 6})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance().empty() &&
              DefaultPlanner::get_sorted_cell_ids_by_clearance().empty(),
              "non-clearance mode still builds a clearance table");
    }
    DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 0, 0.5);
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

    // Joint mode prepares clearance even before a call that assigns everyone tasks.
    env.parking_model = 8;
    DefaultPlanner::schedule_initialize(1000, &env);
    check(DefaultPlanner::get_map_clearance() == expected_clearance,
          "joint clearance was not built during initialization");
    env.curr_task_schedule = {-1, -1};
    env.task_pool.at(0).agent_assigned = -1;
    env.task_pool.emplace(1, Task(1, {4}, 0));
    vector<int> schedule;
    DefaultPlanner::schedule_plan_flow_joint_task_parking(100, schedule, &env, flow, false, false);
    check(schedule[0] >= 0 && schedule[1] >= 0 &&
          DefaultPlanner::get_map_clearance() == expected_clearance,
          "joint task-only assignment changed preprocessed clearance");

    // Removing a task introduces parking demand, which reuses the same table.
    env.task_pool.erase(1);
    env.curr_task_schedule = {-1, -1};
    DefaultPlanner::schedule_plan_flow_joint_task_parking(100, schedule, &env, flow, false, false);
    check(DefaultPlanner::get_map_clearance() == expected_clearance &&
          env.dummy_parking_allocation.size() == 1 &&
          DefaultPlanner::get_sorted_cell_ids_by_clearance().empty(),
          "joint parking did not reuse clearance without sorting");
}
void test_boundary_clearance()
{
    using namespace DefaultPlanner;
    for (auto shape : vector<pair<int, int>>{{1, 1}, {1, 5}, {5, 1}, {5, 5}, {3, 7}})
    for (int layout : {0, 1, 2})
    for (bool boundary_aware : {false, true})
    {
        SharedEnvironment env;
        env.rows = shape.first;
        env.cols = shape.second;
        env.map.assign(env.rows * env.cols, 0);
        env.num_of_agents = 0;
        env.parking_model = 7;
        env.boundary_aware_clearance = boundary_aware;
        if (layout == 1)
            env.map[env.map.size() / 2] = 1;
        if (layout == 2)
            for (int r = 0; r < env.rows; ++r)
                for (int c = 0; c < env.cols; ++c)
                    if (r == 0 || c == 0 || r == env.rows - 1 || c == env.cols - 1)
                        env.map[r * env.cols + c] = 1;

        // Independent oracle: enumerate explicit wall cells in a padded grid,
        // then exhaustively select the best cell after each virtual placement.
        auto blocked = env.map;
        vector<bool> selected(env.map.size(), false);
        ClearanceTable iterative(env.map.size());
        auto wall = [&](int r, int c) {
            if (r < 0 || c < 0 || r >= env.rows || c >= env.cols)
                return boundary_aware;
            return blocked[r * env.cols + c] == 1;
        };
        for (size_t step = 0; step < env.map.size(); ++step)
        {
            std::tuple<int, int, int> best{INT_MAX, INT_MAX, INT_MAX};
            for (int loc = 0; loc < static_cast<int>(env.map.size()); ++loc)
            {
                if (selected[loc]) continue;
                const int r = loc / env.cols, c = loc % env.cols;
                int distance = env.rows + env.cols;
                for (int wr = -1; wr <= env.rows; ++wr)
                    for (int wc = -1; wc <= env.cols; ++wc)
                        if (wall(wr, wc))
                            distance = std::min(distance, std::max(std::abs(r-wr), std::abs(c-wc)));
                int obstacles = 0;
                for (int wr = r-distance; wr <= r+distance; ++wr)
                    for (int wc = c-distance; wc <= c+distance; ++wc)
                        obstacles += wall(wr, wc);
                best = std::min(best, std::make_tuple(-distance, obstacles, loc));
            }
            const auto [negative_distance, obstacles, loc] = best;
            iterative[loc] = {-negative_distance, obstacles};
            selected[loc] = true;
            blocked[loc] = 1;
        }
        const auto expected = calculate_neighbourhood_clearance(&env, iterative);
        env.boundary_aware_clearance = !boundary_aware;
        check(calculate_neighbourhood_clearance(&env, iterative) == expected,
              "boundary setting changed smoothing for the same input table");
        env.boundary_aware_clearance = boundary_aware;
        schedule_initialize(1000, &env);
        check(get_map_clearance() == expected,
              "clearance preprocessing disagrees with explicit outside-wall oracle");
        if (boundary_aware && layout == 0 && (env.rows == 1 || env.cols == 1))
            for (const auto& cell : get_map_clearance())
                check(std::get<0>(cell) == 1.0, "open one-cell-wide maps must have clearance 1");
    }
    check(SharedEnvironment{}.boundary_aware_clearance, "boundary-aware clearance must default to enabled");
}

void test_clearance_disk_cache()
{
    Fixture fixture(1);
    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.num_of_agents = 0;
    env.map.assign(18, 0);
    env.map[7] = 1;
    env.parking_model = 7;
    env.map_path = (fixture.directory / "map").string();
    const std::filesystem::path path = env.map_path + ".clearance.json";
    auto read_cache = [&]() {
        std::ifstream input(path);
        return nlohmann::json::parse(input);
    };
    auto write_cache = [&](const nlohmann::json& value) {
        std::ofstream(path) << value;
    };
    DefaultPlanner::schedule_initialize(1000, &env);
    const auto expected = DefaultPlanner::get_map_clearance();
    check(std::filesystem::is_regular_file(path), "missing cache was not saved beside the map");
    const auto original = read_cache();
    check(original.at("metadata").at("boundary_aware") == true,
          "clearance cache omitted the enabled boundary setting");
    check(original.at("clearance") == nlohmann::json(expected),
          "saved cache changed clearance precision");
    const auto old_time = std::filesystem::file_time_type::clock::now() - std::chrono::hours(24);
    std::filesystem::last_write_time(path, old_time);
    const auto timestamp = std::filesystem::last_write_time(path);
    for (int model : {3, 4, 5, 7, 8})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance() == expected &&
              std::filesystem::last_write_time(path) == timestamp,
              "valid clearance cache was recomputed or changed values across parking models");
        check(DefaultPlanner::get_sorted_cell_ids_by_clearance().empty() == (model >= 7),
              "loaded clearance did not respect the model's ranking requirement");
    }

    env.boundary_aware_clearance = false;
    DefaultPlanner::schedule_initialize(1000, &env);
    const auto unaware = DefaultPlanner::get_map_clearance();
    check(unaware != expected && read_cache().at("metadata").at("boundary_aware") == false,
          "disabling boundary awareness reused the enabled cache");
    std::filesystem::last_write_time(path, old_time);
    const auto unaware_timestamp = std::filesystem::last_write_time(path);
    DefaultPlanner::schedule_initialize(1000, &env);
    check(DefaultPlanner::get_map_clearance() == unaware &&
          std::filesystem::last_write_time(path) == unaware_timestamp,
          "disabled boundary clearance did not reuse its compatible cache");
    env.boundary_aware_clearance = true;
    DefaultPlanner::schedule_initialize(1000, &env);
    check(DefaultPlanner::get_map_clearance() == expected && read_cache() == original,
          "enabling boundary awareness reused the disabled cache");

    // Each invalid cache must be replaced with a valid one, without publishing
    // a partially loaded table or silently accepting incompatible metadata.
    for (int invalid = 0; invalid < 11; ++invalid)
    {
        auto damaged = original;
        switch (invalid)
        {
        case 0: damaged["metadata"]["version"] = -1; break;
        case 1: damaged["metadata"]["rows"] = 99; break;
        case 2: damaged["metadata"]["decay"] = 0.9; break;
        case 3: damaged["metadata"]["neighbourhood"] = false; break;
        case 4: damaged["clearance"].erase(0); break;
        case 5: damaged["clearance"][0][0] = -1; break;
        case 6: damaged["clearance"][7][0] = 1; break;
        case 7: damaged["clearance"][0][1] = 0.5; break;
        case 8: damaged["clearance"][0][0] = nullptr; break;
        case 9: damaged["metadata"].erase("boundary_aware"); break;
        case 10: damaged["metadata"]["boundary_aware"] = false; break;
        }
        write_cache(damaged);
        DefaultPlanner::schedule_initialize(1000, &env);
        check(DefaultPlanner::get_map_clearance() == expected && read_cache() == original,
              "invalid clearance cache was not rebuilt correctly");
    }
    std::ofstream(path) << "{truncated";
    DefaultPlanner::schedule_initialize(1000, &env);
    check(read_cache() == original, "truncated cache was not rebuilt");

    // Same filename and dimensions, different obstacle layout.
    env.map[8] = 1;
    DefaultPlanner::schedule_initialize(1000, &env);
    const auto changed = DefaultPlanner::get_map_clearance();
    check(read_cache().at("metadata") != original.at("metadata") &&
          std::get<0>(changed[8]) == 0, "map change reused stale clearance");
    const auto source_path = env.map_path;
    env.map_path.clear();
    DefaultPlanner::schedule_initialize(1000, &env);
    check(DefaultPlanner::get_map_clearance() == changed,
          "cached and uncached clearance differ for the changed map");

    env.map_path = source_path;
    std::filesystem::remove(path);
    for (int model : {1, 2, 6})
    {
        env.parking_model = model;
        DefaultPlanner::schedule_initialize(1000, &env);
        check(!std::filesystem::exists(path) && DefaultPlanner::get_map_clearance().empty(),
              "non-clearance model created a clearance cache");
    }
    // A directory at the destination forces publication to fail, including
    // under privileged test runners where chmod does not prohibit writes.
    std::filesystem::create_directory(path);
    env.parking_model = 8;
    DefaultPlanner::schedule_initialize(1000, &env);
    check(DefaultPlanner::get_map_clearance() == changed,
          "cache write failure prevented use of computed clearance");
    for (const auto& file : std::filesystem::directory_iterator(fixture.directory))
        check(file.path().filename().string().find(".tmp.") == string::npos,
              "cache write failure left staging files behind");
}

void test_neighbourhood_clearance()
{
    using DefaultPlanner::calculate_neighbourhood_clearance;
    SharedEnvironment env;
    env.rows = 1;
    env.cols = 3;
    env.map = {0, 1, 0};
    DefaultPlanner::ClearanceTable original = {{2, 3}, {0, 0}, {10, 2}};
    auto values = calculate_neighbourhood_clearance(&env, original);
    check(std::abs(std::get<0>(values[0]) - 3.6) < 1e-12 &&
          std::abs(std::get<0>(values[2]) - 8.4) < 1e-12 &&
          values[1] == std::make_tuple(0.0, 0),
          "empty rings were counted or geometric rings failed to cross a wall");
    check(calculate_neighbourhood_clearance(&env, original, 0.5, 0.25) == values,
          "a ring at exactly the cutoff weight was omitted");
    check(std::get<0>(calculate_neighbourhood_clearance(&env, original, 0.5, 0.5)[0]) == 2.0,
          "empty rings changed the normalization");

    env.cols = 8;
    env.map.assign(8, 0);
    original.assign(8, {1, 0});
    original[7] = {100, 0};
    check(std::get<0>(calculate_neighbourhood_clearance(&env, original)[0]) == 1.0,
          "default neighbourhood includes radius 7");
    original[6] = {65, 0};
    check(std::abs(std::get<0>(calculate_neighbourhood_clearance(&env, original)[0]) -
                   191.0 / 127.0) < 1e-12,
          "default neighbourhood excludes radius 6 or loses fractional precision");

    // Independent exhaustive ring enumeration checks prefix sums, clipping,
    // per-ring averaging, and immutable input on rectangular and thin maps.
    std::mt19937 rng(17);
    for (int rows = 1; rows <= 7; ++rows)
    for (int cols = 1; cols <= 8; ++cols)
    for (double decay : {0.0, 0.5, 0.8})
    {
        env.rows = rows;
        env.cols = cols;
        env.map.resize(rows * cols);
        original.resize(rows * cols);
        for (int loc = 0; loc < rows * cols; ++loc)
        {
            env.map[loc] = rng() % 4 == 0 ? 1 : 0;
            original[loc] = {env.map[loc] == 1 ? 0.0 : 1.0 + (rng() % 80) / 4.0,
                             static_cast<int>(rng() % 8)};
        }
        const auto snapshot = original;
        values = calculate_neighbourhood_clearance(&env, original, decay);
        check(original == snapshot, "neighbourhood preprocessing modified the iterative table");
        for (int loc = 0; loc < rows * cols; ++loc)
        {
            double expected = 0;
            if (env.map[loc] != 1)
            {
                double numerator = std::get<0>(original[loc]);
                double denominator = 1;
                double weight = decay;
                for (int radius = 1; radius < std::max(rows, cols) && weight >= 0.01;
                     ++radius, weight *= decay)
                {
                    double sum = 0;
                    int count = 0;
                    for (int q = 0; q < rows * cols; ++q)
                        if (env.map[q] != 1 &&
                            std::max(std::abs(q / cols - loc / cols),
                                     std::abs(q % cols - loc % cols)) == radius)
                        {
                            sum += std::get<0>(original[q]);
                            ++count;
                        }
                    if (count)
                    {
                        numerator += weight * sum / count;
                        denominator += weight;
                    }
                }
                expected = numerator / denominator;
            }
            check(std::abs(std::get<0>(values[loc]) - expected) < 1e-12 &&
                  std::get<1>(values[loc]) == 0,
                  "prefix-sum neighbourhood differs from direct ring enumeration");
        }
    }
    env.rows = env.cols = 1;
    env.map = {0};
    original = {{4, 5}};
    check(calculate_neighbourhood_clearance(&env, original)[0] == std::make_tuple(4.0, 0),
          "isolated free cell changed clearance");
    env.map[0] = 1;
    check(calculate_neighbourhood_clearance(&env, original)[0] == std::make_tuple(0.0, 0),
          "obstacle-only map should have zero clearance scores");
    env.map[0] = 0;
    for (auto parameters : vector<pair<double,double>>{
             {-0.5, 0.01}, {1, 0.01}, {0.5, 0}, {0.5, 1.1},
             {std::numeric_limits<double>::quiet_NaN(), 0.01},
             {0.5, std::numeric_limits<double>::infinity()}})
    {
        bool rejected = false;
        try { calculate_neighbourhood_clearance(&env, original, parameters.first, parameters.second); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid neighbourhood weights accepted");
    }
    for (int invalid_case = 0; invalid_case < 4; ++invalid_case)
    {
        SharedEnvironment invalid_env;
        invalid_env.rows = 1;
        invalid_env.cols = 1;
        invalid_env.map = {0};
        auto invalid_table = original;
        if (invalid_case == 1) invalid_env.cols = 2;
        if (invalid_case == 2) invalid_table.clear();
        if (invalid_case == 3) invalid_table[0] = {0, 0};
        bool rejected = false;
        try { calculate_neighbourhood_clearance(invalid_case == 0 ? nullptr : &invalid_env, invalid_table); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid neighbourhood map or iterative table accepted");
    }
}

void test_shared_clearance_consumers()
{
    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.num_of_agents = 1;
    env.parking_model = 5;
    env.curr_states = {State(0, 0, 0)};
    env.curr_task_schedule = {-1};
    DefaultPlanner::schedule_initialize(1000, &env);

    const auto before = DefaultPlanner::get_map_clearance();
    const auto expected = DefaultPlanner::calculate_neighbourhood_clearance(&env, before);
    DefaultPlanner::calculate_neighbourhood_clearance(&env);
    check(DefaultPlanner::get_map_clearance() == expected,
          "shared-table overload did not replace the completed input scores");
    vector<int> ranking(18);
    std::iota(ranking.begin(), ranking.end(), 0);
    std::sort(ranking.begin(), ranking.end(), [&](int a, int b) {
        return std::get<0>(expected[a]) != std::get<0>(expected[b]) ?
            std::get<0>(expected[a]) > std::get<0>(expected[b]) : a < b;
    });
    check(DefaultPlanner::get_sorted_cell_ids_by_clearance() == ranking,
          "shared-table overload did not refresh fractional ranking and tie order");

    // Compare sequential and joint penalty flows with exhaustive single-agent costs.
    // This detects integer truncation between the shared table and arc costs.
    const vector<DefaultPlanner::Double4> flow(18, DefaultPlanner::Double4{0, 0, 0, 0});
    for (int start : {0, 5, 17})
    for (int model : {7, 8})
    {
        env.curr_states[0].location = start;
        vector<int> schedule;
        if (model == 7)
            DefaultPlanner::schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 64, 1, 0);
        else
            DefaultPlanner::schedule_plan_flow_joint_task_parking(100, schedule, &env, flow, false, false, 64, 1, 0);
        auto cost = [&](int loc) {
            const double c = std::get<0>(expected[loc]);
            const int distance = std::abs(start / 6 - loc / 6) + std::abs(start % 6 - loc % 6);
            return 100LL * distance + std::llround(6400.0 / (c*c));
        };
        long long best = LLONG_MAX;
        for (int loc = 0; loc < 18; ++loc)
            best = std::min(best, cost(loc));
        check(cost(env.dummy_parking_allocation.at(0).at(0)) == best,
              "parking flow did not optimize the fractional shared-clearance penalty");
        check(DefaultPlanner::get_map_clearance() == expected,
              "parking flow rebuilt or modified the shared clearance table");
    }
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

void test_normalized_traffic_parking()
{
    using namespace DefaultPlanner;
    SharedEnvironment env;
    env.rows = 1;
    env.cols = 5;
    env.map = {0, 1, 0, 0, 0};
    vector<Double4> flow(5, Double4{0, 0, 0, 0});
    flow[0] = Double4{1, 1, 0, 0};
    flow[1] = Double4{1000, 0, 0, 0};
    flow[3] = Double4{1, 2, 3, 0};
    vector<double> traffic;
    check(calculate_mean_nonzero_traffic(&env, flow, traffic) == 4.0 &&
          traffic == vector<double>({2, 0, 0, 6, 0}),
          "traffic reference includes obstacles/zeros or misses directional contributions");
    for (auto& cell : flow) for (double& value : cell.d) value *= 10;
    check(calculate_mean_nonzero_traffic(&env, flow, traffic) == 40.0 &&
          traffic[0]/40 == 0.5 && traffic[3]/40 == 1.5,
          "linear traffic normalization is not scale invariant");
    flow.assign(5, Double4{0, 0, 0, 0});
    check(calculate_mean_nonzero_traffic(&env, flow, traffic) == 0.0,
          "empty traffic must have zero reference");

    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.num_of_agents = 3;
    env.parking_model = 5;
    env.curr_states = {State(0, 0, 0), State(5, 0, 0), State(17, 0, 0)};
    env.curr_task_schedule = {-1, -1, 0};
    env.task_pool.emplace(0, Task(0, {16, 17}, 0));
    env.task_pool.at(0).agent_assigned = 2;
    env.task_pool.at(0).idx_next_loc = 1;
    schedule_initialize(1000, &env);
    reset_combined_parking_model(); // reference movement cost = 1 for this oracle
    const auto clearance = get_map_clearance();
    std::mt19937 rng(23);
    for (int profile = 0; profile < 3; ++profile)
    {
        flow.assign(18, Double4{0, 0, 0, 0});
        if (profile == 1) flow[17] = Double4{2, 2, 0, 0}; // protected cells still contribute
        if (profile == 2)
            for (auto& cell : flow)
                for (double& value : cell.d) value = (rng() % 6) / 2.0;
        vector<double> values(18, 0.0);
        double sum = 0;
        int count = 0;
        for (int loc = 0; loc < 18; ++loc)
        {
            for (double value : flow[loc].d) values[loc] += value;
            if (values[loc] > 0) { sum += values[loc]; ++count; }
        }
        const double reference = count ? sum / count : 0;
        for (bool use_traffic : {false, true})
        for (auto weights : vector<pair<double,double>>{{1, 0}, {0, 0.5}, {1, 0.5}, {0, 0.1}, {1, 0.1}, {0, 0}})
        {
            const auto [bc, bt] = weights;
            // Independent shortest-path enumeration exists only in the tests.
            vector<vector<long long>> dist(18, vector<long long>(18, 1000000000));
            for (int a = 0; a < 18; ++a)
            for (int b = 0; b < 18; ++b)
            {
                if (a == b) dist[a][b] = 0;
                if (std::abs(a / 6 - b / 6) + std::abs(a % 6 - b % 6) != 1) continue;
                long long movement = 1;
                if (use_traffic)
                {
                    const int d = a-b == 1 ? 0 : a-b == -1 ? 2 : a-b == 6 ? 1 : 3;
                    int occupancy = 0;
                    for (double value : flow[b].d) occupancy += static_cast<int>(value);
                    movement += static_cast<int>((flow[a].d[d]+1) * flow[b].d[(d+2)%4]) + occupancy/2;
                }
                dist[a][b] = 100 * movement;
            }
            for (int mid = 0; mid < 18; ++mid)
            for (int a = 0; a < 18; ++a)
            for (int b = 0; b < 18; ++b)
                dist[a][b] = std::min(dist[a][b], dist[a][mid] + dist[mid][b]);
            auto penalty = [&](int loc) {
                const double c = std::get<0>(clearance[loc]);
                const double fc = 1/(c*c); // neighbourhood tie-breakers are zero
                const double normalized = reference > 0 ? values[loc]/reference : 0;
                return std::llround(1600 * (bc*fc + bt*normalized));
            };
            long long optimum = LLONG_MAX;
            for (int a = 0; a < 17; ++a)
            for (int b = 0; b < 17; ++b)
                if (a != b)
                    optimum = std::min(optimum, dist[0][a] + penalty(a) + dist[5][b] + penalty(b));
            // Models 5/6/7 are coefficient settings of the sequential solver.
            for (int model : {7, 8})
            {
                vector<int> proposed;
                if (model == 7) schedule_plan_flow_dummy_combined(100, &env, flow, use_traffic, false, 16, bc, bt);
                if (model == 8) schedule_plan_flow_joint_task_parking(100, proposed, &env, flow, use_traffic, false, 16, bc, bt);
                const int a = env.dummy_parking_allocation.at(0).at(0);
                const int b = env.dummy_parking_allocation.at(1).at(0);
                check(a != b && a != 17 && b != 17 && env.dummy_parking_allocation.size() == 2,
                      "normalized traffic parking violated destination constraints");
                check(dist[0][a] + penalty(a) + dist[5][b] + penalty(b) == optimum,
                      "normalized traffic assignment differs from exhaustive optimum");
                check(env.curr_task_schedule == vector<int>({-1, -1, 0}) &&
                      env.task_pool.at(0).agent_assigned == 2 && env.task_pool.at(0).idx_next_loc == 1 &&
                      get_map_clearance() == clearance,
                      "parking changed fixed tasks or the clearance table");
                if (bc == 0 && (bt == 0 || reference == 0))
                    check(a == 0 && b == 5, "zero destination penalty should allow staying");
            }
        }
    }

    // Joint task quota and assignment objective with one flexible task and one
    // parking goal; an opened task remains fixed to agent 2.
    env.task_pool.emplace(1, Task(1, {4, 15}, 0));
    double mean = calculate_mean_nonzero_traffic(&env, flow, traffic);
    auto distance = [](int a, int b) { return std::abs(a/6-b/6) + std::abs(a%6-b%6); };
    auto joint_penalty = [&](int loc) {
        const double c = std::get<0>(clearance[loc]);
        return std::llround(100 * env.parking_strength * (1/(c*c) + 0.5*traffic[loc]/mean));
    };
    long long best = LLONG_MAX;
    for (int task_agent : {0, 1})
    for (int loc = 0; loc < 17; ++loc)
        if (loc != 4)
            best = std::min(best, 100LL * (distance(env.curr_states[task_agent].location, 4) +
                distance(env.curr_states[1-task_agent].location, loc)) + joint_penalty(loc));
    vector<int> proposed;
    schedule_plan_flow_joint_task_parking(100, proposed, &env, flow, false, false);
    check(proposed[2] == 0 && std::count(proposed.begin(), proposed.end(), 1) == 1 &&
          env.dummy_parking_allocation.size() == 1, "joint task quota or opened assignment changed");
    const int task_agent = proposed[0] == 1 ? 0 : 1;
    const int target = env.dummy_parking_allocation.at(1-task_agent).at(0);
    check(target != 4 && target != 17 && 100LL * (distance(env.curr_states[task_agent].location, 4) +
          distance(env.curr_states[1-task_agent].location, target)) + joint_penalty(target) == best,
          "joint normalized task-and-parking objective differs from exhaustive optimum");

    // Disabled terms must not inspect traffic or construct a clearance table.
    env.task_pool.erase(1);
    env.curr_task_schedule = {-1, -1, 0};
    env.parking_model = 6;
    schedule_initialize(1000, &env);
    flow.assign(18, Double4{1, 0, 0, 0});
    schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 0, 0.5);
    schedule_plan_flow_joint_task_parking(100, proposed, &env, flow, false, false, 16, 0, 0.1);
    check(get_map_clearance().empty(), "traffic-only parking computed clearance");
    flow.clear();
    schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 0, 0);
    schedule_plan_flow_joint_task_parking(100, proposed, &env, flow, false, false, 16, 0, 0);
    check(get_map_clearance().empty(), "zero-weight parking computed clearance");
    // Clearance-only must also skip traffic normalization (the empty snapshot
    // is intentionally invalid for any helper that tries to read traffic).
    schedule_plan_flow_dummy_combined(100, &env, flow, false, false, 16, 1, 0);
    const auto old_goals = env.dummy_parking_allocation;
    for (auto weights : vector<pair<double,double>>{{-1, 0.1}, {16, -0.1},
             {16, std::numeric_limits<double>::quiet_NaN()}, {std::numeric_limits<double>::infinity(), 0.1}})
    {
        bool rejected = false;
        try { schedule_plan_flow_dummy_combined(100, &env, flow, false, false, weights.first, 0, weights.second); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected && env.dummy_parking_allocation == old_goals, "invalid traffic weights accepted");
    }
}

void check_same_traffic(const vector<DefaultPlanner::Double4>& a,
                        const vector<DefaultPlanner::Double4>& b, const char* message)
{
    check(a.size() == b.size(), message);
    for (size_t i = 0; i < a.size(); ++i)
        for (int d = 0; d < 4; ++d)
            check(a[i].d[d] == b[i].d[d], message);
}

void test_pickup_traffic_arcs()
{
    using namespace DefaultPlanner;
    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.map[4] = 1;
    ListDigraph graph;
    ListDigraph::ArcMap<int> flow(graph);
    vector<ListDigraph::Node> nodes;
    boost::unordered_map<int,int> locations;
    for (int loc = 0; loc < 18; ++loc)
    {
        nodes.push_back(graph.addNode());
        locations[graph.id(nodes.back())] = loc;
    }
    const auto source = graph.addNode(), sink = graph.addNode();
    flow[graph.addArc(source, nodes[7])] = 99;
    flow[graph.addArc(nodes[8], sink)] = 99;
    auto add = [&](int from, int to, int count) { flow[graph.addArc(nodes[from], nodes[to])] = count; };
    add(7, 8, 3); add(7, 13, 2); add(7, 6, 4); add(7, 1, 5);
    add(5, 6, 9); // Row wrap, not a grid edge.
    add(7, 14, 9); add(7, 7, 9); add(3, 4, 9); // Diagonal, self and obstacle arcs.
    add(12, 13, 0);
    vector<Double4> traffic(18, Double4{0,0,0,0});
    traffic[7].d[0] = 0.25;
    auto expected = traffic;
    expected[7] = Double4{3.25,2,4,5};
    add_pickup_flow_to_background_traffic(graph, flow, locations, &env, traffic);
    check_same_traffic(traffic, expected, "pickup traffic used incorrect direction, multiplicity or non-movement arcs");
    vector<double> vertex;
    check(calculate_mean_nonzero_traffic(&env, traffic, vertex) == 14.25,
          "augmented traffic mean did not include pickup movement");
}

SharedEnvironment pickup_test_environment(int model)
{
    SharedEnvironment env;
    env.rows = 3;
    env.cols = 6;
    env.map.assign(18, 0);
    env.num_of_agents = 3;
    env.parking_model = model;
    env.curr_states = {State(0,0,0), State(5,0,0), State(17,0,0)};
    env.curr_task_schedule = {-1,-1,1};
    env.task_pool.emplace(0, Task(0, {2}, 0));
    env.task_pool.emplace(1, Task(1, {17,16}, 0));
    env.task_pool.at(1).idx_next_loc = 1;
    env.task_pool.at(1).agent_assigned = 2;
    return env;
}

void test_pickup_task_flow()
{
    using namespace DefaultPlanner;
    vector<Double4> delivery(18, Double4{0,0,0,0});
    delivery[17].d[2] = 1.25;
    for (bool traffic_costs : {false, true})
    {
        auto original = pickup_test_environment(7);
        auto baseline = original;
        vector<int> before, after;
        schedule_plan_flow(1000, before, &baseline, delivery, traffic_costs, false);
        const auto baseline_guides = get_guide_path();
        vector<Double4> augmented(18, Double4{99,99,99,99});
        for (int repeat = 0; repeat < 2; ++repeat)
        {
            auto env = original;
            schedule_plan_flow(1000, after, &env, delivery, traffic_costs, false, &augmented);
            check(after == before && get_guide_path() == baseline_guides,
                  "pickup augmentation changed first-flow assignments or paths");
            auto expected = delivery;
            expected[0].d[0] += 1;
            expected[1].d[0] += 1;
            check_same_traffic(augmented, expected, "pickup flows accumulated or counted delivery agents again");
            check(delivery[0].d[0] == 0 && delivery[17].d[2] == 1.25,
                  "delivery snapshot was modified");
        }
        auto empty = original;
        empty.task_pool.erase(0);
        schedule_plan_flow(1000, after, &empty, delivery, traffic_costs, false, &augmented);
        check_same_traffic(augmented, delivery, "no-task call retained the previous pickup flow");
        auto at_pickup = original;
        at_pickup.task_pool.at(0).locations = {0};
        schedule_plan_flow(1000, after, &at_pickup, delivery, traffic_costs, false, &augmented);
        check_same_traffic(augmented, delivery, "zero-length pickup route counted terminal flow as traffic");
    }
}

void test_sequential_pickup_dispatch()
{
    using namespace DefaultPlanner;
    vector<Double4> delivery(18, Double4{0,0,0,0});
    delivery[17].d[2] = 1.25;
    for (int model : {4,5,6,7,8})
    for (bool traffic_costs : {false, true})
    for (bool enabled : {false, true})
    {
        TaskScheduler scheduler;
        *scheduler.env = pickup_test_environment(model);
        scheduler.set_flow(delivery);
        scheduler.set_use_traffic(traffic_costs);
        scheduler.pickup_traffic = enabled;
        scheduler.initialize(1000);
        const auto original = *scheduler.env;
        vector<int> expected_schedule, actual_schedule;
        vector<Double4> augmented;
        if (model == 8)
            schedule_plan_flow_joint_task_parking(1000, expected_schedule, scheduler.env,
                delivery, traffic_costs, false, original.parking_strength, 1, 0.5);
        else
        {
            schedule_plan_flow(1000, expected_schedule, scheduler.env, delivery, traffic_costs, false,
                               enabled ? &augmented : nullptr);
            const auto& parking = enabled ? augmented : delivery;
            if (model == 4) schedule_plan_flow_dummy(1000, scheduler.env, parking, traffic_costs, false);
            if (model == 5) schedule_plan_flow_dummy_combined(1000, scheduler.env, parking, traffic_costs, false,
                                                           original.parking_strength, 1, 0);
            if (model == 6) schedule_plan_flow_dummy_combined(1000, scheduler.env, parking, traffic_costs, false,
                                                           original.parking_strength, 0, 0.5);
            if (model == 7) schedule_plan_flow_dummy_combined(1000, scheduler.env, parking, traffic_costs, false,
                                                           original.parking_strength, 1, 0.5);
        }
        const auto expected_goals = scheduler.env->dummy_parking_allocation;
        const auto expected_guides = get_guide_path();
        *scheduler.env = original;
        scheduler.plan(1000, actual_schedule);
        check(actual_schedule == expected_schedule && scheduler.env->dummy_parking_allocation == expected_goals &&
              get_guide_path() == expected_guides,
              "scheduler did not pass the expected traffic snapshot to parking");
        check_same_traffic(scheduler.background_flow, delivery, "scheduler retained augmented traffic between calls");
    }
}

void test_map_adaptive_parking_strength()
{
    using namespace DefaultPlanner;
    check(calculate_parking_strength(33, 57) == 22.5, "warehouse-small B is incorrect");
    check(std::abs(calculate_parking_strength(64, 64) - 32.0) < 1e-12,
          "random-map B lost fractional precision");
    check(std::abs(calculate_parking_strength(140, 500) - 160.0) < 1e-12,
          "sortation-large B is incorrect");
    check(calculate_parking_strength(500, 140) == calculate_parking_strength(140, 500),
          "B depends on map orientation");
    check(calculate_parking_strength(INT_MAX, INT_MAX) == static_cast<double>(INT_MAX)/2.0,
          "map dimension sum overflowed before floating-point conversion");
    for (double beta : {-1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    {
        bool rejected = false;
        try { calculate_parking_strength(3, 6, beta); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid beta was accepted");
    }
    for (auto dimensions : vector<pair<int,int>>{{0, 6}, {3, -1}})
    {
        bool rejected = false;
        try { calculate_parking_strength(dimensions.first, dimensions.second); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected, "invalid map dimensions were accepted");
    }
    bool overflow_rejected = false;
    try { calculate_parking_strength(140, 500, std::numeric_limits<double>::max()); }
    catch (const std::overflow_error&) { overflow_rejected = true; }
    check(overflow_rejected, "overflowing adaptive strength was accepted");

    for (int model : {5, 6, 7, 8})
    for (double beta : {0.0, 0.75, 2.0})
    {
        TaskScheduler scheduler;
        auto& env = *scheduler.env;
        env.rows = 3;
        env.cols = 6;
        env.map.assign(18, 0);
        env.num_of_agents = 2;
        env.curr_states = {State(0, 0, 0), State(5, 0, 0)};
        env.curr_task_schedule = {-1, -1};
        env.parking_model = model;
        env.parking_beta = beta;
        vector<Double4> flow(18, Double4{0, 0, 0, 0});
        flow[0] = Double4{10, 0, 0, 0};
        scheduler.set_flow(flow);
        scheduler.initialize(1000);
        check(env.parking_strength == beta * 3.0, "initialization did not set map-adaptive B");
        const double B = env.parking_strength;
        vector<int> proposed;
        auto run = [&](bool explicit_B, double traffic_weight) {
            if (model == 5 || model == 6) {
                schedule_plan_flow_dummy_combined(1000, &env, flow, false, false, B,
                                                 model == 5 ? 1 : 0,
                                                 model == 5 ? 0 : traffic_weight);
            } else if (model == 7) {
                if (explicit_B) schedule_plan_flow_dummy_combined(1000, &env, flow, false, false, B, 1, traffic_weight);
                else schedule_plan_flow_dummy_combined(1000, &env, flow, false, false);
            } else {
                if (explicit_B) schedule_plan_flow_joint_task_parking(1000, proposed, &env, flow, false, false, B, 1, traffic_weight);
                else schedule_plan_flow_joint_task_parking(1000, proposed, &env, flow, false, false);
            }
        };
        const double api_traffic_weight = 0.5;
        run(true, api_traffic_weight);
        const auto explicit_goals = env.dummy_parking_allocation;
        run(false, api_traffic_weight);
        check(env.dummy_parking_allocation == explicit_goals,
              "default parking API did not use initialized B");
        run(true, 0.5);
        const auto expected = env.dummy_parking_allocation;
        scheduler.plan(1000, proposed);
        check(env.dummy_parking_allocation == expected && env.parking_strength == B,
              "scheduler dispatch changed adaptive B or existing traffic coefficients");
        if (beta == 0.0)
            check(expected.at(0).at(0) == 0 && expected.at(1).at(0) == 5,
                  "zero beta must allow zero-cost staying");
    }
}

void test_parking_model_dispatch()
{
    for (int model : {6, 7, 8})
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
        test_parking_history();
        test_gzip_results();
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
        test_boundary_clearance();
        test_clearance_disk_cache();
        test_neighbourhood_clearance();
        test_shared_clearance_consumers();
        test_parking_cost_validation();
        test_normalized_traffic_parking();
        test_parking_model_dispatch();
        test_map_adaptive_parking_strength();
        test_pickup_traffic_arcs();
        test_pickup_task_flow();
        test_sequential_pickup_dispatch();
        std::cout << "All commit-window tests passed.\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
