# Start-Kit

## Join the competition

Log in to the [competition website](http://www.leagueofrobotrunners.org/) with a GitHub account, and we will automatically create a private GitHub submission repo for you.
The repo will be the place where you submit codes. In the `My Submission` page, you can click "My Repo" to open your GitHub submission repo page.

## Clone your submission repo

Clone your submission repo to your local machine. The repo contains starter codes to help you prepare your submission.

```
$ git clone git@github.com:your_submission_repo_address
$ cd your_submission_repo
```

## Compile the start-kit

### Dependencies

- [cmake >= 3.16](https://cmake.org/)
- [libboost >= 1.49.0](https://www.boost.org/)
- Python3 >= 3.11 and [pybind11](https://pybind11.readthedocs.io/en/stable/) >=2.10.1 are recommanded for python interface user.

Install dependencies on Ubuntu or Debian Linux:
```shell
sudo apt-get update
sudo apt-get install build-essential libboost-all-dev python3-dev python3-pybind11 
```

[Homebrew](https://brew.sh/) is recomanded for installing dependencies on Mac OS.

### Compiling

Using `compile.sh`:
```shell
./compile.sh
```

Using cmake: 
```shell
mkdir build
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build -j
```

## Run the start kit

Running the start-kit using commands: 
```shell
./build/lifelong --inputFile the_input_file_name -o output_file_location
```

for example:
```shell
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```

more info on help:
```shell
./build/lifelong --help
```

## Parking models

Select a parking strategy with `--parkingModel N`:

| N | Strategy |
|---|---|
| 1 | Random parking locations |
| 2 | Stay at the current location |
| 3 | Clearance preselection |
| 4 | Parking flow with clearance-preselected destinations |
| 5 | Parking flow over all eligible cells with a clearance penalty |
| 6 | Parking flow with a normalized linear traffic penalty |
| 7 | Task assignment first, then parking flow with combined clearance and traffic penalties |
| 8 | One joint flow assigning tasks and parking goals |

Models 7 and 8 use hard-coded weights `B=16`, `b_c=1`, and `b_t=0.1`
in `TaskScheduler::plan`. Model 7 retains the separate `schedule_plan_flow`
task assignment. Model 8 skips it and calls `schedule_plan_flow_joint_task_parking`
directly. The joint objective can trade task travel distance against parking cost
while assigning as many tasks as possible and preserving already-opened tasks.
`--useTraffic` independently controls traffic-aware movement costs.

Models 6–8 use the normalized additive destination penalty
`B * avg_step_cost * (b_c * F_C + b_t * T / mean_T)`.
`T` is the sum of the four directional background-flow values; `mean_T` is
computed once per parking call over all traversable cells with positive traffic,
including protected task targets. An all-zero field contributes zero traffic cost.
The traffic term is skipped when `b_t=0`; clearance work is skipped when `b_c=0`.
Traffic-weighted movement costs and task-assignment semantics are unchanged.

Model 6 uses `b_c=0`, `B=16`, and `b_t=0.1`. Its C++ function now takes
`B` and `b_t` as its last two arguments. Models 7 and 8 use `B=16`, `b_c=1`,
and `b_t=0.1`; the corresponding parameters remain available in their C++ APIs.
The former model 9 has been removed; select models 6–8 for traffic-based parking.

Clearance preprocessing stores one shared table of `(double clearance, int tie-breaker)`.
After the iterative table is built, `calculate_neighbourhood_clearance(env)`
replaces it with distance-decayed neighbourhood scores and refreshes its ranking.
The default decay is 0.5, using Chebyshev rings 1–6 and zero tie-breakers.
Both clearance-based preselection and parking penalties use the updated table.
For the original iterative-clearance baseline, comment out that single call
in `ensure_parking_clearance` in `default_planner/scheduler.cpp` and rebuild.

Parking flows in models 5–8 scale costs by 100 before integer optimization,
rounding parking penalties to the nearest 0.01 cost unit (at most 0.005
rounding error per parking assignment). Movement costs use the same scale.
Near-tied assignments can differ from the previous scale of 1000. Invalid
or oversized parking costs are rejected rather than clamped.

```shell
./build/lifelong --inputFile input.json --parkingModel 8 --commitWindow 5
```

## Multi-step commits

The result JSON includes `parking`, an array indexed by simulation timestep:

```json
{"parking": [[1, 3], [0, 1, 3], []]}
```

`parking[t]` lists the zero-based agent IDs with no assigned task at the start
of step `t -> t+1`, after any new schedule has been validated and accepted.
For example, the first entry above means agents 1 and 3 are unassigned at
timestep 0. An empty entry means every agent has a task. These agents may be
moving toward parking goals; the list does not imply they are stationary.
The history includes every executed step, including timeout waits, and is
included at all output detail levels. Its length is the number of simulated
steps; a zero-step simulation produces `[]`.

Use `--commitWindow W` (or `-w W`) to schedule and plan once per W simulated
steps. `--planTimeLimit T` remains the budget **per step**, in milliseconds;
each planning call receives a total budget of `W * T`. Both options must be
positive, and their product must fit in a C++ `int`.

```shell
./build/lifelong --inputFile input.json --commitWindow 5 --planTimeLimit 100
```

`Entry::compute` returns `std::vector<std::vector<Action>>`, indexed as
`plan[step][agent]`, and one task assignment for the window. To implement a
native multi-step planner, override:

```cpp
void plan(int time_limit, std::vector<std::vector<Action>>& commits) override;
```

The requested length is `env->commit_window`. `MAPFPlanner.cpp` forwards the
remaining time budget to `DefaultPlanner::plan` in `default_planner/planner.cpp`.
That function optimizes guide paths once, then runs W joint PIBT steps using
predicted positions and per-agent errand indices. It resets move reservations
each step and refreshes guides when goals change, using goal distances if the
refresh budget is exhausted. The shared simulation states and tasks are never
advanced during planning. The scheduler runs once per window.

The single-step `plan(int, std::vector<Action>&)` interface remains available
for explicit single-step calls. Custom committed planners should implement the
batch overload above; overriding only the single-step overload does not replace
the committed planner.

The simulator executes up to W returned steps in order, fills a short or empty
batch with ordinary waits, and ignores steps beyond W. Every step validates
joint moves and advances tasks. An invalid move becomes an all-agent wait and
the rest of that batch is replaced with waits because its predicted starting
states are no longer valid. Task assignments are accepted once per window;
new-task and free-agent notifications accumulate until the next planning call.

Planning runs synchronously. After executing the window, the simulator appends
`ceil(max(0, elapsed_ms - W*T - 5) / T)` all-agent timeout waits. A hard-coded
5 ms tolerance applies once per planning call, regardless of W; the budget
passed to the planner remains W*T. For example, W=3 and T=100 allow up to
305 ms without a timeout; a 350 ms call adds one timeout wait after the commits.
Timeout waits also advance task errands. Execution always stops at
`simulationTime`, including within a batch or its timeout waits.
`numEntryTimeouts` counts timeout waits actually executed; `plannerTimes` has
one elapsed time in seconds per planning call. Timeout waits appear as `T` in
the output paths, ordinary waits as `W`.

Because planning is synchronous, timeout accounting happens after a planner
returns; it cannot interrupt a planner that never returns. Preprocessing keeps
its existing separate timeout enforcement.

Build and run the C++ regression tests with:

```shell
cmake -S . -B build -DBUILD_TESTING=ON -DPYTHON=OFF
cmake --build build -j
ctest --test-dir build --output-on-failure
```

For experiment sweeps, give each domain its own commit window in `run_exp.py`:

```shell
python3 run_exp.py --domains warehouseSmall:5 random:2 --skipLess
```

Every instance in a domain uses that domain's window. Bare domain names default
to window 1, and omitting `--domains` runs `warehouseSmall:1`. Conflicting
windows for the same domain are rejected. Output filenames include `_w5`,
`_w2`, etc. so different windows remain distinguishable.

The `empty` domain is also supported. Select parking models inside `run_exp.py`,
alongside the other sweep settings:

```python
PARKING_MODELS = [5, 6, 7, 8]
```

```shell
python3 run_exp.py --domains empty:3 warehouseSmall:5
```

Use model numbers 1–8. The default `PARKING_MODELS = []` leaves
`--parkingModel` out of the simulator command, so its
default applies. Explicit models add `_pm5`, `_pm7`, etc. to output filenames.

## Windows users
If you are a Windows user, the most straightforward method to utilize our start-kits is by employing the WSL (Windows Subsystem for Linux) subsystem. Follow these steps:
1. Install WSL, please refer to [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)
2. Open a shell in WSL and execute the following commands to install the necessary tools (CMake, GCC, Boost, pip, Pybind11):
```shell
sudo apt-get update
sudo apt-get install cmake g++ libboost-all-dev python3-dev python3-pip
pip install pybind11-global numpy
```
3. Employ the commands provided above to compile the start-kit.

While it's technically possible to use our start-kit with Cygwin, Mingw, and MSVC, doing so would be more complex compared to using WSL. You would likely need to configure the environment yourself.

If you are a docker user, another choice is to develop and test your python implementation under a docker environment. You can the re-create the evaluation environment locally on your machine. For more details, check out the [Test in Docker](./Prepare_Your_Submission.md#test-in-docker) section.

## Upgrade Your Start-Kit

If your private start-kit copy repo was created before a start-kit upgrade, you could run the script `./upgrade_start_kit.sh` to upgrade your start-kit to the latest version.

You can check `version.txt` to know the current version of your start-kit.

The `upgrade_start_kit.sh` will check which file is marked as an upgrade needed and pull those files from the start-kit. It will pull and stage the files, but not commit them. This allows you to review the changes before committing them. 

For files stated as unmodifiable in [Parepare_Your_Planner.md](./Prepare_Your_Submission.md), you always commit their changes.

⚠️ But please be aware that, the start-kit v2.1.0 introduces requested API changes on `task_pool`. This requires minor revision to your implementation to adapt to the new API.  
This change also impacts the implementation of function `update_goal_locations` in `src/Entry.cpp`, therefore, the upgrade script will pull the new version of `src/Entry.cpp` and may overwrite your changes. You could compare the difference using `git diff` and decide whether to revert some modifications or partially accept changes on this file. 

The upgrade script will not touch most of the participants' implementation file.
How every the example implementation in `python/pyMAPFPlanner.py`,`python/pyTaskScheduler.py`, `inc/MAPFPlanner.h`, `inc/TaskScheduler.h`, `src/MAPFPlanner.cpp`, `src/TaskScheduler.cpp`, `default_planner/planner.cpp` and `default_planner/scheduler.cpp` are updated with with new API and additional documentaion. You may want to view changes on these files. 

## Input output description

Please refer to the [Input_Output_Format.md](./Input_Output_Format.md).

## Prepare Your Planner

Please refer to the [Prepare_Your_Submission.md](./Prepare_Your_Submission.md).

## Debug and Visualise Your Planner
We provide a visualisation tool written in Python: [https://github.com/MAPF-Competition/PlanViz](https://github.com/MAPF-Competition/PlanViz).
It is able to visualise the output of the start-kit program and help participants debug the implementations. 

Please refer to the project website for more information. Also the document [Debug_and_Visualise_Your_Planner](./Debug_and_Visualise_Your_Planner.md) which provides helpful hints for interpreting and diagnosing planner output.

## Submission Instruction

Please refer to the [Submission_Instruction.md](./Submission_Instruction.md).



