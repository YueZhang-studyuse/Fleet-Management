#!/usr/bin/env python3
"""Run simulation sweeps sequentially from the repository root.

Examples:
    python3 run_exp.py
        Run warehouseSmall with commit window 1 and the settings below.

    python3 run_exp.py --domains empty:3
        Run instances/empty/ with commitWindow=3: schedule and plan once,
        then execute three simulation steps before replanning.

    python3 run_exp.py --domains empty:3 random:2 warehouseSmall:5 --skipLess
        Run three domains with their own commit windows, keeping only
        instances whose team size is greater than the task-reveal count.

Command-line options:
    --domains DOMAIN[:WINDOW] [DOMAIN[:WINDOW] ...]
        Domains: empty, random, warehouseSmall, warehouseLarge, sortationLarge.
        WINDOW is a positive commit window, not a parking-model number.
        A bare name such as empty uses window 1. Default: warehouseSmall:1.
    --skipLess
        Skip instances with team size <= task-reveal count.
    -h, --help
        Show command-line help without running experiments.

Edit these settings inside this script:
    PARKING_MODELS = []          Use the executable's default parking model.
    PARKING_MODELS = [9]         Run only traffic-adjusted clearance parking.
    PARKING_MODELS = [5, 7, 9]   Sweep clearance-only, additive clearance/traffic,
                                and traffic-adjusted clearance parking.
    SCHEDULE_MODELS              Scheduler-model values passed to --scheduleModel.
    USE_TRAFFIC_VALUES          [True], [False], or [False, True]; controls
                                traffic-aware scheduling movement costs.
    SIMULATION_TIME             Number of simulation steps per run.
    PLAN_TIME_LIMIT             Budget per step in ms; each planning call gets
                                commitWindow * PLAN_TIME_LIMIT milliseconds.
    PREPROCESS_TIME_LIMIT       Initialization budget in milliseconds.
    OUTPUT_SCREEN              Result detail level: 1 full, 2 omit events/tasks,
                                3 also omit errors, times, starts and paths.
    EXPERIMENT_NAME             Subfolder identifying this experiment batch.
    CONTINUE_ON_ERROR           Whether to continue after a failed simulation.

Each instance runs every combination of the scheduler, parking, and traffic
settings. An empty PARKING_MODELS list runs once per other combination and
omits --parkingModel, allowing the executable's default to apply.
Results go to outputs/<domain>_sweep/<EXPERIMENT_NAME>/{json,logs,cout}/.
Filenames include the commit window and, when selected explicitly, parking model.
"""

import argparse
import itertools
import subprocess
import sys
from pathlib import Path


# Basic paths
EXE_PATH = Path("./build/lifelong")
INSTANCES_ROOT = Path("./instances")
OUTPUT_ROOT = Path("./outputs")
DOMAIN_PREFIXES = {
    "warehouseSmall": ("WS",),
    "warehouseLarge": ("WL", "warehouseLarge"),
    "sortationLarge": ("SL", "sortationLarge"),
    "random": ("R", "random"),
    "empty": ("empty",),
}

# Name this run to keep outputs from different sweeps separated.
EXPERIMENT_NAME = "ws_small_exp_01"

# Common runtime options
SIMULATION_TIME = 1000
PLAN_TIME_LIMIT = 1000
PREPROCESS_TIME_LIMIT = 30000
OUTPUT_SCREEN = 1

# Sweep options from src/driver.cpp
SCHEDULE_MODELS = [1]          # -m / --scheduleModel
PARKING_MODELS = []            # --parkingModel; empty uses the executable's default
USE_TRAFFIC_VALUES = [True]   # -u / --useTraffic

# Set True if you want to keep running after a failed command.
CONTINUE_ON_ERROR = False


def tf(value: bool) -> str:
    return "true" if value else "false"


def parse_domain(value: str) -> tuple[str, int]:
    domain, separator, window_text = value.partition(":")
    if domain not in DOMAIN_PREFIXES:
        raise argparse.ArgumentTypeError(
            f"Unknown domain '{domain}'; choose from {', '.join(DOMAIN_PREFIXES)}."
        )
    try:
        window = int(window_text) if separator else 1
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Invalid commit window in '{value}'; use DOMAIN:POSITIVE_INTEGER."
        ) from None
    if window <= 0 or window > 2_147_483_647 // PLAN_TIME_LIMIT:
        raise argparse.ArgumentTypeError(
            "Commit window must be positive and window * PLAN_TIME_LIMIT must fit in a C++ int."
        )
    return domain, window


def main() -> int:
    parser = argparse.ArgumentParser(description="Run sweeps for selected domains.")
    parser.add_argument(
        "--domains",
        nargs="+",
        type=parse_domain,
        metavar="DOMAIN[:WINDOW]",
        default=[("warehouseSmall", 1)],
        help=(
            "Domains with individual commit windows, e.g. warehouseSmall:5 random:2. "
            "Bare names use window 1 (default: warehouseSmall:1). "
            f"Domains: {', '.join(DOMAIN_PREFIXES)}."
        ),
    )
    parser.add_argument(
        "--skipLess",
        action="store_true",
        help="Skip cases where team size <= task reveal.",
    )
    args = parser.parse_args()
    domain_windows = {}
    for domain, window in args.domains:
        if domain in domain_windows and domain_windows[domain] != window:
            parser.error(f"Domain '{domain}' has conflicting commit windows; specify one per domain.")
        domain_windows[domain] = window
    domains = list(domain_windows)
    parking_models = list(dict.fromkeys(PARKING_MODELS)) or [None]

    if not EXE_PATH.exists():
        print(f"Executable not found: {EXE_PATH}", file=sys.stderr)
        return 2

    instances_by_domain = {}
    for domain in domains:
        instances_dir = INSTANCES_ROOT / domain
        patterns = [f"{prefix}_*_*.json" for prefix in DOMAIN_PREFIXES[domain]]
        if not instances_dir.is_dir():
            print(f"Instances directory not found: {instances_dir}", file=sys.stderr)
            return 2
        instance_files = sorted(
            path for pattern in patterns for path in instances_dir.glob(pattern)
        )
        if not instance_files:
            print(
                f"No instance files matching {patterns} under {instances_dir}",
                file=sys.stderr,
            )
            return 2
        instances_by_domain[domain] = instance_files

    combos = list(
        itertools.product(
            SCHEDULE_MODELS,
            parking_models,
            USE_TRAFFIC_VALUES
        )
    )

    jobs = []
    for domain, instance_files in instances_by_domain.items():
        for instance in instance_files:
            # Instance name is <domain>_<taskReveal>_<teamSize>.
            parts = instance.stem.split("_")
            if args.skipLess and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
                reveal, team_size = int(parts[1]), int(parts[2])
                if team_size <= reveal:
                    print(f"Skipping {instance.name}: team size {team_size} <= task reveal {reveal}")
                    continue
            jobs.append((domain, instance))

    total_runs = len(jobs) * len(combos)
    failures = 0
    run_idx = 0

    print(f"Experiment: {EXPERIMENT_NAME}")
    print("Domains: " + ", ".join(
        f"{domain} (commitWindow={window})" for domain, window in domain_windows.items()
    ))
    print("Parking models: " + ", ".join(
        str(model) if model is not None else "executable default" for model in parking_models
    ))
    for domain in domains:
        print(f"Run folder: {OUTPUT_ROOT / f'{domain}_sweep' / EXPERIMENT_NAME}")
    print(f"Instance jobs: {len(jobs)}")
    print(f"Option combinations: {len(combos)}")
    print(f"Total runs: {total_runs}")

    # Sequential execution in the main thread: one command at a time.
    for domain, instance in jobs:
        window = domain_windows[domain]
        run_root = OUTPUT_ROOT / f"{domain}_sweep" / EXPERIMENT_NAME
        json_dir = run_root / "json"
        log_dir = run_root / "logs"
        cout_dir = run_root / "cout"
        for directory in (json_dir, log_dir, cout_dir):
            directory.mkdir(parents=True, exist_ok=True)
        for model, parking_model, use_traffic in combos:
            run_idx += 1
            parking_tag = f"_pm{parking_model}" if parking_model is not None else ""
            run_tag = (
                f"{instance.stem}_m{model}{parking_tag}_w{window}_u{int(use_traffic)}"
            )
            out_file = json_dir / f"{run_tag}.json"
            log_file = log_dir / f"{run_tag}.log"
            cout_file = cout_dir / f"{run_tag}.txt"

            cmd = [
                str(EXE_PATH),
                "--inputFile",
                str(instance),
                "-o",
                str(out_file),
                "-s",
                str(SIMULATION_TIME),
                "-t",
                str(PLAN_TIME_LIMIT),
                "-p",
                str(PREPROCESS_TIME_LIMIT),
                "-c",
                str(OUTPUT_SCREEN),
                "-l",
                str(log_file),
                "-m",
                str(model),
                "-w",
                str(window),
                "-u",
                tf(use_traffic),
            ]
            if parking_model is not None:
                cmd.extend(["--parkingModel", str(parking_model)])

            print(
                f"[{run_idx}/{total_runs}] {instance.name} "
                f"m={model} parking={parking_model if parking_model is not None else 'default'} "
                f"w={window} u={int(use_traffic)}"
            )
            with cout_file.open("w", encoding="utf-8") as fout:
                result = subprocess.run(
                    cmd,
                    stdout=fout,
                    stderr=subprocess.STDOUT,
                    text=True,
                )

            if result.returncode != 0:
                failures += 1
                print(f"  FAILED with exit code {result.returncode}", file=sys.stderr)
                print(f"  cout: {cout_file}", file=sys.stderr)
                print(f"  log:  {log_file}", file=sys.stderr)
                if not CONTINUE_ON_ERROR:
                    print("Stopping on first failure.", file=sys.stderr)
                    return 1

    if failures > 0:
        print(f"Completed with {failures} failures.", file=sys.stderr)
        return 1

    print("All runs completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
