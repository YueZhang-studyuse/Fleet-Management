#!/usr/bin/env python3

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
}

# Name this run to keep outputs from different sweeps separated.
EXPERIMENT_NAME = "ws_small_exp_01"

# Common runtime options
SIMULATION_TIME = 2000
PLAN_TIME_LIMIT = 1000
PREPROCESS_TIME_LIMIT = 30000
OUTPUT_SCREEN = 1

# Sweep options from src/driver.cpp
SCHEDULE_MODELS = [1]          # -m / --scheduleModel
COMMIT_WINDOWS = [1]                 # -w / --commitWindow
USE_TRAFFIC_VALUES = [True]   # -u / --useTraffic

# -a / --activeAgents offsets below the agent count XXX in WS_XXX_YYY.json.
# ACTIVE_AGENT_OFFSETS = [40, 20, 0]
ACTIVE_AGENT_OFFSETS = [0]

# Set True if you want to keep running after a failed command.
CONTINUE_ON_ERROR = False


def tf(value: bool) -> str:
    return "true" if value else "false"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run sweeps for selected domains.")
    parser.add_argument(
        "--domains",
        nargs="+",
        choices=DOMAIN_PREFIXES,
        default=["warehouseSmall"],
        help="Domains to run (default: warehouseSmall).",
    )
    domains = list(dict.fromkeys(parser.parse_args().domains))

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
            COMMIT_WINDOWS,
            USE_TRAFFIC_VALUES
        )
    )

    jobs = []
    for domain, instance_files in instances_by_domain.items():
        for instance in instance_files:
            parts = instance.stem.split("_")
            agent_count = int(parts[2]) if len(parts) == 3 and parts[2].isdigit() else None
            active_list = (
                [agent_count - off for off in ACTIVE_AGENT_OFFSETS if agent_count > off]
                if agent_count is not None
                else []
            )
            if not active_list:
                print(f"Skipping {instance.name}: cannot derive activeAgents")
                continue
            for active in active_list:
                jobs.append((domain, instance, active))

    total_runs = len(jobs) * len(combos)
    failures = 0
    run_idx = 0

    print(f"Experiment: {EXPERIMENT_NAME}")
    print(f"Domains: {', '.join(domains)}")
    for domain in domains:
        print(f"Run folder: {OUTPUT_ROOT / f'{domain}_sweep' / EXPERIMENT_NAME}")
    print(f"Instance/activeAgents jobs: {len(jobs)}")
    print(f"Option combinations: {len(combos)}")
    print(f"Total runs: {total_runs}")

    # Sequential execution in the main thread: one command at a time.
    for domain, instance, active in jobs:
        run_root = OUTPUT_ROOT / f"{domain}_sweep" / EXPERIMENT_NAME
        json_dir = run_root / "json"
        log_dir = run_root / "logs"
        cout_dir = run_root / "cout"
        for directory in (json_dir, log_dir, cout_dir):
            directory.mkdir(parents=True, exist_ok=True)
        for model, window, use_traffic in combos:
            run_idx += 1
            run_tag = (
                f"{instance.stem}__a{active}_m{model}_w{window}_u{int(use_traffic)}"
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
                "-a",
                str(active),
            ]

            print(
                f"[{run_idx}/{total_runs}] {instance.name} "
                f"a={active} m={model} w={window} u={int(use_traffic)}"
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
