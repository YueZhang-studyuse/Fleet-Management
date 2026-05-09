#!/usr/bin/env python3

import itertools
import subprocess
import sys
from pathlib import Path


# Basic paths
EXE_PATH = Path("./build/lifelong")
INSTANCES_DIR = Path("./instances/warehouseSmall")
INSTANCE_PATTERN = "WS_*.json"
OUTPUT_ROOT = Path("./outputs/warehouseSmall_sweep")

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
USE_TRAFFIC_VALUES = [False]   # -u / --useTraffic

# Set True if you want to keep running after a failed command.
CONTINUE_ON_ERROR = False


def tf(value: bool) -> str:
    return "true" if value else "false"


def main() -> int:
    if not EXE_PATH.exists():
        print(f"Executable not found: {EXE_PATH}", file=sys.stderr)
        return 2

    if not INSTANCES_DIR.is_dir():
        print(f"Instances directory not found: {INSTANCES_DIR}", file=sys.stderr)
        return 2

    instance_files = sorted(INSTANCES_DIR.glob(INSTANCE_PATTERN))
    if not instance_files:
        print(
            f"No instance files matching '{INSTANCE_PATTERN}' under {INSTANCES_DIR}",
            file=sys.stderr,
        )
        return 2

    run_root = OUTPUT_ROOT / EXPERIMENT_NAME
    json_dir = run_root / "json"
    log_dir = run_root / "logs"
    cout_dir = run_root / "cout"
    json_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    cout_dir.mkdir(parents=True, exist_ok=True)

    combos = list(
        itertools.product(
            SCHEDULE_MODELS,
            COMMIT_WINDOWS,
            USE_TRAFFIC_VALUES
        )
    )

    total_runs = len(instance_files) * len(combos)
    failures = 0
    run_idx = 0

    print(f"Experiment: {EXPERIMENT_NAME}")
    print(f"Run folder: {run_root}")
    print(f"Instances: {len(instance_files)}")
    print(f"Option combinations: {len(combos)}")
    print(f"Total runs: {total_runs}")

    # Sequential execution in the main thread: one command at a time.
    for instance in instance_files:
        for model, window, use_traffic in combos:
            run_idx += 1
            run_tag = (
                f"{instance.stem}__m{model}_w{window}_u{int(use_traffic)}"
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

            print(
                f"[{run_idx}/{total_runs}] {instance.name} "
                f"m={model} w={window} u={int(use_traffic)}"
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
