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
SIMULATION_TIME = 1000
PLAN_TIME_LIMIT = 1000
PREPROCESS_TIME_LIMIT = 30000
OUTPUT_SCREEN = 1

# Sweep options from src/driver.cpp
SCHEDULE_MODELS = [1]          # -m / --scheduleModel
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
        for model, use_traffic in combos:
            run_idx += 1
            run_tag = (
                f"{instance.stem}_m{model}_w{window}_u{int(use_traffic)}"
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
