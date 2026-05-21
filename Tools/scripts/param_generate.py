#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""
Generate ArduPilot parameter documentation for all vehicles (or a specific one),
organized under a param_<branch> directory for easy cross-branch comparison.

Usage:
    python Tools/scripts/param_generate.py
    python Tools/scripts/param_generate.py --vehicle Copter
    python Tools/scripts/param_generate.py --output-dir /tmp/params_main
"""

import argparse
import os
import subprocess
import sys

VEHICLES = ["Copter", "Plane", "Rover", "Sub", "Tracker", "Blimp", "AP_Periph"]

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_REPO_ROOT = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', '..'))
_PARAM_PARSE = os.path.join(_SCRIPT_DIR, '..', 'autotest', 'param_metadata', 'param_parse.py')


def get_branch_name():
    result = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        capture_output=True,
        text=True,
        cwd=_REPO_ROOT,
    )
    if result.returncode != 0:
        print("Error getting git branch: " + result.stderr.strip(), file=sys.stderr)
        sys.exit(1)
    branch = result.stdout.strip()
    # Replace characters that are invalid in directory names
    return branch.replace('/', '_').replace('\\', '_')


def generate_params(vehicle, output_base):
    vehicle_dir = os.path.join(output_base, vehicle)
    os.makedirs(vehicle_dir, exist_ok=True)

    cmd = [sys.executable, _PARAM_PARSE, "--vehicle", vehicle, "--format", "all"]
    print("  {} -> {}".format(vehicle, vehicle_dir))
    result = subprocess.run(cmd, cwd=vehicle_dir)
    if result.returncode != 0:
        print("  ERROR: param_parse.py failed for {}".format(vehicle), file=sys.stderr)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Generate parameter docs for ArduPilot vehicles, organised by git branch.",
    )
    parser.add_argument(
        "--vehicle",
        choices=VEHICLES + ["all"],
        default="all",
        metavar="VEHICLE",
        help="Vehicle to generate for: {} or all (default: all)".format(", ".join(VEHICLES)),
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Base output directory (default: param_<branch> inside the repo root)",
    )
    args = parser.parse_args()

    branch = get_branch_name()

    if args.output_dir:
        output_base = os.path.abspath(args.output_dir)
    else:
        output_base = os.path.join(_REPO_ROOT, "param_" + branch)

    vehicles = VEHICLES if args.vehicle == "all" else [args.vehicle]

    print("Branch : " + branch)
    print("Output : " + output_base)
    print("Vehicles: " + ", ".join(vehicles))
    print()

    failed = []
    for vehicle in vehicles:
        if not generate_params(vehicle, output_base):
            failed.append(vehicle)

    print()
    if failed:
        print("Failed: " + ", ".join(failed), file=sys.stderr)
        sys.exit(1)

    print("Done. Output in: " + output_base)


if __name__ == "__main__":
    main()
