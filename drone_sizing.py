"""
drone_sizing.py
===============
Command-line entry point for the multirotor drone sizing tool.

This script orchestrates the full sizing pipeline from a mission input file
to a complete HTML or PDF report. It is the only file the user interacts
with directly.

Usage
-----
    python drone_sizing.py <inputs_file> [--output-dir <dir>]

Arguments:
    inputs_file         Path to the mission input JSON file.
                        Example: inputs/heliox_x1.json

Options:
    --output-dir DIR    Directory where the report will be written.
                        Default: current working directory.

    --help, -h          Show this help message and exit.

Examples
--------
    # Generate HTML report (format defined in inputs.json)
    python drone_sizing.py inputs/heliox_x1.json

    # Write report to a specific directory
    python drone_sizing.py inputs/my_mission.json --output-dir output/

Exit codes:
    0   Success — report generated successfully.
    1   Error — configuration, input, or computation failure.
    2   Usage error — wrong arguments.

Dependencies
------------
    Standard library : sys, os, argparse, traceback
    Third-party      : numpy, matplotlib (both required)
    Optional         : wkhtmltopdf system binary (required for PDF output only)

Configuration
-------------
    The physical model parameters are loaded from physical_config.json,
    located in the same directory as this script. The path is fixed in
    core/config_loader.py and does not need to be passed on the command line.
"""

import sys
import os
import argparse
import traceback


# ---------------------------------------------------------------------------
# Ensure the project root is on the Python path when run directly
# ---------------------------------------------------------------------------
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)


from core.config_loader import load_physical_config, load_mission_inputs
from report.builder import build
from report.renderer_html import render as render_html
from report.renderer_pdf import render as render_pdf


# ---------------------------------------------------------------------------
# ANSI color helpers (gracefully degraded on Windows)
# ---------------------------------------------------------------------------

def _supports_color() -> bool:
    """Return True if the terminal supports ANSI color codes."""
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


def _green(text: str) -> str:
    return f"\033[32m{text}\033[0m" if _supports_color() else text


def _yellow(text: str) -> str:
    return f"\033[33m{text}\033[0m" if _supports_color() else text


def _red(text: str) -> str:
    return f"\033[31m{text}\033[0m" if _supports_color() else text


def _bold(text: str) -> str:
    return f"\033[1m{text}\033[0m" if _supports_color() else text


# ---------------------------------------------------------------------------
# CLI argument parser
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    """Build and return the argument parser for the CLI."""
    parser = argparse.ArgumentParser(
        prog="drone_sizing.py",
        description=(
            "Multirotor drone sizing tool.\n"
            "Computes optimal propeller, battery, and generates a sizing report."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python drone_sizing.py inputs/my_mission.json\n"
            "  python drone_sizing.py inputs/my_mission.json --output-dir output/\n"
        )
    )
    parser.add_argument(
        "inputs_file",
        metavar="inputs_file",
        type=str,
        help="Path to the mission input JSON file."
    )
    parser.add_argument(
        "--output-dir",
        metavar="DIR",
        type=str,
        default=".",
        help="Directory where the report will be written. Default: current directory."
    )
    return parser


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    """Run the drone sizing pipeline and generate the report.

    Returns:
        int: Exit code. 0 on success, 1 on error, 2 on usage error.
    """
    parser = _build_parser()

    # Show help if no arguments provided
    if len(sys.argv) == 1:
        parser.print_help()
        return 2

    args = parser.parse_args()

    print(_bold("\n=== Drone Sizing Tool ===\n"))

    # --- Step 1: Load configuration files ---
    print("[ 1/5 ] Loading configuration files...")
    try:
        phys = load_physical_config()
        print(f"        {_green('✓')} physical_config.json loaded")
    except (FileNotFoundError, ValueError) as e:
        print(f"        {_red('✗')} physical_config.json: {e}")
        return 1

    try:
        inputs = load_mission_inputs(args.inputs_file)
        mission_name = inputs["mission"]["name"]
        print(f"        {_green('✓')} {args.inputs_file} loaded  [{mission_name}]")
    except (FileNotFoundError, ValueError) as e:
        print(f"        {_red('✗')} {args.inputs_file}: {e}")
        return 1

    # --- Step 2: Run sizing computations ---
    print("[ 2/5 ] Running sizing computations...")
    try:
        data = build(inputs, phys)
        print(f"        {_green('✓')} MTOW           = {data.masses.mtow_kg:.3f} kg")
        print(f"        {_green('✓')} m_bat optimal  = {data.masses.m_bat_kg:.3f} kg")
        print(f"        {_green('✓')} Diameter       = {data.prop.diameter_inch:.1f} in")
        print(f"        {_green('✓')} Endurance max  = {data.battery.endurance_max_min:.1f} min")
        print(f"        {_green('✓')} TWR effective  = {data.twr_result.twr:.3f}")
    except (ValueError, RuntimeError) as e:
        print(f"        {_red('✗')} Computation failed: {e}")
        traceback.print_exc()
        return 1

    # --- Step 3: Display warnings ---
    if data.warnings:
        print(f"[ 3/5 ] {_yellow('Warnings')} ({len(data.warnings)}):")
        for w in data.warnings:
            print(f"        {_yellow('⚠')} {w}")
    else:
        print(f"[ 3/5 ] No warnings — all constraints satisfied.")

    # --- Step 4: Generate curves (already done in builder) ---
    print("[ 4/5 ] Curves generated (4/4)")
    try:
        print(f"        {_green('✓')} Hover power vs MTOW")
        print(f"        {_green('✓')} Endurance vs payload mass")
        print(f"        {_green('✓')} Endurance vs battery mass")
        print(f"        {_green('✓')} Effective TWR vs battery mass")
    except Exception as e:
        print(f"        {_red('✗')} Curve generation failed: {e}")
        traceback.print_exc()
        return 1

    # --- Step 5: Render report ---
    fmt = inputs["output"]["format"]
    print(f"[ 5/5 ] Rendering {fmt.upper()} report...")
    try:
        if fmt == "html":
            output_path = render_html(data, output_dir=args.output_dir)
        elif fmt == "pdf":
            output_path = render_pdf(data, output_dir=args.output_dir)
        else:
            print(f"        {_red('✗')} Unknown format '{fmt}'. Must be 'html' or 'pdf'.")
            return 1

        file_size_kb = os.path.getsize(output_path) / 1024
        print(
            f"        {_green('✓')} Report written: {output_path}  "
            f"({file_size_kb:.1f} kB)"
        )
    except (EnvironmentError, RuntimeError, OSError) as e:
        print(f"        {_red('✗')} Render failed: {e}")
        return 1

    print(f"\n{_green(_bold('Done.'))}\n")
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
