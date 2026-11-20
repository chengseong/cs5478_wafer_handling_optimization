"""
===============================================================================
CS5478 Project Milestone: Non-Prehensile Wafer Transfer Simulation Suite
===============================================================================
Unified Runner & Comparative Benchmark Demonstrator (Phases 1 through 5)

Usage:
  python run_milestone_demo.py --phase 1    # Phase 1: 1D Rigid Box Anchor Baseline (Friction Calibration)
  python run_milestone_demo.py --phase 2    # Phase 2: Classical Baselines & Failure Modes (Runs 1-4)
  python run_milestone_demo.py --phase 3    # Phase 3: Joint-Space TOPP-RA 8-Facet Polyhedral Constraint
  python run_milestone_demo.py --phase 4    # Phase 4: Cleanroom 90° EFEM Layout (Active Physical Collisions)
  python run_milestone_demo.py --phase 5    # Phase 5: RNEA Inverse Dynamics Torque Decoupling (Full Cycle)
  python run_milestone_demo.py --all        # Sequential benchmark across all phases
===============================================================================
"""

import sys
import argparse
import subprocess


PHASE_SCRIPTS = {
    1: ("verify_phase1_baseline.py", "Phase 1: Physics Calibration & 1D Anchor Baseline"),
    2: ("verify_phase2_baselines.py", "Phase 2: Kinematic Coupling Fallacy & Baselines (Runs 1-4)"),
    3: ("verify_phase3_joint_toppra.py", "Phase 3: Joint-Space TOPP-RA with 8-Facet 2-Norm Constraint"),
    4: ("verify_phase4_multisegment.py", "Phase 4: Cleanroom 90° EFEM Enclosure (Active Collisions)"),
    5: ("verify_phase5_decoupled_efem.py", "Phase 5: RNEA Dynamic Decoupling & Torque Feedforward")
}


def run_phase(phase_num):
    if phase_num not in PHASE_SCRIPTS:
        print(f"Error: Unknown phase {phase_num}. Choose from 1 to 5.")
        return False

    script_name, description = PHASE_SCRIPTS[phase_num]
    print("\n" + "=" * 75)
    print(f"EXECUTING: {description}")
    print(f"Target Script: {script_name}")
    print("=" * 75 + "\n")

    cmd = [sys.executable, script_name]
    ret = subprocess.run(cmd)
    return ret.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="CS5478 Milestone Benchmark & Slide Demonstrator")
    parser.add_argument("--phase", type=int, choices=[1, 2, 3, 4, 5], help="Run a specific phase (1 to 5)")
    parser.add_argument("--all", action="store_true", help="Run all phases sequentially for full demonstration")

    args = parser.parse_args()

    if args.all:
        print("\nStarting Full Benchmark Suite (Phases 1 through 5)...")
        for p in range(1, 6):
            success = run_phase(p)
            if not success:
                print(f"\nPhase {p} encountered an issue or was closed by user.")
                break
    elif args.phase:
        run_phase(args.phase)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

