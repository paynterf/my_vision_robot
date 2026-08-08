#!/usr/bin/env python3
"""
Post-Build OTA Helper for Visual Micro

Copies the newly built .hex file to the Pi5 so the wifi-ota-watcher
can flash it to the Teensy over serial.

Standardized destination (Aug 2026):
  pi@RobotPi5:~/my_vision_robot/firmware/latest.hex
"""

import sys
import os
import subprocess
from datetime import datetime

# ============== CONFIGURATION ==============
VERBOSE = False         # Set to True only when debugging
PI_USER_HOST = "pi@RobotPi5"
PI_DEST = "~/my_vision_robot/firmware/latest.hex"
# ===========================================


def main():
    print("\n" + "=" * 60)
    print("Post-Build OTA Transfer to Pi5")
    print("=" * 60)

    if VERBOSE:
        print("\n=== Visual Micro Path Variables ===")
        print(f"build.project_path              = {sys.argv[3] if len(sys.argv) > 3 else 'Not provided'}")
        print(f"vm.runtime.build.intermediate_output_path = {sys.argv[4] if len(sys.argv) > 4 else 'Not provided'}")
        print(f"vm.runtime.build.final_output_path = {sys.argv[1] if len(sys.argv) > 1 else 'Not provided'}")
        print("=" * 40)

    if len(sys.argv) < 4:
        print("ERROR: Not enough arguments provided.")
        print("Expected: final_output_path  project_name  project_path  intermediate_output_path")
        input("Press Enter to close...")
        sys.exit(1)

    intermediate_path = sys.argv[4].rstrip('\\')
    project_name = sys.argv[2] if len(sys.argv) > 2 else "Unknown"
    hex_path = f"{intermediate_path}\\{project_name}.hex"

    if VERBOSE:
        print("\n=== Derived File Paths ===")
        print(f"Project Name     : {project_name}")
        print(f"Intermediate Dir : {intermediate_path}")
        print(f"Hex File Path    : {hex_path}")
        print("=" * 40)
        print(f"Time : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    if not os.path.exists(hex_path):
        print(f"ERROR: Hex file not found: {hex_path}")
        input("Press Enter to close...")
        sys.exit(1)

    print(f"\nSource : {hex_path}")
    print(f"Dest   : {PI_USER_HOST}:{PI_DEST}")
    print("\nCopying .hex to Pi5...")

    try:
        subprocess.run(
            ["scp", hex_path, f"{PI_USER_HOST}:{PI_DEST}"],
            capture_output=True, text=True, check=True
        )
        print("SUCCESS: .hex file copied to Pi5.")
    except subprocess.CalledProcessError as e:
        print("\nERROR: Failed to copy .hex file to Pi5.")
        if VERBOSE:
            print(e.stderr if e.stderr else str(e))
            print("Check that the Pi5 is reachable and scp / SSH keys are set up.")
        sys.exit(1)
    except FileNotFoundError:
        print("\nERROR: 'scp' command not found. Is OpenSSH installed?")
        sys.exit(1)

    print(f"\nPost-build completed at {datetime.now().strftime('%H:%M:%S')}")
    print("Ready for next build.\n")

    input("Press Enter to close this window...")


if __name__ == "__main__":
    main()