#!/usr/bin/env python3
"""
Post-Build OTA Helper for Visual Micro
"""

import sys
import os
import subprocess
from datetime import datetime

# ============== CONFIGURATION ==============
VERBOSE = False         # Set to True only when debugging
# ===========================================

def main():
    print("\n" + "="*60)
    print("Post-Build OTA Transfer to Pi5")
    print("="*60)

    if VERBOSE:
        print("\n=== Visual Micro Path Variables ===")
        print(f"build.project_path              = {sys.argv[3] if len(sys.argv) > 3 else 'Not provided'}")
        print(f"vm.runtime.build.intermediate_output_path = {sys.argv[4] if len(sys.argv) > 4 else 'Not provided'}")
        print(f"vm.runtime.build.final_output_path = {sys.argv[1] if len(sys.argv) > 1 else 'Not provided'}")
        print("="*40)

    if len(sys.argv) < 4:
        print("ERROR: Not enough arguments provided.")
        input("Press Enter to close...")
        sys.exit(1)

    intermediate_path = sys.argv[4].rstrip('\\')
    project_name = sys.argv[2] if len(sys.argv) > 2 else "Unknown"
    #hex_path = f"{intermediate_path}\\Wifi_OTA_Demo.hex"
    hex_path = f"{intermediate_path}\\Wifi_OTA.hex"

    if VERBOSE:
        print("\n=== Derived File Paths ===")
        print(f"Project Name     : {project_name}")
        print(f"Intermediate Dir : {intermediate_path}")
        print(f"Hex File Path    : {hex_path}")
        print("="*40)
        print(f"Time : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    if not os.path.exists(hex_path):
        print(f"ERROR: Hex file not found: {hex_path}")
        input("Press Enter to close...")
        sys.exit(1)

    print("\nCopying .hex to Pi5...")
    
    try:
        #subprocess.run(["scp", hex_path, "pi@RobotPi5:~/my_vision_robot/firmware/latest.hex"], 
        #               capture_output=True, text=True, check=True)
        subprocess.run(["scp", hex_path, "pi@RobotPi5:~/my_vision_robot/latest.hex"], 
                       capture_output=True, text=True, check=True)

        print("SUCCESS: .hex file copied to Pi5.")
    except subprocess.CalledProcessError:
        print("\nERROR: Failed to copy .hex file to Pi5.")
        if VERBOSE:
            print("Check that the Pi5 is reachable and scp is set up.")
        sys.exit(1)
    except FileNotFoundError:
        print("\nERROR: 'scp' command not found. Is OpenSSH installed?")
        sys.exit(1)

    print(f"\nPost-build completed at {datetime.now().strftime('%H:%M:%S')}")
    print("Ready for next build.\n")

    input("Press Enter to close this window...")

if __name__ == "__main__":
    main()