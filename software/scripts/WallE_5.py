#!/usr/bin/env python3
"""
WallE_5.py – Supervisory program for the vision-enhanced 4-wheel robot

Features:
  - Does NOT automatically enter Teensy command mode
  - Starts camera pipeline (background)
  - Opens serial link to Teensy and relays telemetry
  - Background WiFi OTA watcher for latest.hex
  - Simple interactive command interface

Location (recommended):
  /home/pi/my_vision_robot/software/scripts/WallE_5.py
"""

import serial
import threading
import time
import subprocess
import sys
import os
from pathlib import Path
from datetime import datetime
import select
import rclpy
from rclpy.node import Node
from my_vision_robot_msgs.msg import ClearestDirection

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
SERIAL_PORT     = "/dev/ttyAMA0"
BAUD            = 115200
CAMERA_SCRIPT   = Path.home() / "my_vision_robot/software/scripts/restart_camera.sh"

# OTA configuration
HEX_FILE_PATH   = Path.home() / "my_vision_robot/firmware/latest.hex"
OTA_POLL_INTERVAL = 1.0          # seconds
MIN_HEX_SIZE    = 50000          # bytes
MAX_AGE_SECONDS = 300            # 5 minutes

# Telemetry log file (for the second terminal)
TELEMETRY_LOG = Path.home() / "my_vision_robot/logs/telemetry.log"
TELEMETRY_LOG_MAX_DAYS = 30

# Navigation (clearest-direction)
NAV_TOPIC = "/clearest_direction"
NAV_COMMAND_INTERVAL_SEC = 0.8
NAV_DEADBAND_DEG = 15.0   # was 6.0; includes 0° and ±10° (12:00, 11:00, 1:00)
NAV_MAX_SPEED_INCREMENT = 1.0 #added 09/03/26


# Startup / auto-nav (default = drive; C = manual override)
AUTO_NAV_ON_START = True
AUTO_NAV_MIN_WAIT_SEC = 5.0
AUTO_NAV_REQUIRE_VISION = True
AUTO_NAV_VISION_TIMEOUT_SEC = 30.0

# ----------------------------------------------------------------------
# Globals
# ----------------------------------------------------------------------
ser = None
telemetry_running = True
nav_enabled = False
nav_node = None
last_nav_cmd_time = 0.0
last_nav_cmd_sent = None
latest_clearest = None
last_speed_increment = 0

# ----------------------------------------------------------------------
# Utility
# ----------------------------------------------------------------------
def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S")

def log(msg: str):
    print(f"[{timestamp()}] {msg}")

# ----------------------------------------------------------------------
# Serial helpers
# ----------------------------------------------------------------------
def open_serial(max_attempts: int = 10, flush: bool = True) -> bool:
    """Try to open the serial port. Returns True on success."""
    global ser
    for attempt in range(1, max_attempts + 1):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            time.sleep(0.4)
            if flush:
                ser.reset_input_buffer()
            log(f"Serial opened on {SERIAL_PORT}")
            return True
        except Exception as e:
            log(f"Serial open attempt {attempt}/{max_attempts} failed: {e}")
            time.sleep(2)
    return False

def send_raw(cmd: str):
    """Send a raw string + newline to the Teensy."""
    if ser and ser.is_open:
        ser.write((cmd.strip() + "\n").encode())
        log(f"TX → {cmd.strip()}")

def send_turn(direction: str, degrees: float, rate: float = None):
    """
    Helper for navigation code.
    direction : 'L' or 'R'
    degrees   : positive float
    rate      : optional deg/sec (Teensy uses default if omitted)
    """
    direction = direction.upper()
    if direction not in ("L", "R"):
        print("direction must be 'L' or 'R'")
        return

    if rate is None:
        cmd = f"{direction}{degrees:.2f}"
    else:
        cmd = f"{direction}{degrees:.2f},{rate:.1f}"
    send_raw(cmd)

# ----------------------------------------------------------------------
# Telemetry relay
# ----------------------------------------------------------------------
def telemetry_loop():
    """Read lines from the Teensy and write them cleanly to the telemetry log."""
    global telemetry_running, ser

    TELEMETRY_LOG.parent.mkdir(parents=True, exist_ok=True)

    with open(TELEMETRY_LOG, "a", buffering=1) as logf:
        while True:
            if telemetry_running and ser and ser.is_open and ser.in_waiting:
                try:
                    line = ser.readline().decode(errors="ignore").rstrip()
                    if line:
                        logf.write(line + "\n")
                except Exception as e:
                    print(f"[telemetry] {e}")
            time.sleep(0.02)# ----------------------------------------------------------------------
# Camera
# ----------------------------------------------------------------------
def start_camera():
    log("Starting camera pipeline...")
    try:
        subprocess.Popen(
            ["bash", str(CAMERA_SCRIPT)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True
        )
        time.sleep(2)
        log("Camera start requested (running in background)")
    except Exception as e:
        log(f"Camera start failed: {e}")

# ----------------------------------------------------------------------
# OTA support
# ----------------------------------------------------------------------
def is_fresh_hex_file(path: Path) -> bool:
    if not path.exists():
        return False
    try:
        size = path.stat().st_size
        if size < MIN_HEX_SIZE:
            return False
        age = time.time() - path.stat().st_mtime
        if age > MAX_AGE_SECONDS:
            return False
        return True
    except Exception:
        return False

def _tee_teensy_line(line: str) -> str:
    """WallE stdout (top pane) + telemetry.log (bottom pane)."""
    line = (line or "").strip("\r\n")
    if not line:
        return ""
    print(f"[Teensy] {line}")
    try:
        with TELEMETRY_LOG.open("a") as f:
            f.write(line + "\n")
            f.flush()
    except OSError:
        pass
    return line


def perform_ota_upload(hex_path: Path) -> bool:
    """
    OTA transfer. Caller must have already closed the main serial port.
    On success, leaves global ser OPEN (same port used for the upload).
    On failure, closes the OTA port.
    """
    global ser

    log("=== OTA Upload Started ===")
    log(f"File: {hex_path}")

    ota_ser = None
    success = False
    try:
        ota_ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        ser = ota_ser
    except Exception as e:
        log(f"Cannot open serial for OTA: {e}")
        return False

    try:
        log("Sending 'U' trigger...")
        ota_ser.write(b"U")
        time.sleep(1.0)

        log("Waiting for Teensy prompt...")
        start = time.time()
        got_prompt = False
        while time.time() - start < 10:
            if ota_ser.in_waiting:
                line = ota_ser.readline().decode("utf-8", errors="replace").strip()
                _tee_teensy_line(line)
                if "reading hex lines" in line.lower():
                    log("Got prompt")
                    got_prompt = True
                    break
            time.sleep(0.2)

        if not got_prompt:
            log("Timed out waiting for prompt")
            return False

        log("Sending .hex file...")
        line_count = 0
        with open(hex_path, "r") as f:
            for line_count, line in enumerate(f, 1):
                ota_ser.write(line.encode("utf-8"))
                if line_count % 200 == 0:
                    time.sleep(0.005)

        log(f"Sent {line_count} lines. Sending EOF...")
        ota_ser.write(b":00000001FF\r\n")
        ota_ser.flush()
        time.sleep(1.0)

        log("Waiting for line count prompt or boot...")
        start = time.time()
        got_confirm = False
        saw_boot = False
        while time.time() - start < 20:
            if ota_ser.in_waiting:
                line = ota_ser.readline().decode("utf-8", errors="replace").strip()
                _tee_teensy_line(line)
                low = line.lower()

                if "enter" in low and "flash" in low:
                    parts = line.split()
                    if len(parts) > 1 and parts[1].isdigit():
                        num = parts[1]
                        log(f"Sending line count: {num}")
                        ota_ser.write((num + "\r\n").encode("utf-8"))
                        ota_ser.flush()
                        got_confirm = True

                if "MPU6050 Ready" in line or line.startswith("Time"):
                    saw_boot = True
                    break
            time.sleep(0.2)

        if not got_confirm and not saw_boot:
            log("OTA failed: no flash confirm and no boot")
            return False

        log("OTA UPDATE SUCCESSFUL!")
        if saw_boot:
            log("Teensy already booted – draining boot telemetry...")
            drain_end = time.time() + 2.0
            while time.time() < drain_end:
                if ota_ser.in_waiting:
                    line = ota_ser.readline().decode("utf-8", errors="replace").strip()
                    _tee_teensy_line(line)
                else:
                    time.sleep(0.05)
        else:
            log("Waiting for Teensy reboot...")
            start = time.time()
            while time.time() - start < 12:
                if ota_ser.in_waiting:
                    line = ota_ser.readline().decode("utf-8", errors="replace").strip()
                    _tee_teensy_line(line)
                    if "MPU6050 Ready" in line or line.startswith("Time"):
                        saw_boot = True
                        break
                time.sleep(0.2)
            if not saw_boot:
                time.sleep(2)

        success = True
        return True

    except Exception as e:
        log(f"OTA exception: {e}")
        return False
    finally:
        if not success and ota_ser is not None:
            try:
                ota_ser.close()
            except Exception:
                pass

def ota_watcher_loop():
    """Background thread that watches for a new latest.hex and triggers OTA."""
    global ser, telemetry_running

    last_mtime = 0.0
    if HEX_FILE_PATH.exists():
        last_mtime = HEX_FILE_PATH.stat().st_mtime
        log(f"OTA watcher: initial latest.hex found "
            f"(size {HEX_FILE_PATH.stat().st_size} bytes)")

    log(f"OTA watcher started – monitoring {HEX_FILE_PATH}")

    while True:
        try:
            if is_fresh_hex_file(HEX_FILE_PATH):
                current_mtime = HEX_FILE_PATH.stat().st_mtime
                if current_mtime > last_mtime + 0.5:
                    log("=== New latest.hex detected! ===")
                    last_mtime = current_mtime

                    telemetry_running = False
                    time.sleep(0.3)
                    if ser and ser.is_open:
                        ser.close()
                        log("Serial closed for OTA")

                    success = perform_ota_upload(HEX_FILE_PATH)

                    # Success: perform_ota_upload left ser open — do NOT reopen or flush
                    if success and ser and ser.is_open:
                        telemetry_running = True
                        log("Telemetry resumed on OTA port (no close/reopen)")
                        log("OTA cycle complete – Teensy should be running new firmware\n")
                    else:
                        time.sleep(1.0)
                        if open_serial(max_attempts=6, flush=False):
                            telemetry_running = True
                            log("Serial re-opened and telemetry restarted")
                        else:
                            log("WARNING: Could not re-open serial after OTA")
                        if success:
                            log("OTA cycle complete – Teensy should be running new firmware\n")
        except Exception as e:
            log(f"OTA watcher error: {e}")

        time.sleep(OTA_POLL_INTERVAL)
# ----------------------------------------------------------------------
# Interactive help
# ----------------------------------------------------------------------
def print_help():
    print("""
Available commands:
  C / manual   Stop auto-nav → manual control (override)
  auto / nav   Resume autonomous clearest-direction mode
  5 / stop     Stop motors
  Lxx.xx       Turn left  xx.xx degrees
  Rxx.xx       Turn right xx.xx degrees
  help         Show this help
  quit / exit  Exit WallE_5
""")

# ----------------------------------------------------------------------
# ROS nav subscriber (clearest direction)
# ----------------------------------------------------------------------
class ClearestDirectionSubscriber(Node):
    def __init__(self):
        super().__init__("walle5_clearest_sub")
        self.subscription = self.create_subscription(
            ClearestDirection,
            NAV_TOPIC,
            self._on_clearest,
            10,
        )
        self.get_logger().info(f"Subscribed to {NAV_TOPIC}")

    def _on_clearest(self, msg: ClearestDirection):
        global latest_clearest
        latest_clearest = msg


def ros_spin_thread():
    global nav_node
    rclpy.spin(nav_node)


def start_ros_subscriber():
    """Init rclpy and start subscriber node in a background thread."""
    global nav_node
    if not rclpy.ok():
        rclpy.init(args=None)
    nav_node = ClearestDirectionSubscriber()
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    log(f"ROS subscriber started on {NAV_TOPIC}")


def apply_nav_from_latest():
    """
    Translate latest ClearestDirection into Teensy command-mode chars.
    Call periodically from main loop while nav_enabled.

    Speed-up ('8') only in deadband, and only once per stop.
    Repeated 4/6 are allowed so a corridor can accumulate heading.
    """
    global last_nav_cmd_time, last_nav_cmd_sent, last_speed_increment

    if not nav_enabled or latest_clearest is None:
        return
    if ser is None or not ser.is_open:
        return

    now = time.time()
    if (now - last_nav_cmd_time) < NAV_COMMAND_INTERVAL_SEC:
        return

    msg = latest_clearest
    turn_deg = float(msg.turn_deg)
    depth = float(msg.best_depth_mm)
    status = int(msg.status)
    in_deadband = abs(turn_deg) < NAV_DEADBAND_DEG

    log(
        f"NAV dbg turn={turn_deg:.1f} status={status} "
        f"dead={in_deadband} latch={last_speed_increment} depth={depth:.0f}"
    )

    cmd = None
    reason = ""

    # status: 0=OK, 1=TOO_CLOSE, 2=NO_VALID_DATA
    if status == 1 or status == 2:
        cmd = "5"
        last_speed_increment = 0
        reason = f"stop (status={status}, depth={depth:.0f} mm)"
    elif in_deadband and last_speed_increment == 0:
        cmd = "8"
        last_speed_increment = 1
        reason = (
            f"deadband speed-up (turn_deg={turn_deg:.1f}, depth={depth:.0f} mm)"
        )
    elif in_deadband:
        log(
            f"NAV hold (deadband turn={turn_deg:.1f}, "
            f"latch={last_speed_increment}, depth={depth:.0f})"
        )
        return
    elif turn_deg < 0:
        cmd = "4"  # 10° CCW
        reason = f"left (turn_deg={turn_deg:.1f})"
    else:
        cmd = "6"  # 10° CW
        reason = f"right (turn_deg={turn_deg:.1f})"

    if cmd is None:
        return

    send_raw(cmd)
    last_nav_cmd_time = now
    last_nav_cmd_sent = cmd
    log(f"NAV → {cmd}  ({reason})")    
def enter_manual_mode(reason: str = "manual override"):
    """Stop autonomous nav and halt motors. C / manual uses this."""
    global nav_enabled, last_nav_cmd_sent
    nav_enabled = False
    last_nav_cmd_sent = None
    send_raw("5")
    log(f"MANUAL mode – {reason}")


def enter_auto_mode(reason: str = "auto"):
    """Resume autonomous nav. Assumes Teensy already accepts 4/6/5 (command mode)."""
    global nav_enabled, last_nav_cmd_sent
    nav_enabled = True
    last_nav_cmd_sent = None
    log(f"AUTO NAV – {reason}")


def try_auto_nav():
    """
    After startup checks: wait for settle (+ optional vision), then enable AUTO.
    Runs in a background thread so the prompt/override loop stays alive.
    """
    if not AUTO_NAV_ON_START:
        log("AUTO_NAV_ON_START is False – staying manual")
        return

    log("Auto-nav: waiting for system to settle...")
    time.sleep(AUTO_NAV_MIN_WAIT_SEC)

    if AUTO_NAV_REQUIRE_VISION:
        log("Auto-nav: waiting for /clearest_direction...")
        t0 = time.time()
        while latest_clearest is None and (time.time() - t0) < AUTO_NAV_VISION_TIMEOUT_SEC:
            time.sleep(0.2)
        if latest_clearest is None:
            log("Auto-nav: no vision data – staying MANUAL")
            return

    # Enter Teensy command mode once, then enable auto commanding
    send_raw("C")
    time.sleep(0.3)
    enter_auto_mode("startup checks passed")
    
def rotate_telemetry_logs():
    """Archive telemetry.log to YYMMDD_HHMM_telemetry.log; prune old archives."""
    log_dir = TELEMETRY_LOG.parent
    log_dir.mkdir(parents=True, exist_ok=True)

    if TELEMETRY_LOG.is_file() and TELEMETRY_LOG.stat().st_size > 0:
        stamp = datetime.now().strftime("%y%m%d_%H%M")
        archive = log_dir / f"{stamp}_telemetry.log"
        # collision in the same minute
        n = 1
        while archive.exists():
            archive = log_dir / f"{stamp}_{n}_telemetry.log"
            n += 1
        TELEMETRY_LOG.rename(archive)
        print(f"Archived previous telemetry to {archive.name}")

    TELEMETRY_LOG.write_text("")  # new session file

    cutoff = time.time() - (TELEMETRY_LOG_MAX_DAYS * 86400)
    for p in log_dir.glob("*_telemetry.log"):
        if p.name == TELEMETRY_LOG.name:
            continue
        try:
            if p.stat().st_mtime < cutoff:
                p.unlink()
                print(f"Deleted old telemetry archive {p.name}")
        except OSError as e:
            print(f"Could not delete {p.name}: {e}")

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    global telemetry_running, nav_enabled, last_nav_cmd_sent

    print("\n========================================")
    print("       WallE_5 Supervisor Starting")
    print("========================================\n")

    print(f"Telemetry log: {TELEMETRY_LOG}")
    rotate_telemetry_logs()
    print("In a second terminal run:  tail -f ~/my_vision_robot/logs/telemetry.log\n")

    # 1. Serial
    if not open_serial():
        log("Could not open serial port. Exiting.")
        sys.exit(1)

    # 2. Camera
    #start_camera() #disabled to make sure camera is up and running before starting WallE_5.py

    # 3. Telemetry thread
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()
    log("Telemetry relay started")

    # 4. ROS subscriber (clearest direction)
    try:
        start_ros_subscriber()
    except Exception as e:
        log(f"ROS subscriber failed (nav will not work): {e}")
        log("Did you run: source ~/my_vision_robot/software/ros2/install/setup.bash ?")

    # 5. OTA watcher thread
    ota_thread = threading.Thread(target=ota_watcher_loop, daemon=True)
    ota_thread.start()

    # 6. Wait for Enter, then AUTO (temporary)
    has_tty = sys.stdin.isatty()
    if not has_tty:
        log("No TTY – not starting auto-nav (no way to confirm or override)")
    else:
        input("Press Enter to start navigating...")
        threading.Thread(target=try_auto_nav, daemon=True).start()
        log("Ready – auto-nav will start after checks (C = manual override)")

    print("Type 'help' for commands. C = manual, auto = resume nav.\n")
    if has_tty:
        print("> ", end="", flush=True)

    # 7. Interactive loop (CheckForManualOverride + apply auto-nav)
    while True:
        user = ""
        try:
            if has_tty and select.select([sys.stdin], [], [], 0.1)[0]:
                user = sys.stdin.readline().strip()
            elif not has_tty:
                time.sleep(0.1)
        except (EOFError, KeyboardInterrupt):
            print()
            break

        apply_nav_from_latest()

        if not user:
            continue

        low = user.lower()

        if low in ("quit", "exit", "q"):
            break
        elif low in ("help", "h", "?"):
            print_help()
        elif low in ("c", "cmd", "manual"):
            enter_manual_mode("operator C / manual")
        elif low in ("auto", "nav"):
            send_raw("C")
            time.sleep(0.2)
            enter_auto_mode("operator resume")
        elif low in ("5", "stop"):
            send_raw("5")
        elif user[0].upper() in ("L", "R"):
            send_raw(user.upper())
        else:
            send_raw(user)

        if has_tty:
            print("> ", end="", flush=True)

    # Cleanup
    log("Shutting down WallE_5...")
    nav_enabled = False
    telemetry_running = False
    time.sleep(0.3)
    try:
        if nav_node is not None:
            nav_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    if ser and ser.is_open:
        ser.close()
    log("Done.")
 
if __name__ == "__main__":
    main()