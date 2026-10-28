#!/usr/bin/env python3
"""
WallE_5.py – Supervisory program for the vision-enhanced 4-wheel robot

Features:
  - Sends one 'C' after opening serial so the Teensy is in the 'command' menu
  - Stays MANUAL until the operator types 'auto'
  - Starts camera pipeline (background)
  - Opens serial link to Teensy and relays telemetry
  - Background WiFi OTA watcher for latest.hex
  - Simple interactive command interface
  - Use python3 -m pyflakes software/scripts/WallE_5.py for static analysis

Location (recommended):
  /home/pi/my_vision_robot/software/scripts/WallE_5.py
"""

import serial
import threading
import time
import subprocess
import sys
from pathlib import Path
from datetime import datetime
import select
import rclpy
from rclpy.node import Node
from my_vision_robot_msgs.msg import ClearestDirection
import queue, threading
import numpy as np

teensy_events = queue.Queue()
heading_events = queue.Queue()

#added 10/24/26 for '!STUCK_AHEAD' implementation
stuck_ahead_event = threading.Event() #set in telemetry_loop() when '!STUCK_AHEAD' is detected in telemetry
scan_queue = queue.Queue()          # holds the finished list of rows
scan_ready = threading.Event()

#09/29/26 Added to store latest IMU heading from telemetry
imu_lock = threading.Lock()
latest_imu_hdg = None
last_imu_hdg_t_msec = 0.0

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

# Telemetry log file
TELEMETRY_LOG = Path.home() / "my_vision_robot/logs/telemetry.log"
TELEMETRY_LOG_MAX_DAYS = 3 #open this back up to 30 days if you want to keep more history, but it will take up more space
SECONDS_PER_DAY = 86400 #24 * 60 * 60

# Navigation (clearest-direction)
NAV_TOPIC = "/clearest_direction"
NAV_COMMAND_INTERVAL_SEC = 0.8
NAV_DEADBAND_DEG = 15.0   # was 6.0; includes 0° and ±10° (12:00, 11:00, 1:00)
NAV_MAX_SPEED_INCREMENT = 1.0 #added 09/03/26
NAV_DEFAULT_TURN_RATE_DEGPERSEC = 45.0 #must match teensy DEFAULT_TURN_RATE_DEGPERSEC
NAV_HANDSHAKE_PREFIXES = ("L", "R", "0", "1", "T", "5" )  # add others if firmware blocks on them
NAV_SEQ_CONTINUE_ON_FW_ABORT = True
NAV_TURN_TIMEOUT_MARGIN_SEC = 4.0 #for debugging with Teensy on blocks
NAV_LOG = Path.home() / "my_vision_robot/logs/nav.log"
NAV_LOG_MAX_DAYS = 3 * SECONDS_PER_DAY #open this back up to 30 days if you want to keep more history, but it will take up more space

# Startup / auto-nav (default = drive; C = manual override)
AUTO_NAV_ON_START = False
AUTO_NAV_MIN_WAIT_SEC = 5.0
AUTO_NAV_REQUIRE_VISION = True
AUTO_NAV_VISION_TIMEOUT_SEC = 30.0

#09/09/26 Added 'No Data' & 'Too Close' detection window values
NO_DATA_DETECTION_WINDOW = 3  # frames
TOO_CLOSE_DETECTION_WINDOW = 3  # frames
MIN_BACKUP_CLEAR_DIST_CM    = 20  # cm
MIN_RUN_TO_DAYLIGHT_DIST_CM = 100 # cm

#logging levels
LOG_NONE = 0
LOG_INFO = 1
LOG_VERBOSE = 2
LOG_LEVEL = LOG_VERBOSE   # <-- the one-line change

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

#09/09/26 Added 'No Data' & 'Too Close' detection window globals
last_no_data_count = 0
last_too_close_count = 0
log_level = LOG_INFO

#09/13/26 Added to prevent nav command lockout after firmware OTA update
teensy_needs_command_mode = False
last_repeat_cmd = None   # added to allow repeated 4/6 commands to accumulate heading in a corridor

#latest_clearest_age = -1.0
latest_clearest_rx = 0.0
clearest_count = 0

#10/24/26 part of '!STUCK_AHEAD' implementation
global bestWidth, bestStart, bestEnd, numruns, runStart


# ----------------------------------------------------------------------
# Utility
# ----------------------------------------------------------------------
def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S")

def log(msg: str, level: int = LOG_INFO) -> None:
    if level > LOG_LEVEL: #higher number = more verbose
        return
    line = f"[{timestamp()}] {msg}"
    print(line, flush=True)
    try:
        NAV_LOG.parent.mkdir(parents=True, exist_ok=True)
        with NAV_LOG.open("a") as f:
            f.write(line + "\n")
    except Exception:
        pass 
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
        if LOG_LEVEL >= LOG_VERBOSE:
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
    
    scan_rows = []
    collecting = False

    global telemetry_running, ser, latest_imu_hdg

    TELEMETRY_LOG.parent.mkdir(parents=True, exist_ok=True)

    with open(TELEMETRY_LOG, "a", buffering=1) as logf:
        while True:
            if telemetry_running and ser and ser.is_open and ser.in_waiting:
                try:
                    line = ser.readline().decode(errors="ignore").rstrip()
                    if line:
                        logf.write(line + "\n")
                        logf.flush()

                        #pass any nav_seq control lines to the telemetry queue for wait_for_done()

                        body = strip_teensy_ts(line) #added to avoid teensy timestamp in front of DONE/ACK/BUSY lines
                        if body.startswith(("M ")):
                           nav_enabled= False          

                        if body.startswith(("DONE ", "ACK ", "BUSY ")):
                            teensy_events.put(body)          # queue "DONE H", not "1016295 DONE H"

                        #10/24/26 Added to notify main loop of '!STUCK_AHEAD' detection
                        if "!Stuck_Ahead" in body:
                            drain_scan_queue()                  # clear any old scan data
                            stuck_ahead_event.set()          # threading.Event created at module level
                            # optional: also log it
                            log("Teensy reports STUCK_AHEAD")

                        if "IMU Heading" in body:
                            try:
                                hdg = float(body.split("=")[1].strip().split()[0])
                                t_msec = int(line.strip().split(":", 1)[0].split()[0])
                                set_imu_hdg(hdg, t_msec)
                            except (ValueError, IndexError):
                                log(f"bad IMU line: {line!r}")

                        if body.startswith("!TEENSY_TIME"):
                            try:
                                teensy_time = int(body.split("=")[1].strip().split()[0])
                                log(f"Teensy time: {teensy_time} msec")
                            except (ValueError, IndexError):
                                pass

                        #10/24/26 Added to collect scan data for '!STUCK_AHEAD' detection
                        if "!SCAN_START" in body:
                            drain_scan_queue()                  # clear any old scan data
                            scan_rows = []
                            collecting = True
                        elif collecting and "!SCAN_END" in body:
                            collecting = False
                            scan_queue.put(scan_rows)       # hand the whole table over
                            scan_ready.set()
                        elif collecting:
                            # parse a data line into (msec, dist, hdg) and append
                            parts = body.split()
                            if len(parts) >= 3:
                                try:
                                    scan_rows.append((int(parts[0]), float(parts[1]), float(parts[2])))
                                except ValueError:
                                    pass                       
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
    global ser, teensy_needs_command_mode

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
        teensy_needs_command_mode = True #added 09/13/26 to prevent nav command lockout after firmware OTA update
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
    global ser, telemetry_running, nav_enabled, last_nav_cmd_sent

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

                    # Stop nav FIRST so apply_nav_from_latest() cannot write 4/6/8
                    nav_enabled = False
                    last_nav_cmd_sent = None
                    telemetry_running = False

                    # Let an in-flight send_raw / apply_nav finish
                    time.sleep(0.5)

                    if ser and ser.is_open:
                        try:
                            ser.write(b"5\n")
                            ser.flush()
                            log("TX → 5  (OTA: stop motors / zero speed latch)")
                        except Exception as e:
                            log(f"OTA: stop command failed: {e}")
                        time.sleep(0.2)
                        ser.close()
                        log("Serial closed for OTA")

                    success = perform_ota_upload(HEX_FILE_PATH)

                    if success and ser and ser.is_open:
                        telemetry_running = True
                        log("Telemetry resumed on OTA port (no close/reopen)")
                        log("OTA cycle complete – Teensy should be running new firmware")
                    else:
                        time.sleep(1.0)
                        if open_serial(max_attempts=6, flush=False):
                            telemetry_running = True
                            log("Serial re-opened and telemetry restarted")
                        else:
                            log("WARNING: Could not re-open serial after OTA")
                        if success:
                            log("OTA cycle complete – Teensy should be running new firmware")

                    # Teensy rebooted out of command mode. Re-run the same
                    # startup checks (C + first 8). Do not set nav_enabled
                    # True here — try_auto_nav() does that after checks.
                    if success:
                        try:
                            try_auto_nav()
                        except Exception as e:
                            log(f"OTA: try_auto_nav failed: {e}")
        except Exception as e:
            log(f"OTA watcher error: {e}")

        time.sleep(OTA_POLL_INTERVAL)
        
# ----------------------------------------------------------------------
# Interactive help
# ----------------------------------------------------------------------
def print_help():
    helptext = """
********************************************************
****************** COMMAND KEYS ************************
********************************************************
0 = 180 deg CCW Turn
1 = 180 deg CW Turn
A = Abort - Reboots Processor
/ = Forward
.(dot) = Reverse
* = Exit ChkForUserInput()
 Faster
	8
Left 4	5 6 Right
	2
 Slower
Lxx.x or Rxx.x = SpinTurn xx.x degrees
Lxx.x,yy or Rxx.x,yy = SpinTurn with custom rate
Tddd.d = Turn to heading ddd.d

help/C Show this help
quit / exit  Exit WallE_5
"""

    print(helptext)
    log(helptext, level=LOG_VERBOSE)

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
        global latest_clearest, latest_clearest_age, latest_clearest_rx
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latest_clearest_age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        latest_clearest = msg
        latest_clearest_rx = time.monotonic()

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
    D-steps are WallE delays — never sent to the Teensy.
    """
    global last_nav_cmd_time, last_nav_cmd_sent, last_speed_increment
    global last_no_data_count, last_too_close_count

    if not nav_enabled or latest_clearest is None:
        return
    if ser is None or not ser.is_open:
        return

    now = time.time()
    if (now - last_nav_cmd_time) < NAV_COMMAND_INTERVAL_SEC:
        return
    last_nav_cmd_time = now

    msg = latest_clearest
    turn_deg = float(msg.turn_deg)
    depth = float(msg.best_depth_mm)
    status = int(msg.status)
    in_deadband = abs(turn_deg) < NAV_DEADBAND_DEG
    log(f"depth={depth:.0f} mm")


    match status:
        case 0:  # OK
            last_no_data_count = 0
            last_too_close_count = 0

            if in_deadband: #send_nav_cmd now handles speed incrementing and max speed increment
                send_nav_cmd(
                    "8",
                    f"deadband speed-up (turn_deg={turn_deg:.1f}, depth={depth:.0f} mm)",
                    force=True,
                )

            elif turn_deg < 0:
                send_nav_cmd(
                    "4",
                    f"left (turn_deg={turn_deg:.1f})",
                    force=True,
                )
            else:
                 send_nav_cmd(
                    "6",
                    f"right (turn_deg={turn_deg:.1f})",
                    force=True,
                )

        case 2:  # TOO_CLOSE
            #last_too_close_count += 1
            last_too_close_count = TOO_CLOSE_DETECTION_WINDOW + 1
            if last_too_close_count < TOO_CLOSE_DETECTION_WINDOW:
                return
            last_too_close_count = 0
            send_nav_cmd(
                "5",
                f"TOO_CLOSE stop, depth={depth:.0f} mm",
                force=True
            )           
            last_speed_increment = 0

            if depth < (MIN_BACKUP_CLEAR_DIST_CM * 10.0):
                
                log(f"TOO_CLOSE: depth={depth:.0f} mm < {MIN_BACKUP_CLEAR_DIST_CM*10:.0f} mm")

                # send_nav_seq(
                #     [
                #         (".", "reverse"),
                #         ("8", "speed up"),
                #         ("D1", "delay 1s"),
                #         ("5", "stop"),
                #     ],
                #     f"TOO_CLOSE backup, depth={depth:.0f} mm",
                # )
            # find_clear_direction()
            send_nav_cmd("8", "resume after TOO_CLOSE", force=True)

        # case 2:  # NO_VALID_DATA
        #     last_no_data_count += 1
        #     if last_no_data_count < NO_DATA_DETECTION_WINDOW:
        #         return
        #     last_no_data_count = 0
        #     send_nav_seq(
        #         [
        #             ("5", "stop"),
        #             ("D1", "delay 1s"),
        #             ("L45", "turn"),
        #             ("R90", "turn"),
        #             ("L45", "turn"),
        #             ("D1", "delay 1s"),
        #             ("8", "run"),
        #         ],
        #         f"NO_DATA, depth={depth:.0f} mm",
        #     )
        #     last_speed_increment = 0

        case _:
            return

def enter_teensy_command_menu():
    """Once per serial session: outer CFUI -> inner command menu."""
    # send_raw("C")
    # time.sleep(0.2)
    # log("TX command menu – sent C")
    log("Bypassing 'enter_teensy_command_menu()")


def enter_manual_mode(reason: str = "manual override"):
    """WallE MANUAL: stop AUTO and send inner-menu 5."""
    global nav_enabled, last_nav_cmd_sent, last_speed_increment
    nav_enabled = False
    last_nav_cmd_sent = None
    last_speed_increment = 0
    send_raw("5")
    log(f"MANUAL mode – {reason}")



def enter_auto_mode(reason: str = "auto"):
    """WallE AUTO only. Teensy must already be in the command menu."""
    global nav_enabled, last_nav_cmd_sent, last_speed_increment
    nav_enabled = True
    last_nav_cmd_sent = None
    last_speed_increment = 1 #
    send_raw("/") #make sure robot is in forward mode when entering AUTO mode
    send_raw("8") #make sure robot is moving forward when entering AUTO mode
    log(f"AUTO NAV – {reason}")


def try_auto_nav():
    if not AUTO_NAV_ON_START:
        log("AUTO_NAV_ON_START is False – staying manual")
        print("> ", end="", flush=True)
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

    cutoff = time.time() - (TELEMETRY_LOG_MAX_DAYS * SECONDS_PER_DAY)
    for p in log_dir.glob("*_telemetry.log"):
        if p.name == TELEMETRY_LOG.name:
            continue
        try:
            if p.stat().st_mtime < cutoff:
                p.unlink()
                print(f"Deleted old telemetry archive {p.name}")
        except OSError as e:
            print(f"Could not delete {p.name}: {e}")

def rotate_nav_logs():
    """Archive nav.log to YYMMDD_HHMM_nav.log; prune old archives."""
    log_dir = NAV_LOG.parent
    log_dir.mkdir(parents=True, exist_ok=True)

    if NAV_LOG.is_file() and NAV_LOG.stat().st_size > 0:
        stamp = datetime.now().strftime("%y%m%d_%H%M")
        archive = log_dir / f"{stamp}_nav.log"
        # collision in the same minute
        n = 1
        while archive.exists():
            archive = log_dir / f"{stamp}_{n}_nav.log"
            n += 1
        NAV_LOG.rename(archive)
        print(f"Archived previous nav log to {archive.name}")

    NAV_LOG.write_text("")  # new session file

    cutoff = time.time() - (NAV_LOG_MAX_DAYS * SECONDS_PER_DAY)
    for p in log_dir.glob("*_nav.log"):
        if p.name == NAV_LOG.name:
            continue
        try:
            if p.stat().st_mtime < cutoff:
                p.unlink()
                print(f"Deleted old nav archive {p.name}")
        except OSError as e:
            print(f"Could not delete {p.name}: {e}")

def drain_teensy_events():
    while True:
        try:
            teensy_events.get_nowait()
        except queue.Empty:
            break

def turn_timeout(cmd: str) -> float:
    """Worst-case wait for DONE. Safety net only; DONE is the real sync."""
    token = cmd.strip()
    if not token:
        return 2.0 + NAV_TURN_TIMEOUT_MARGIN_SEC
    op = token[0].upper()
    body = token[1:]
    rate = NAV_DEFAULT_TURN_RATE_DEGPERSEC
    deg = 0.0
    try:
        if op in ("L", "R"):
            if "," in body:
                deg_s, rate_s = body.split(",", 1)
                deg = float(deg_s)
                rate = float(rate_s)
            else:
                deg = float(body or 0)
        elif op == "T":
            deg = 180.0  # unknown heading error; cap generously
        elif op in ("0", "1"):
            deg = 180.0
        else:
            return 2.0 + NAV_TURN_TIMEOUT_MARGIN_SEC
    except ValueError:
        return 3.0 + NAV_TURN_TIMEOUT_MARGIN_SEC
    return abs(deg) / max(rate, 1.0) + NAV_TURN_TIMEOUT_MARGIN_SEC

def wait_for_done(cmd: str, timeout: float):
    """
    Block until Teensy DONE for this cmd, operator abort, or timeout.
    Returns the DONE/ABORT line, or None on timeout.

    Match the command letter after "DONE ", not the full token.
    Firmware prints "DONE T" for "T79.7", "DONE H" for "H",
    "DONE L45.00 deg..." for "L45".
    """
    deadline = time.time() + timeout
    trigger = cmd.strip().split(",")[0]  # "T79.7" from "T79.7,30"
    kind = trigger[:1].upper()           # "T", "L", "R", "H", "5"
    while time.time() < deadline:
        if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
            user = sys.stdin.readline().strip().lower()
            if user in ("5", "stop"):
                send_raw("5")
                log("Operator STOP during wait_for_done")
                return "ABORT STOP"
            if user in ("m", "cmd", "manual"):
                send_raw("M")
                enter_manual_mode("operator M during wait_for_done")
                return "ABORT MANUAL"
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        try:
            line = teensy_events.get(timeout=min(0.1, remaining))
        except queue.Empty:
            continue
        body = line.strip()
        if body.startswith("DONE ABORT"):
            return body
        if body.startswith("DONE ") and kind and body[5:].startswith(kind):
            return body
    return None

def send_nav_cmd(cmd: str, reason: str, *, force: bool = False, wait: bool = False) -> bool:
    """
    Send one nav item.
    D* is a local WallE delay (not sent to the Teensy).
    wait=True: block until Teensy DONE/ABORT or timeout.
    wait=False (default): fire-and-forget (vision nudges 4/6/8/5).
    Returns False if a waited motion timed out or was aborted.
    09/26/26: Added last_speed_increment logic to prevent '8' being sent too often
    """
    global last_nav_cmd_sent, last_nav_cmd_time, last_speed_increment
    print(f"send_nav_cmd: cmd={cmd}, reason={reason}, force={force}, wait={wait}")


    cmd = (cmd or "").strip()
    if not cmd:
        return True

    if not force and cmd == last_nav_cmd_sent and cmd != "5":
        return True

    # WallE-only delay — do not TX to Teensy
    if cmd[0] in ("D", "d"):
        try:
            sec = float(cmd[1:] or 1.0)
        except ValueError:
            sec = 1.0
        if LOG_LEVEL >= LOG_INFO:
            log(f"NAV → {cmd}  ({reason})")
        time.sleep(max(0.0, sec))
        last_nav_cmd_time = time.time()
        last_nav_cmd_sent = cmd
        return True

    if ser is None or not ser.is_open:
        log(f"NAV skip {cmd} – serial closed ({reason})")
        return False

    if wait:
        drain_teensy_events()

    #09/26/26 watch out for '8' (speed-up) being sent too often
    if cmd == "8":
        if last_speed_increment < NAV_MAX_SPEED_INCREMENT:
            last_speed_increment += 1
            print(f"last_speed_increment increased to {last_speed_increment}")
            send_raw(cmd)

    elif cmd in ("4", "6"):
        send_raw(cmd)
        log_heading_after_tweak(cmd, reason)
    else:
        send_raw(cmd)
    
    last_nav_cmd_time = time.time()
    last_nav_cmd_sent = cmd
    if LOG_LEVEL >= LOG_INFO:
        log(f"NAV → {cmd}  ({reason})")

    if not wait:
        return True

    done = wait_for_done(cmd, turn_timeout(cmd))
    if done is None:
        log(f"TIMEOUT waiting DONE for {cmd} ({reason})")
        send_raw("5")
        return False
    if "ABORT" in done:
        log(f"ABORTED {cmd}: {done}")
        if "MANUAL" in done or "STOP" in done:
            return False
        return NAV_SEQ_CONTINUE_ON_FW_ABORT    
    if LOG_LEVEL >= LOG_INFO:
        log(f"DONE ← {done}")
    return True

def send_nav_seq(steps, reason):
    """Run a recovery sequence. Each non-D step waits for Teensy DONE."""
    global nav_enabled

    log(f"NAV SEQUENCE: {reason}")
    for cmd, step_reason in steps:
        token = (cmd or "").strip()
        wait = bool(token) and token[0].upper() in NAV_HANDSHAKE_PREFIXES
        if not send_nav_cmd(cmd, step_reason, force=True, wait=wait):
            log(f"SEQUENCE aborted at {cmd}")
            nav_enabled = False
            break

# def find_clear_direction():
#     """Teensy performs the 360° LIDAR scan; Pi selects the heading."""
#     global nav_enabled
#     log("Stuck-ahead recovery: scanning for best heading...")
#     # optional short backup sequence here if you still want it
#     send_nav_cmd("!STUCK_SCAN", "start Teensy scan", force=True, wait=False)
#     if not scan_ready.wait(timeout=45):
#         log("scan timed out")
#         return
#     scan_ready.clear()
#     rows = scan_queue.get()
#     best_hdg = get_stuck_ahead_recovery_heading(rows)   # the polar_scan logic
#     send_nav_cmd(f"T{best_hdg:.1f}", "turn to recovery heading", force=True, wait=True)
#     send_nav_cmd("8", "resume forward", force=True, wait=False)
           
#added 09/29/26 to manage teensy heading telemetry and avoid race conditions with latest_imu_hdg
def set_imu_hdg(hdg, t_msec):
    global latest_imu_hdg, last_imu_hdg_t_msec
    with imu_lock:
        latest_imu_hdg = hdg
        last_imu_hdg_t_msec = t_msec

def get_current_imu_hdg(timeout=0.5):
    global latest_imu_hdg, last_imu_hdg_t_msec

    with imu_lock:
        prev_ms = last_imu_hdg_t_msec

    send_nav_cmd("H", "query heading", force=True, wait=True)  # ACK/DONE H

    deadline = time.time() + timeout
    while time.time() < deadline:
        with imu_lock:
            hdg, ms = latest_imu_hdg, last_imu_hdg_t_msec
        if ms is not None and (prev_ms is None or ms > prev_ms):
            return hdg
        time.sleep(0.02)

    log("get_current_imu_hdg: no newer Teensy IMU timestamp")
    return None

def strip_teensy_ts(line: str) -> str:
    s = line.strip()
    if s[:1].isdigit():
        i = 0
        while i < len(s) and s[i].isdigit():
            i += 1
        if i < len(s):
            sep = s[i]
            rest = s[i+1:].lstrip()
            # colon timestamp, or space timestamp followed by a word
            # (DONE / ACK / BUSY / IMU ...). Do not strip tab-separated
            # numeric scan rows.
            if sep == ':' or (sep == ' ' and rest[:1].isalpha()):
                s = rest
    return s
    
def log_heading_after_tweak(cmd: str, reason: str):
    """4 and 6 do not wait. Pause, then ask Teensy for the heading they stopped on."""
    time.sleep(1)
    hdg = get_current_imu_hdg()
    log(f"tweak {cmd} ({reason}): IMU heading {hdg}")
   
def get_stuck_ahead_recovery_heading(rows) -> float:
    """rows = list of (msec, dist, hdg) from the Teensy scan."""
    if not rows:
        log("no scan data")
        return get_current_imu_hdg() or 0.0

    THRESHOLD = 200.0
    hdg  = np.array([r[2] for r in rows])
    dist = np.array([r[1] for r in rows])
    N = len(hdg)

    bestStart = bestEnd = -1
    bestWidth = 0.0
    numruns = 0
    runStart = -1

    def terminate_run(i):
        nonlocal bestWidth, bestStart, bestEnd, numruns, runStart
        width = hdg[i-1] - hdg[runStart]
        if width > 180:  width -= 360
        if width < -180: width += 360
        width = abs(width)
        if width > bestWidth:
            bestWidth = width
            bestStart = runStart
            bestEnd   = i - 1
        runStart = -1
        numruns += 1

    for i in range(N):
        if dist[i] >= THRESHOLD:
            if runStart < 0:
                runStart = i
            if i >= N - 1:
                terminate_run(i + 1)
        elif runStart >= 0:
            terminate_run(i)

    # wrap-around stitch (same as before)
    if N > 1 and dist[0] >= THRESHOLD and dist[N-1] >= THRESHOLD:
        endRun = N - 1
        while endRun > 0 and dist[endRun] >= THRESHOLD:
            endRun -= 1
        endRun += 1
        startRun = 0
        while startRun < N - 1 and dist[startRun] >= THRESHOLD:
            startRun += 1
        startRun -= 1
        width = hdg[startRun] - hdg[endRun]
        if width > 180:  width -= 360
        if width < -180: width += 360
        width = abs(width)
        if width > bestWidth:
            bestWidth = width
            bestStart = endRun
            bestEnd   = startRun
            numruns -= 1

    if bestStart < 0:                       # no far run found
        return float(hdg[np.argmax(dist)])

    if bestStart <= bestEnd:
        win_idx = list(range(bestStart, bestEnd + 1))
    else:
        win_idx = list(range(bestStart, N)) + list(range(0, bestEnd + 1))

    bestHdg_idx = win_idx[len(win_idx) // 2]
    return float(hdg[bestHdg_idx])


def drain_scan_queue():
    while True:
        try:
            scan_queue.get_nowait()
        except queue.Empty:
            break

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    global teensy_needs_command_mode, telemetry_running, nav_enabled, last_nav_cmd_sent

    print("\n========================================")
    print("       WallE_5 Supervisor Starting")
    print("========================================\n")

    print(f"Telemetry log: {TELEMETRY_LOG}")
    rotate_telemetry_logs()

    NAV_LOG.parent.mkdir(parents=True, exist_ok=True)
    print(f"Supervisor log: {NAV_LOG}")
    rotate_nav_logs()
    print("In another terminal:  tail -f ~/my_vision_robot/logs/nav.log\n")

    # 1. Serial
    if not open_serial():
        log("Could not open serial port. Exiting.")
        sys.exit(1)
    enter_teensy_command_menu()
    enter_manual_mode("startup")

    # 2. Camera (disabled – start externally)
    # start_camera()

    # 3. Telemetry thread
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()
    log("Telemetry relay started")

    # 4. ROS subscriber
    try:
        start_ros_subscriber()
    except Exception as e:
        log(f"ROS subscriber failed (nav will not work): {e}")
        log("Did you run: source ~/my_vision_robot/software/ros2/install/setup.bash ?")

    # 5. OTA watcher thread
    ota_thread = threading.Thread(target=ota_watcher_loop, daemon=True)
    ota_thread.start()

    # 6. Stay MANUAL until operator types 'auto'
    has_tty = sys.stdin.isatty()
    if has_tty:
        input("Press Enter for the command prompt (motors stay stopped until 'auto')...")
        log("Ready – MANUAL (type auto to navigate, M = manual, M or 5 to stop)")
        print("Type 'help' for commands.  M = manual, auto = resume nav.\n")
        print("> ", end="", flush=True)
    else:
        log("No TTY – keyboard unavailable; staying MANUAL (AUTO_NAV_ON_START=False)")
    threading.Thread(target=try_auto_nav, daemon=True).start()

    # 7. Interactive loop
    last_repeat_cmd = None

    while True:
        user = None
        try:
            if has_tty and select.select([sys.stdin], [], [], 0.1)[0]:
                user = sys.stdin.readline().strip()
            elif not has_tty:
                time.sleep(0.1)
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if teensy_needs_command_mode:
            teensy_needs_command_mode = False
            enter_teensy_command_menu()
            enter_manual_mode("post-OTA")
            if has_tty:
                print("> ", end="", flush=True)

        apply_nav_from_latest()

        # Stuck-ahead recovery (Teensy scan + Pi heading selection)
        if stuck_ahead_event.is_set():
            stuck_ahead_event.clear()
            if not nav_enabled:
                log("STUCK_AHEAD ignored (not in auto-nav)")
            else:
                nav_enabled = False
                log("Stuck ahead detected — starting recovery")

                send_nav_seq([
                    ("5", "stop"),
                    ("D0.5", "pause"),
                    (".", "reverse"),
                    ("8", "speed up"),
                    ("D1", "back up"),
                    ("5", "stop"),
                ], "pre-scan backup")

                send_nav_cmd("!STUCK_SCAN", "start Teensy scan", force=True, wait=False)
                if not scan_ready.wait(timeout=45):
                    log("scan timed out")
                    nav_enabled = True
                else:
                    scan_ready.clear()
                    rows = scan_queue.get()
                    best_hdg = get_stuck_ahead_recovery_heading(rows)
                    log(f"Recovery heading: {best_hdg:.1f}°")

                    send_nav_cmd(f"T{best_hdg:.1f}", "turn to recovery heading", force=True, wait=True)
                    send_nav_cmd("8", "resume forward", force=True, wait=False)
                    nav_enabled = True

        if user is None:
            continue

        if user == "":
            if last_repeat_cmd:
                user = last_repeat_cmd
                log(f"repeat → {user}")
            else:
                if has_tty:
                    print("> ", end="", flush=True)
                continue

        low = user.lower()

        if low in ("quit", "exit", "q"):
            break
        elif low in ("h", "heading request"):
            send_raw("h")
            last_repeat_cmd = None
        elif low in ("m", "cmd", "manual"):
            log(f"Received {user}: Entering MANUAL mode")
            enter_manual_mode("operator C / manual")
            last_repeat_cmd = None
            nav_enabled = False
        elif low in ("auto", "nav"):
            print("Resuming AUTO navigation...")
            enter_auto_mode("operator resume auto-nav")
            send_raw("!TEENSY_TIME")
            last_repeat_cmd = None
        elif low in ("5", "stop"):
            send_raw("5")
            last_repeat_cmd = None
        elif user and user[0].upper() in ("L", "R"):
            send_raw(user.upper())
            last_repeat_cmd = user.upper()
        else:
            send_raw(user)
            last_repeat_cmd = user if user in ("4", "6", "8", "2", ".", "/", "0", "1") else None

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
        try:
            send_raw("5")
            time.sleep(0.2)
        except Exception:
            pass
        ser.close()
    log("Done.")

if __name__ == "__main__":
    main()