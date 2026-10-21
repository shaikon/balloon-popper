#!/usr/bin/env python3
# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Vision Balloon Chaser — entry point.

Usage:
    python main.py <lat> <lon> [--alt ALT] [--connect URL] [--platform {sim,real}]

Example:
    python main.py -35.3633 149.1652 --alt 100 --platform sim

The --platform flag picks the adapter set:
    sim  — GazeboCamera + GzGimbalDriver + HsvDetectionSensor
    real — V4l2NoseCamera + PhysicalGimbalDriver + CompanionDetectionSensor
"""

from __future__ import annotations
import argparse
import logging
import os
import sys

from Companion.csv_logger import StartupBufferHandler

# Logging setup
log = logging.getLogger("balloon_chaser")
log.setLevel(logging.DEBUG)
_log_format = logging.Formatter(
    "%(asctime)s [%(threadName)s] %(message)s", datefmt="%H:%M:%S")
_handler = logging.StreamHandler(sys.stdout)
_handler.setFormatter(_log_format)
log.addHandler(_handler)

# In-memory startup-log buffer: captures every log message emitted before the
# operator enables CSV logging (the 'q' key).  If logging is enabled, those
# lines are drained into the head of the CSV file so the unique startup data
# (FAST/POLLED telemetry path, parameter readbacks, configuration
# verifications) travels with the flight log.  If logging is never enabled
# the buffer is discarded on process exit.  Terminal output is unaffected -
# this handler is an additional sink, not a replacement.  Same formatter so
# the lines in the CSV head read identically to what scrolled by on screen.
_startup_buffer = StartupBufferHandler()
_startup_buffer.setFormatter(_log_format)
log.addHandler(_startup_buffer)

# The Companion.* module loggers are NOT children of "balloon_chaser", so every
# log.info() inside the package went to a logger with no handler and was
# silently discarded - the entire Lua deploy and scripting-setup diagnostic
# trail included.  The startup then looked like nothing had happened at all.
#
# Both sinks, deliberately: these lines ARE the "parameter readbacks,
# configuration verifications" the startup buffer exists to preserve, so they
# belong in the CSV head as well as on screen.
_pkg_log = logging.getLogger("Companion")
_pkg_log.setLevel(logging.INFO)
_pkg_log.addHandler(_handler)
_pkg_log.addHandler(_startup_buffer)

# Export destination follows the code: a "Downloaded Files" folder beside this
# file, recorded at launch from __file__ - so it is where main.py lives, not
# wherever the program happened to be started from.  The folder is not created
# at launch; the export buttons make it on the first Copy that has a file to
# write, so a run that never exports leaves no empty folder behind.
from Companion import file_export as _file_export
_file_export.set_destination(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "Downloaded Files"))

# Save the console output to a file as well, so the startup, deploy and fault
# messages that scroll off the terminal can be kept, exported and deleted from
# the Gain Tuner (the "Console log" row).  Same formatter as the terminal, so the
# file reads identically.  Best effort: a log file that will not open must not
# stop the program, which still runs with terminal output.
try:
    _console_handler, _console_log_path = _file_export.start_console_log()
    _console_handler.setFormatter(_log_format)
    log.addHandler(_console_handler)
    _pkg_log.addHandler(_console_handler)
    log.info("[LOG] console output is being saved to %s "
             "(export or delete it from the Gain Tuner)", _console_log_path)
except OSError as _console_err:
    log.warning("[LOG] could not open a console log file (%s); "
                "terminal output only", _console_err)


def main():
    parser = argparse.ArgumentParser(
        description="Vision Balloon Chaser - catapult takeoff, fly to waypoint, gimbal tracking")
    parser.add_argument("lat",  type=float, help="Waypoint latitude  (e.g. -35.3633)")
    parser.add_argument("lon",  type=float, help="Waypoint longitude (e.g. 149.1652)")
    parser.add_argument("--alt", type=float, default=100.0,
                        help="Cruise / loiter altitude in metres AGL (default: 100)")
    parser.add_argument("--connect", default="udp:127.0.0.1:14550",
                        help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    parser.add_argument("--platform", choices=['sim', 'real'], default='sim',
                        help="Adapter set: sim=Gazebo+HSV, real=MAVLink-gimbal+companion CC")
    parser.add_argument("--calibrate-gimbal", action="store_true",
                        help="Before launch, sweep the gimbal through a known "
                             "pan/tilt grid and write gimbal_calib.csv. Holds the "
                             "catapult until the sweep completes, then launches.")
    parser.add_argument("--arduplane-autotune", "--autotune", action="store_true",
                        help="Maintenance mode: launch, climb, run ArduPlane "
                             "AUTOTUNE (roll+pitch), save the tuned rate PIDs, then "
                             "exit. Tunes the rate loop UNDER the ACRO layer; runs no "
                             "balloon chaser/vision/gimbal. Back up params first if unsure.")
    parser.add_argument("--lua-scripts", metavar="DIR", default=None,
                        help="ArduPilot scripts directory, copied to "
                             "directly. When omitted: SITL's own scripts "
                             "directory if it runs on this machine, otherwise "
                             "the autopilot's, over MAVLink FTP (the real "
                             "aircraft). The .lua files are verified on every "
                             "launch and only changed ones are sent.")
    parser.add_argument("--lua-serial", type=int, default=5, metavar="N",
                        help="SERIALn on the flight controller carrying the "
                             "steering link (default 5). Must NOT be the port "
                             "used by --connect.")
    parser.add_argument("--lua-path", metavar="PORT", default=None,
                        help="REQUIRED (except with --arduplane-autotune). The "
                             "control law runs in the ArduPilot Lua script "
                             "(LUA/balloon_chaser_fc.lua); PORT is the serial device "
                             "or tcp:host:port carrying the steering link, "
                             "SEPARATE from --connect.")
    args = parser.parse_args()
    # The control law exists only on the flight controller, so there is nothing
    # to fly without the steering link.  The ArduPlane AUTOTUNE maintenance mode
    # flies no balloon chaser and needs no link.
    if not args.lua_path and not args.arduplane_autotune:
        parser.error("--lua-path is required: the control law runs only on "
                     "the flight controller")

    # Autopilot link is the same for sim and real (both run ArduPlane over MAVLink).
    from Companion.adapters.mavlink_link import MavlinkAutopilotLink
    link = MavlinkAutopilotLink(args.connect)

    # --lua-path: wrap the link so it carries the steering link to the FC.  A
    # decorator, not a replacement - the MAVLink link keeps doing telemetry,
    # params, arming and the catapult, and GUIDE / STATE ride beside it.
    if args.lua_path:
        # The scripting parameters are saved on the FC and set by hand; startup
        # only CHECKS them and halts for the operator on any difference -
        # SCR_ENABLE silently off would mean no script is flying at all.
        # 1. Check the scripting parameters (read-only), BEFORE the scripts are
        #    written.  The operator may stop at the halt; written first, the new
        #    files would sit on disk unloaded, and the next launch would find
        #    them up to date and never reload the FC, which goes on flying the
        #    old script with this build's gain indices.
        from Companion.adapters.lua_setup import (ensure_scripting,
                                                       restart_scripting)
        ensure_scripting(link, serial_index=args.lua_serial)

        # 2. Put the scripts on the FC and verify every byte landed: a local
        #    copy when the scripts directory is a path here (--lua-scripts, or
        #    SITL running on this machine), MAVLink FTP otherwise - the real
        #    aircraft's SD card.  The eeprom.bin and fixed-path guesses are not
        #    consulted for that choice: a development machine has them while
        #    flying the real aircraft, and a copy there is never read.
        from Companion.adapters.lua_deploy import (
            deploy as _deploy_lua, deploy_ftp, _sitl_cwd_from_proc)
        if args.lua_scripts or _sitl_cwd_from_proc():
            _ok, _scripts_changed, _dest = _deploy_lua(dest_dir=args.lua_scripts)
        else:
            _ok, _scripts_changed, _dest = deploy_ftp(link)
        if not _ok:
            raise SystemExit(
                "[lua] script deployment failed - see the errors above. "
                "Running with a partially-written script is worse than not "
                "running at all.")

        # 3. ArduPilot reads scripts once at VM start, so new files need the VM
        #    reloaded in place to run.
        if _scripts_changed and not restart_scripting(link):
            raise SystemExit(
                "[lua] the scripting engine could not be reloaded - see the "
                "errors above before using --lua-path.")

        import serial as _pyserial
        from Companion.adapters.lua_offload import LuaOffloadLink
        if args.lua_path.startswith("tcp:"):
            import socket
            import select as _select
            import time as _time
            _log = logging.getLogger(__name__)
            _h, _p = args.lua_path[4:].rsplit(":", 1)
            _sock = socket.create_connection((_h, int(_p)))
            _sock.setblocking(False)

            class _SockPort:                      # duck-types pyserial
                # NON-BLOCKING SOCKET, so sendall() is the wrong call.  It has no
                # way to report how much it sent before it gave up: on a full
                # kernel buffer it raises BlockingIOError, the main loop dies, and
                # the FC may already have half a GUIDE frame on the wire.
                #
                # Send the remainder explicitly instead, waiting on writability
                # against a short deadline.  Backpressure on a 47-byte frame at
                # 50 Hz means the FC has stopped draining the port, which is a
                # condition to REPORT and fly through, not one to crash on.  The
                # return says how many bytes left, as pyserial's does: a GUIDE
                # whose tail is dropped is followed by the next tick's (the FC
                # holds the last whole GUIDE until then, and stands down only
                # after GUIDE_STALE_MS without a new one), and a gain push that
                # did not leave whole is sent again.
                # Every frame carries magic, length and CRC, so the FC's parser
                # resynchronises on the next whole frame.
                def write(self, b):
                    mv = memoryview(b)
                    deadline = _time.monotonic() + 0.02
                    while mv:
                        try:
                            mv = mv[_sock.send(mv):]
                        except BlockingIOError:
                            if _time.monotonic() >= deadline:
                                _log.warning(
                                    '[lua] link backed up, dropped %d of %d bytes '
                                    '- the FC is not draining the port', len(mv), len(b))
                                return len(b) - len(mv)
                            _select.select([], [_sock], [], 0.002)
                    return len(b)
                @property
                def in_waiting(self):
                    try:
                        return len(_sock.recv(4096, socket.MSG_PEEK))
                    except (BlockingIOError, OSError):
                        return 0
                def read(self, n):
                    try:
                        return _sock.recv(n) if n else b""
                    except (BlockingIOError, OSError):
                        return b""
            _port = _SockPort()
        else:
            # 921600, matching port:begin() in balloon_chaser_fc.lua.  BOTH SIDES MUST
            # AGREE or the frames arrive as garbage and the CRC rejects every
            # one, which looks exactly like the FC not running.
            #
            # STATE is a raw serial frame at the tick rate, so bandwidth is the
            # cap on how much FC state can be reported.  At 115200 the frame
            # ceiling is 230 B at 50 Hz, which leaves little room to report
            # the FC's values.
            _port = _pyserial.Serial(args.lua_path, 921600, timeout=0)
        link = LuaOffloadLink(link, _port)
        # Drops nothing: every MAVLink stream left is either rate-locked to
        # ATTITUDE or the only source of something.  The call logs why.
        link.drop_telemetry_the_fc_does_not_need()
        link.start_telemetry()
        # Generous first wait: the autopilot may still be booting (GPS probe and
        # EKF alignment alone take ~10 s in SITL), and the scripting thread does
        # not start until init_ardupilot finishes.  Deciding "not running" before
        # that is what made the previous attempt fight the FC's own startup.
        if not link.wait_for_fc(20.0):
            # Scripts are read when the scripting engine starts, so files that
            # landed after the autopilot booted are not running yet.  Reload the
            # Lua VM in place - never a reboot, which startup must not do, and no
            # SITL restart or dropped TCP link - once, then retry once.  By now
            # the FC has finished booting, so the STOP cannot race the scripting
            # thread's own startup.
            print("[lua] no script after 20s - reloading the scripting engine")
            restart_scripting(link._inner)
            link.wait_for_fc(15.0)
        if not link.lua_frames_rx:
            raise SystemExit(
                "[lua] the flight controller script is not responding - "
                "refusing to fly a path that cannot work. See the checks above.")
        print(f"[lua] steering offloaded to the FC via {args.lua_path}")

    # Maintenance mode: drive ArduPlane AUTOTUNE and exit. Reuses the link's proven
    # catapult_launch + MAVLink plumbing; builds none of the vision/gimbal/HUD stack.
    if args.arduplane_autotune:
        from Companion.arduplane_autotune import run_arduplane_autotune
        run_arduplane_autotune(link, goal_alt=args.alt)
        return

    if args.platform == 'sim':
        from Companion.adapters.sim_gazebo import (
            GazeboCamera, GzGimbalDriver, HsvDetectionSensor, NoseCameraIntrinsics,
            SdfManifest,
            GAZEBO_CAMERA_TOPIC, GAZEBO_GIMBAL_CAMERA_TOPIC)
        nose_cam   = GazeboCamera(topic=GAZEBO_CAMERA_TOPIC)
        gimbal_cam = GazeboCamera(topic=GAZEBO_GIMBAL_CAMERA_TOPIC)
        gimbal     = GzGimbalDriver()
        sensor     = HsvDetectionSensor(camera=gimbal_cam, gimbal=gimbal, link=link)
        # DISPLAY-ONLY: read the nose camera's true intrinsics from Gazebo so the
        # HUD projection uses real values (e.g. the native render resolution for
        # the aspect correction), not assumptions.
        nose_intrinsics = NoseCameraIntrinsics()
        # GROUND-TRUTH camera/gimbal parameters, parsed from the ACTIVE model SDF
        # at launch (the sim analogue of the ArduPlane parameter gate).  Logged
        # for comparison ONLY - never used in any calculation.
        gimbal_fov_truth = SdfManifest()
    else:  # real
        from Companion.adapters.real_hardware import (
            PhysicalGimbalDriver, CompanionDetectionSensor, V4l2NoseCamera)
        nose_cam   = V4l2NoseCamera(device="/dev/video0")
        gimbal_cam = V4l2NoseCamera(device="/dev/video1")
        # TODO when implementing real_hardware: settle protocol choice first.
        gimbal     = PhysicalGimbalDriver(link=link, slew_rate_dps=...)
        sensor     = CompanionDetectionSensor(gimbal=gimbal, link=link,
                                              detection_endpoint="udp://0.0.0.0:14660")
        nose_intrinsics = None   # real hardware: HUD falls back to nominal FOV
        gimbal_fov_truth = None  # real hardware: no SDF ground truth

    from Companion.controller import BalloonChaser
    controller = BalloonChaser(
        link=link, gimbal=gimbal, sensor=sensor,
        nose_camera=nose_cam, gimbal_camera=gimbal_cam,
        takeoff_alt=args.alt,
        waypoint_lat=args.lat, waypoint_lon=args.lon)
    # DISPLAY-ONLY intrinsics for the HUD nose projection (never control).
    controller.nose_intrinsics = nose_intrinsics
    # GROUND-TRUTH FOV, for logging/comparison ONLY (never used in calculation).
    controller.gimbal_fov_truth = gimbal_fov_truth
    controller.calibrate_gimbal_on_startup = args.calibrate_gimbal
    # Hand the startup-log buffer to the CSV logger so it can drain it into
    # the head of the file when (and only when) the operator enables logging.
    controller._csv._startup_buffer = _startup_buffer
    controller.run()


if __name__ == "__main__":
    main()
