# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
CSV diagnostic logger.

Owns all CSV state (file handle, writer, mode-event tracking, batched flush).
The controller builds a TickSnapshot once per tick and hands it to write_tick();
the logger never reaches into controller internals.

The HEADER column list and the writerow list are append-only and must stay in
lockstep, so a column never shifts meaning between runs - test_csv_alignment
enforces it.  code_sha256 (package_sha256, below) fingerprints the whole
package and the project's .md documents, so every log records which code, and
which rules and design, produced it.
"""
from __future__ import annotations

import csv
import hashlib
import logging
import math
import os

from dataclasses import dataclass
from typing import Optional

import numpy as np


# -----------------------------------------------------------------------------
# Source fingerprint (package-wide).
# -----------------------------------------------------------------------------
def package_sha256() -> str:
    """SHA-256 over every .py, .lua AND .md file, in sorted-path order: the
    Companion package, the repo-root main.py, the flight scripts in LUA/, and
    every Markdown document in the repo.

    Returns the 64-char hex digest, or "unknown" if the tree can't be read.
    Hashes raw bytes (line endings, BOM, trailing whitespace all matter -
    deliberate, the files on disk are what's running).

    THE .lua FILES ARE NOT OPTIONAL HERE.  The steering cascade runs on the
    flight controller, so a fingerprint over .py alone would be IDENTICAL for
    two runs flown by completely different control laws - which is the one thing
    a fingerprint exists to make impossible.  main.py is in for the same reason:
    it sets the serial link and the startup order, outside the package.

    The .md files are in because they are the rules and the design the code is
    written against (CLAUDE.md, ARCHITECTURE.md and the rest, wherever they sit
    in the repo): a log's digest names the documents as well as the code it was
    flown with.  A docs-only edit therefore moves the digest, and discards the
    resume checkpoints that compare it (plant-ID, the trim map) exactly as a
    code edit does.
    """
    try:
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        lua_dir = os.path.join(os.path.dirname(pkg_dir), "LUA")
        h = hashlib.sha256()
        paths = []
        for root, _dirs, files in os.walk(pkg_dir):
            for name in files:
                if name.endswith(".py"):
                    paths.append(os.path.join(root, name))
        repo_dir = os.path.dirname(pkg_dir)
        for root, dirs, files in os.walk(repo_dir):
            # Hidden directories (.git, .claude) and bytecode caches hold no
            # project document.
            dirs[:] = [d for d in dirs
                       if not d.startswith(".") and d != "__pycache__"]
            for name in files:
                if name.endswith(".md"):
                    paths.append(os.path.join(root, name))
        main_py = os.path.join(repo_dir, "main.py")
        if os.path.isfile(main_py):
            paths.append(main_py)
        if os.path.isdir(lua_dir):
            for name in os.listdir(lua_dir):
                # Only what actually flies: the desktop shim, stubs and test
                # drivers never reach the aircraft and must not move the digest.
                if (name.endswith(".lua")
                        and not name.startswith(("test_", "xcheck_", "e2e_",
                                                 "replay_", "hello_"))
                        and name not in ("ap_shim.lua", "ap_stubs.lua")):
                    paths.append(os.path.join(lua_dir, name))
        for p in sorted(paths):
            # The path is hashed so a rename changes the digest; relative to
            # the REPO root so LUA/ and Companion/ cannot collide.
            rel = os.path.relpath(p, os.path.dirname(pkg_dir)).encode("utf-8")
            h.update(rel)
            with open(p, "rb") as f:
                h.update(f.read())
        return h.hexdigest()
    except (NameError, OSError, IOError):
        return "unknown"


PACKAGE_SHA256 = package_sha256()


# -----------------------------------------------------------------------------
# TickSnapshot - one row's worth of values, built by the controller per tick.
# Field order is documentary only; write_tick() references fields by name.
# Units noted match the source row formatting.
# -----------------------------------------------------------------------------
@dataclass
class TickSnapshot:
    t: float                 # seconds since logging start
    mode: str
    event: str               # already-resolved event string for this row

    # Outer-loop commands and actual state (radians in; written as degrees)
    requested_roll: Optional[float]    # rad
    act_roll: float          # rad
    requested_pitch: Optional[float]   # rad
    log_req_pitch_raw: Optional[float] # rad; the FC does not send it - None, written blank
    act_pitch: float         # rad

    # Airspeed->throttle loop.  thr_pct is the value that reached the throttle
    # channel; g_throttle_pct stays the OPERATOR's setting, so with the loop on
    # the two differ and the pair shows what the loop did with the input it was
    # given.  i_term is the trim throttle the integrator is holding.
    log_asp_en: Optional[int]
    log_asp_err_mps: Optional[float]
    log_asp_i_term: Optional[float]
    log_asp_thr_pct: Optional[float]
    log_asp_sat: Optional[int]

    # Cascade decomposition
    log_p_term_roll: Optional[float]
    pan_integrator: Optional[float]
    pitch_integrator: Optional[float]
    # --- control-law debug instrumentation (column semantics in HEADER) ---
    log_roll_cmd_presat: Optional[float]
    log_roll_sat: Optional[int]
    log_d_term_roll: Optional[float]
    log_d_term_pitch: Optional[float]
    log_pan_i_frozen: Optional[int]
    log_pan_i_clamped: Optional[int]
    log_pitch_i_frozen: Optional[int]
    log_pitch_i_clamped: Optional[int]
    log_in_tol: int
    log_align_tol_deg: float
    g_shallow_bank_lim: float
    g_shallow_pan_kd: float
    g_pitch_kd: float

    # Body-frame errors
    # The FC's steering error: pan is the flight-path lateral error the roll law
    # closes (asin(t'_y), the balloon's angle to the side of the path in the path
    # frame), tilt the NOSE's tilt to the balloon.  NOT the alignment measure -
    # see align_pan_deg / align_tilt_deg.
    steering_pan_deg: Optional[float]
    steering_tilt_deg: Optional[float]
    roll_align_err: float       # un-wrapped degrees(atan2(ty,tz)) (0=belly,
                                # +-180=inverted).  Distinct from the HUD's wrapped
                                # "Roll err" and from steering_pan (the FC's pan_err).
    log_balloon_elev: Optional[float]    # the FC does not send it - None, written blank
    log_aoa_lat: Optional[float]        # the FC does not send it - None, written blank

    # Vision diagnostics
    log_det_norm_x: float
    log_det_norm_y: float
    world_est_trust: Optional[float]

    # ptest / oracle diagnostics
    aircraft_lat: float
    aircraft_lon: float
    aircraft_relative_alt: float
    injected_world_pos: Optional[np.ndarray]   # (3,) NED or None

    # Gimbal
    gimbal_bearing_pan: float
    gimbal_bearing_tilt: float
    gimbal_meas_pan: float      # TRUE gimbal pose from readback (== commanded if
    gimbal_meas_tilt: float     # no readback).  Diff vs bearing_* = PID slew lag.
    gimbal_quat_x: float        # raw gimbal_cam_link quaternion from the readback
    gimbal_quat_y: float        # (0,0,0,0 if no readback) - lets the pan/tilt
    gimbal_quat_z: float        # conversion signs be verified offline from the log.
    gimbal_quat_w: float

    # Nose-projection debug (diagnosing the red-dot mismatch).  These capture the
    # EXACT pixels the HUD drew plus the body-frame balloon, so the projection can
    # be checked from the log instead of reconstructed.  Pixels are -1 when not
    # available (nose behind gimbal / no detection).
    dbg_nose_px: int            # red nose dot x in the gimbal image (-1 = behind)
    dbg_nose_py: int            # red nose dot y
    dbg_det_px: int             # green balloon/detection dot x (-1 = none)
    dbg_det_py: int             # green balloon/detection dot y
    dbg_balloon_body_x: float    # GREEN (actual) body-frame balloon unit vector
    dbg_balloon_body_y: float
    dbg_balloon_body_z: float
    dbg_control_body_x: float   # YELLOW (control-path) body-frame balloon vector
    dbg_control_body_y: float
    dbg_control_body_z: float

    # --- pitch-debug instrumentation.  The EXACT signals the body-plane pitch
    #     law consumes, plus the velocity vector, so the flight path can be
    #     scored directly instead of inferred.  For the near-vertical pitch
    #     stall / cyan-marker-off-balloon case.
    #   aim_body_*   : the true body-frame AIM vector actually fed to the pitch
    #                  law (AoA estimate added, post through-vertical wrap, projected
    #                  with the live attitude) - NOT the display marker above.
    #   dbg_theta_ip_deg : the in-plane pitch angle the law ACTS ON (degrees) -
    #                  the body-frame atan2(-aim_z,aim_x) sub-vertical, blended to
    #                  the roll-stabilized elevation gap once the demand passes
    #                  vertical (the through-vertical pitch fix).
    #   dbg_theta_ip_body_deg : the RAW body-frame atan2(-aim_z,aim_x) before
    #                  roll-stabilization - idles toward 0 at high bank past
    #                  vertical; compare against dbg_theta_ip_deg to see the fix.
    #   los_elev_deg : RAW line-of-sight elevation BEFORE the AoA estimate is
    #                  added.  Read this column for the LOS.
    #   vel_n/e/d    : NED velocity (m/s); flight-path angle = atan2(-vel_d,
    #                  hypot(vel_n,vel_e)).
    # dbg_aim_body_* and dbg_theta_ip_body_deg: the FC does not send them -
    # None, written blank.
    dbg_aim_body_x: Optional[float]
    dbg_aim_body_y: Optional[float]
    dbg_aim_body_z: Optional[float]
    dbg_theta_ip_deg: Optional[float]
    dbg_theta_ip_body_deg: Optional[float]
    los_elev_deg: Optional[float]   # the FC's; None on a tick the law did not run
    vel_n: Optional[float]
    vel_e: Optional[float]
    vel_d: Optional[float]

    # Aircraft telemetry
    aircraft_airspeed: float
    act_yaw: float           # rad
    roll_rate: float         # rad/s
    pitch_rate: float        # rad/s
    yaw_rate: float          # rad/s

    # Aircraft ATTITUDE QUATERNION (w,x,y,z) straight from ATTITUDE_QUATERNION,
    # singularity-free.  Plus true_bank_deg / true_pitch_deg recomputed FROM the
    # quaternion: these let the log show whether a wild act_roll near vertical is a
    # real attitude or an Euler gimbal-lock artifact (act_roll is the Euler angle,
    # which is ill-defined pointing straight down; true_bank from the quaternion is
    # the meaningful comparison).  quat_fresh = 1 only when a NEW quaternion arrived
    # since the previous control tick (per-tick freshness); mean(quat_fresh) over a
    # log therefore measures the EFFECTIVE attitude stream rate as a fraction of the
    # loop rate, which is the number to check before trusting any quantity
    # differentiated from attitude.  It runs below 1 in practice: the attitude
    # stream is slower than the control loop, so some ticks reuse the previous
    # quaternion.
    aircraft_quat_w: float
    aircraft_quat_x: float
    aircraft_quat_y: float
    aircraft_quat_z: float
    true_bank_deg: float
    true_pitch_deg: float
    quat_fresh: int
    control_dt: float

    # Servo + PWM (downstream of ArduPlane rate PID)
    servo_aileron: int
    servo_elevator: int
    servo_ch3: int
    servo_rudder: int
    servo_ch5: int
    servo_ch6: int
    servo_ch7: int
    servo_ch8: int
    # No stick PWM is sent and no rate is computed on this side: None, written
    # blank.  The flown rates are lua_omega_x / lua_omega_y.
    last_pwm_ch1: Optional[int]
    last_pwm_ch2: Optional[int]
    log_rate_cmd_pitch: Optional[float]
    log_rate_cmd_roll: Optional[float]

    # Tunable gains snapshot (flight context: which gains were live this tick)
    g_shallow_pan_p_gain: float
    g_shallow_pan_ki: float
    g_shallow_windup_limit_pan: float
    g_pitch_ki: float
    g_windup_limit_pitch: float
    g_shallow_roll_gain: float
    g_pitch_gain: float
    # THE DE-SKEW LAW'S ENTIRE GAIN SET.  The law has one input, one gain and one
    # state, so if the gain is missing a log can show the loop doing nothing and
    # not say whether that is the aircraft or the operator; it must be recorded.
    g_pitch_turn_ff: float
    g_yaw_point_kp: float
    g_yaw_point_ki: float
    g_yaw_point_lim: float
    g_yaw_point_windup: float
    g_throttle_pct: float
    # Every remaining ControlGains field, from the set flying this tick.
    # g_<field> is the field's own name.
    g_pitch_deadband_deg: float
    g_pitch_kd_rate: float
    g_airspeed_cmd_enable: int
    g_airspeed_cmd_mps: float
    g_airspeed_cmd_kp: float
    g_airspeed_cmd_ki: float
    g_airspeed_cmd_min_valid: float
    g_pan_kd_rate: float
    g_aileron_override: float
    g_aileron_deflection_pct: float
    g_elevator_override: float
    g_elevator_deflection_pct: float
    g_rudder_override: float
    g_rudder_deflection_pct: float
    g_yaw_point_pitch_sched: int
    # Saved ArduPlane parameters (constants.ARDUPLANE_AUTOTUNE_PARAMS): the value the
    # FC last confirmed - startup read, a Read, or the readback after a Write -
    # as the Gain Tuner holds it.  None until the FC has answered for it.
    fc_param_ptch2srv_tconst: Optional[float]
    fc_param_ptch2srv_rmax_up: Optional[float]
    fc_param_ptch2srv_rmax_dn: Optional[float]
    fc_param_ptch_rate_ff: Optional[float]
    fc_param_ptch_rate_p: Optional[float]
    fc_param_ptch_rate_i: Optional[float]
    fc_param_ptch_rate_d: Optional[float]
    fc_param_ptch_rate_imax: Optional[float]
    fc_param_ptch_rate_fltt: Optional[float]
    fc_param_ptch_rate_flte: Optional[float]
    fc_param_ptch_rate_fltd: Optional[float]
    fc_param_ptch_rate_smax: Optional[float]
    fc_param_rll2srv_tconst: Optional[float]
    fc_param_rll2srv_rmax: Optional[float]
    fc_param_rll_rate_ff: Optional[float]
    fc_param_rll_rate_p: Optional[float]
    fc_param_rll_rate_i: Optional[float]
    fc_param_rll_rate_d: Optional[float]
    fc_param_rll_rate_imax: Optional[float]
    fc_param_rll_rate_fltt: Optional[float]
    fc_param_rll_rate_flte: Optional[float]
    fc_param_rll_rate_fltd: Optional[float]
    fc_param_rll_rate_smax: Optional[float]
    fc_param_yaw_rate_ff: Optional[float]
    fc_param_yaw_rate_p: Optional[float]
    fc_param_yaw_rate_i: Optional[float]
    fc_param_yaw_rate_d: Optional[float]
    fc_param_yaw_rate_imax: Optional[float]
    fc_param_yaw_rate_fltt: Optional[float]
    fc_param_yaw_rate_flte: Optional[float]
    fc_param_yaw_rate_fltd: Optional[float]
    fc_param_yaw_rate_smax: Optional[float]
    # FC flight mode as reported live by HEARTBEAT custom_mode (ArduPlane: 0=MANUAL,
    # 4=ACRO, 5=FBWA, 12=LOITER, ...).  Logged so every row records which controller
    # was actually flying the aircraft - trials assume ACRO, level-outs FBWA, and a
    # silent mode mismatch is otherwise invisible in the log.  Appended LAST (with a
    # default) so older positional CSV parsers and older row constructors keep working.
    fc_mode: int = 0
    # FC rate-loop internals from PID_TUNING (GCS_PID_MASK=3). The FC's OWN
    # desired/achieved rate and P/I/D/FF/SRate terms per axis - the authoritative
    # inner-loop view (the noisy passthrough gyro is roll_rate/pitch_rate above).
    # None until the FC streams that axis, written blank.  Column order is the
    # HEADER list, not this file.
    fc_roll_des: Optional[float] = None
    fc_roll_ach: Optional[float] = None
    fc_roll_ff: Optional[float] = None
    fc_roll_p: Optional[float] = None
    fc_roll_i: Optional[float] = None
    fc_roll_d: Optional[float] = None
    fc_pitch_des: Optional[float] = None
    fc_pitch_ach: Optional[float] = None
    fc_pitch_ff: Optional[float] = None
    fc_pitch_p: Optional[float] = None
    fc_pitch_i: Optional[float] = None
    fc_pitch_d: Optional[float] = None
    # YAW.  The rudder law commands a RATE, so this is the only place the log
    # says whether the FC received it and whether the loop tracked it.
    # fc_yaw_des is post-clamp (the FC constrains to +-ACRO_YAW_RATE), so
    # fc_yaw_des != lua_yaw_rate_cmd_dps means the clamp bit.
    fc_yaw_des: Optional[float] = None
    fc_yaw_ach: Optional[float] = None
    fc_yaw_ff: Optional[float] = None
    fc_yaw_p: Optional[float] = None
    fc_yaw_i: Optional[float] = None
    fc_yaw_d: Optional[float] = None

    # Slant range from the aircraft to the balloon this tick (m).  None whenever
    # the balloon has no finite position - a direction anchor is at infinity, and
    # real vision is bearing-only - and written as a BLANK cell, which pandas
    # reads as NaN.  The HUD shows "Infinity" for the same condition.
    balloon_range_m: Optional[float] = None
    # Closest distance at the closest point of approach (m): how close the nose got
    # at the moment the aircraft passed the balloon.  None (blank) until the pass
    # is detected, then held for the rest of the run.
    pass_distance_m: Optional[float] = None

    # --- bank kinematics.  act_roll is the EULER angle, but the roll law
    #     feeds back the PROXY, so these carry the
    #     quantity the roll loop actually closes on and the terms that move it.
    #     Defaulted so older row constructors keep working.
    #   bank_proxy_deg    : asin(sin(roll)cos(pitch)) - the wing tip's angle
    #                       below the horizon: the cascade's feedback bank, and
    #                       the bank either law's Bank Lim holds.
    #   roll_err_proxy_deg: setpoint - feedback, in proxy terms;
    #                       rate_cmd_roll = this * g_shallow_roll_gain.
    #   bank_rate_cmd_dps / bank_rate_coupling_dps: the exact two-term split of
    #                       d(bank_proxy)/dt, computed on the FC (cascade.lua).
    #                       Their sum is the whole bank rate; the coupling half
    #                       is not commanded by anything and reverses sign with
    #                       pitch, so comparing the two columns says whether the
    #                       roll loop is being helped or fought at this attitude.
    #   upright           : cos(roll)cos(pitch); negative = inverted.  The proxy
    #                       alone cannot distinguish belly-up from a small bank.
    #   acro_roll_rate_lim: the envelope rate_cmd_roll is clipped against, so a
    #                       rate-limited command is visible without assuming the
    #                       FC parameter.
    bank_proxy_deg: Optional[float] = None
    roll_err_proxy_deg: Optional[float] = None
    bank_rate_cmd_dps: Optional[float] = None       # the FC's; None when the law
    bank_rate_coupling_dps: Optional[float] = None  # did not run this tick
    upright: Optional[float] = None
    acro_roll_rate_lim: float = 0.0

    # SITL ground truth (SIMSTATE), None on real hardware.  ATTITUDE carries the
    # EKF's attitude next to a gyro-derived rate and nothing guarantees the two
    # agree; these are the reference that says which one is wrong.  No control
    # path reads them.
    # Airflow angles from the velocity vector in BODY axes.  Sideslip is the
    # entire source of yaw while no rudder is commanded, and it is the only
    # aerodynamic state that was never recorded.  Ground velocity, so these are
    # true in still air; with wind they are offset by the wind vector but stay
    # comparable between cells flown in the same conditions.
    # THE ALIGNMENT ERROR: the flight path against the balloon
    # (controller.alignment_errors), the quantity the laws drive to zero and the
    # one every alignment check applies ALIGN_TOL_DEG to.  An aircraft flies a
    # path shallower than its nose, so pointing the boresight at a balloon makes
    # it overfly.
    #
    # steering_tilt_deg above is NOSE-referenced and settles at -AoA by design;
    # beside the path-referenced steering_pan_deg the pair looks like one
    # tracking error with an unexplained pitch bias.  These two settle at zero
    # on both axes and are the columns to judge the loops by.
    align_pan_deg: Optional[float] = None
    align_tilt_deg: Optional[float] = None
    sideslip_deg: Optional[float] = None
    aoa_deg: Optional[float] = None
    sim_roll_deg: Optional[float] = None
    sim_pitch_deg: Optional[float] = None
    sim_p_dps: Optional[float] = None
    sim_r_dps: Optional[float] = None

    # --- The FC's mirrored state.  DEFAULTED, therefore LAST: every field
    # without a default must precede every field with one, or the dataclass
    # raises at import.
    lua_omega_x: Optional[float] = None
    # Rate demand BEFORE the ACRO clip. Equal to lua_omega_x/y means the law
    # got what it asked for; different means it is clipped and no gain change
    # will help - the envelope is the limit.
    lua_omega_x_presat: Optional[float] = None
    lua_omega_y_presat: Optional[float] = None
    # Written by the FC and decoded here; logged so the pre-limit bank the law
    # asked for is visible in the record, not just on the wire.
    lua_roll_cmd_presat: Optional[float] = None  # bank the law asked for, BEFORE the limit (deg)
    lua_requested_roll: Optional[float] = None   # bank AFTER the limit (rad)
    lua_requested_pitch: Optional[float] = None  # pitch setpoint the rate loop tracks (rad)
    lua_bank_lim_deg: float = 45.0    # the limit in force this tick (deg)
    lua_roll_err_deg: Optional[float] = None     # bank error the rate loop acts on
    lua_throttle: Optional[float] = None         # throttle the FC commanded, %
    lua_fc_millis: int = 0            # FC clock. Delta against the companion's
                                      # monotonic clock over the same window is
                                      # the ONLY valid RTF measurement.
    lua_cs_flags: Optional[int] = None             # saturation / integrator-freeze bits
    # -- YAW / COORDINATION --------------------------------------------
    # BODY-frame specific force from the FC's primary IMU.  lua_accel_y is
    # the turn-coordination signal: zero in a coordinated turn, and unlike
    # sideslip_deg it does NOT move with wind.  sideslip_deg is computed
    # from GROUND velocity, so a 3 m/s crosswind at 15 m/s fabricates
    # 11.5 deg of sideslip - larger than the whole quantity being measured.
    # lua_accel_z is kept so lua_accel_y can be checked rather than trusted
    # (expect about -9.81/cos(bank) in a coordinated turn).
    lua_accel_y: Optional[float] = None
    lua_accel_z: Optional[float] = None
    # UNCOMMANDED yaw feedforwards: computed on the FC every tick, commanded
    # NOWHERE.  They confirm the arithmetic and the sizing against real
    # flight before a surface moves or ACRO_YAW_RATE is opened up.
    lua_yaw_ff_coord: Optional[float] = None     # g*sin(roll)*cos(pitch)/V, deg/s
    lua_yaw_ff_ari: Optional[float] = None       # rolling yaw p*tan(alpha), commanded p, deg/s
    # -- yaw law -------------------------------------------------------
    # One column, because the law is one expression. lua_accel_y says whether
    # it is working; this says what was commanded.
    lua_lat_marker: Optional[float] = None       # sideways gap, nose to flight path, deg
    lua_pan_err_deg: Optional[float] = None      # PATH minus BALLOON, HUD sign; roll's error
    lua_yaw_rate_cmd_dps: Optional[float] = None
    lua_yaw_point_dps: Optional[float] = None
    lua_pitch_turn_dps: Optional[float] = None
    # ArduPilot's OWN aerodynamic angles.  Its SSA subtracts the wind
    # estimate before computing the angle, so (sideslip_deg - fc_ssa)
    # IS the wind contamination in sideslip_deg.  None until the message
    # arrives, written blank.
    fc_aoa: Optional[float] = None
    fc_ssa: Optional[float] = None
    lua_omega_y: Optional[float] = None
    lua_gate: int = 0
    lua_trust: Optional[float] = None
    lua_dt_ms: int = 0
    lua_frames: int = 0
    # FOUR DECIMALS, not one.  Identification reads this to divide the FC's speed
    # compensation out of the aircraft's response, and the gain multiplier is the
    # SQUARE of it - so one decimal quantises a 1.2-1.5 range into four values
    # and buries up to 7% of gain error in the rounding, which is the same size
    # as the effects being identified.
    lua_scaler: Optional[float] = None
    lua_seq: int = 0
    lua_seq_lag: int = 0
    servo1_trim: float = 0.0
    servo1_min: float = 0.0
    servo1_max: float = 0.0
    servo2_trim: float = 0.0
    # The second ruddervator, so the V-tail split has a scale on both halves.
    servo4_trim: float = 0.0
    servo4_min: float = 0.0
    servo4_max: float = 0.0
    servo2_min: float = 0.0
    servo2_max: float = 0.0
    elevon_mixing: int = 0
    hz_quat: int = 0
    hz_gyro: int = 0
    hz_vel: int = 0
    hz_asp: int = 0
    gimbal_age_ms: int = 0
    rtf: float = 0.0
    gimbal_sim_age_ms: int = 0
    # Background FOV estimator (DIAGNOSTIC ONLY; 0 unless the switch is on).
    fov_est_deg: float = 0.0
    fov_est_m1_deg: float = 0.0
    fov_est_m2_deg: float = 0.0
    fov_truth_deg: float = 0.0     # SDF/CameraInfo ground truth - comparison only
    fov_est_n: int = 0
    # The FC's vertical flight-path marker: the path's angle off the nose in the
    # body pitch plane, positive above it, each tick's reading as it is.  Logged;
    # the laws read the path frame (track.lua), not the marker.
    lua_vel_marker: Optional[float] = None
    # The pitch trim gains flying this tick (offset and attitude here, lift and
    # thrust at the end of the row), and what the FC made of them: the trim
    # elevator (ArduPlane units, 45 = full travel) and the pitch-rate request
    # that carries it, which is inside lua_omega_y but never flown.
    g_pitch_trim_offset: float = 0.0
    g_pitch_trim_attitude: float = 0.0
    lua_pitch_trim_elev: Optional[float] = None
    lua_pitch_trim_dps: Optional[float] = None
    # --- STATE2 (protocol.STATE2): the FC's own sensors, law internals and
    # surfaces, in the units the FC holds.  None - written blank - before the
    # first frame, where a reading is absent, and for the law internals on a
    # tick the law did not run.  lua_state2_seq equal to lua_seq pairs the
    # row's two frames from the same tick.
    lua_state2_seq: Optional[int] = None
    lua_fc_mode: Optional[int] = None         # vehicle:get_mode() this tick
    lua_q_w: Optional[float] = None           # ahrs:get_quaternion(), world->body
    lua_q_x: Optional[float] = None
    lua_q_y: Optional[float] = None
    lua_q_z: Optional[float] = None
    lua_gyro_x_rads: Optional[float] = None   # ahrs:get_gyro(), rad/s, body axes
    lua_gyro_y_rads: Optional[float] = None
    lua_gyro_z_rads: Optional[float] = None
    lua_vel_n_mps: Optional[float] = None     # ahrs:get_velocity_NED(), m/s
    lua_vel_e_mps: Optional[float] = None
    lua_vel_d_mps: Optional[float] = None
    lua_eas_mps: Optional[float] = None       # ahrs:airspeed_EAS()
    lua_balloon_body_x: Optional[float] = None    # balloon direction the law flew,
    lua_balloon_body_y: Optional[float] = None    # body frame, unit vector
    lua_balloon_body_z: Optional[float] = None
    lua_yaw_point_i_dps: Optional[float] = None   # yaw pointing integrator
    lua_coord_valid: Optional[int] = None     # coordination feedforward valid
    lua_accel_valid: Optional[int] = None     # IMU specific-force read valid
    lua_lat_marker_valid: Optional[int] = None    # lateral marker valid
    # What ArduPlane asked each surface for (get_output_scaled, before the
    # V-tail mix: +-4500 = full travel, throttle in percent) and the servo
    # signal after the mix (get_output_pwm; 0 = no channel carries it).
    lua_out_aileron: Optional[float] = None
    lua_out_elevator: Optional[float] = None
    lua_out_rudder: Optional[float] = None
    lua_out_throttle: Optional[float] = None
    lua_pwm_aileron: Optional[int] = None
    lua_pwm_vtail_l: Optional[int] = None
    lua_pwm_vtail_r: Optional[int] = None
    # STATE flags bits 1 and 2: the FC's world-balloon estimate is valid, and an
    # EXCITE excitation is running.  Blank until the first STATE.
    lua_est_valid: Optional[int] = None
    lua_excite_running: Optional[int] = None
    # STATE flags bit 0: the script holds the controls this tick (its
    # nav_scripting claim succeeded).  Blank until the first STATE.
    lua_nav_active: Optional[int] = None
    g_pitch_trim_lift: float = 0.0
    g_pitch_trim_thrust: float = 0.0
    lua_eas2tas: Optional[float] = None       # ahrs:get_EAS2TAS()
    g_plane_side_hold_deg: float = 0.0
    g_plane_gain: float = 0.0
    g_plane_ff_gain: float = 0.0
    g_plane_ff_lpf_rate: float = 0.0
    lua_plane_drift_dps: Optional[float] = None      # balloon's filtered drift round the clock
    lua_plane_ff_dps: Optional[float] = None         # roll-to-plane feedforward
    g_plane_kd: float = 0.0
    g_plane_kd_rate: float = 0.0
    lua_plane_err_rate_dps: Optional[float] = None   # clock error's filtered rate
    lua_plane_d_dps: Optional[float] = None          # roll-to-plane derivative term
    g_plane_steep_start_deg: float = 0.0
    g_plane_steep_full_deg: float = 0.0
    g_plane_ki: float = 0.0
    g_plane_windup: float = 0.0
    lua_plane_path_share: Optional[float] = None     # its share of the roll command, from the flight-path angle
    lua_plane_i_dps: Optional[float] = None          # roll-to-plane integrator
    lua_plane_i_frozen: Optional[float] = None       # 1 = integrator held at saturation
    lua_plane_path_deg: Optional[float] = None       # angle the path share read
    lua_plane_path_from_vel: Optional[float] = None  # 1 = velocity, 0 = nose pitch stood in
    lua_excite_value: Optional[float] = None         # EXCITE output, added to or replacing the command
    g_yaw_coordination_ff: float = 0.0               # scale on the coordination yaw, 1 = the geometry
    lua_yaw_coordination_dps: Optional[float] = None # coordination the yaw command carried, deg/s
    # Roll-to-plane, from STATE2; blank when the law did not run.
    lua_plane_off_nose_deg: Optional[float] = None   # balloon's angle off the nose
    lua_plane_err_deg: Optional[float] = None        # deg still to roll to 12 or 6 round the nose
    lua_plane_side: Optional[float] = None           # 1 = 12 o'clock, -1 = 6
    lua_plane_axis_up: Optional[float] = None        # world-vertical part of the turn axis
    # The Curve Flyer: its settings, the FC's state and what it hands the law
    # from STATE2 (blank on a tick the law flew no PLAN), and the PLAN the
    # Curve Planner made for this chase (blank until one is ready).
    g_curve_flyer_enable: int = 0
    g_tilt_back_travel_deg: float = 0.0
    lua_cf_state: Optional[int] = None
    lua_cf_plan_id: Optional[int] = None
    lua_cf_progress: Optional[float] = None
    lua_cf_eps_deg: Optional[float] = None
    lua_cf_bank_ref_deg: Optional[float] = None
    cf_plan_id: Optional[int] = None
    cf_plan_eps0_deg: Optional[float] = None
    cf_plan_knots_deg: Optional[tuple] = None
    cf_plan_s: Optional[float] = None
    g_cf_transfer_when_done: int = 0
    lua_cf_hold_n: Optional[float] = None
    lua_cf_hold_e: Optional[float] = None
    lua_cf_hold_d: Optional[float] = None
    g_cf_plan_wait_s: float = 0.0
    lua_path_turn_up_dps: Optional[float] = None
    lua_path_turn_right_dps: Optional[float] = None
    g_cf_approach_deg: float = 0.0
    cf_plan_runin_climb_deg: Optional[float] = None
    cf_plan_runin_dep_deg: Optional[float] = None
    # The steering law and Lift Vector Pointing: the gains flying this tick,
    # and the law's internals from STATE and STATE2, blank on a tick it did
    # not run.
    g_steering_law: int = 0
    g_att_k_path: float = 0.0
    g_att_path_rate_max_dps: float = 0.0
    g_att_lead_s: float = 0.0
    g_att_k_roll: float = 0.0
    g_att_k_side: float = 0.0
    g_att_roll_ki: float = 0.0
    g_att_pitch_ki: float = 0.0
    g_att_i_zone_deg: float = 0.0
    lua_pitch_trim_lift_term: Optional[float] = None
    lua_att_roll_i_dps: Optional[float] = None
    lua_att_pitch_i_dps: Optional[float] = None
    lua_att_wx_dps: Optional[float] = None
    lua_att_wy_dps: Optional[float] = None
    lua_att_wz_dps: Optional[float] = None
    lua_att_bank_deg: Optional[float] = None
    lua_att_band_lo_deg: Optional[float] = None
    lua_att_band_hi_deg: Optional[float] = None
    lua_att_drift_dps: Optional[float] = None
    lua_att_theta_now_deg: Optional[float] = None
    lua_att_p_dps: Optional[float] = None
    lua_att_q_dps: Optional[float] = None
    lua_att_theta_deg: Optional[float] = None
    lua_att_u_y: Optional[float] = None
    lua_att_u_z: Optional[float] = None
    lua_att_omega_dps: Optional[float] = None
    lua_att_f_y_mps2: Optional[float] = None
    lua_att_f_z_mps2: Optional[float] = None
    lua_att_mu_deg: Optional[float] = None
    lua_att_side: Optional[float] = None
    lua_att_w_f: Optional[float] = None
    lua_att_n_lift: Optional[float] = None
    # Body longitudinal specific force, beside lua_accel_y / lua_accel_z: the
    # Lift Vector Pointing's lead reads all three.
    lua_accel_x: Optional[float] = None
    lua_att_room_lo_deg: Optional[float] = None
    lua_att_room_hi_deg: Optional[float] = None
    lua_att_omega_ask_dps: Optional[float] = None
    cf_plan_law: Optional[int] = None                # Curve Planner: the steering law the plan was made for


_CSV_FLUSH_EVERY = 25   # ~0.5 s at 50 Hz - see flush note below.


def _knot(knots, i):
    """Knot i of a plan's bank schedule, or None with no plan."""
    return knots[i] if knots is not None and i < len(knots) else None


def _opt(x, fmt: str) -> str:
    """An optional value in `fmt`, or blank for None - never a zero."""
    return "" if x is None else format(x, fmt)


def _deg(rad, fmt: str) -> str:
    """An optional angle in radians, written in degrees; blank for None."""
    return "" if rad is None else format(math.degrees(rad), fmt)


def _comment_lines(label: str, text) -> str:
    """`text` as head-of-log comment lines, '# <label>: ' on EVERY line.  A
    console message can span several lines; written as one, its continuation
    lines carry no '#', and a reader skipping comments meets them as data rows
    before the header - pandas stops there, and a spreadsheet import shifts."""
    return "".join(f"# {label}: {part}\n" for part in (str(text).splitlines() or [""]))


class StartupBufferHandler(logging.Handler):
    """Captures formatted log lines into an in-memory list during startup, so
    they can be preserved in the head of the CSV log when (and only when) the
    operator enables logging.  Behaviour:

      - Active from install (early in main()) until drain() is called.
      - drain() returns the accumulated lines and flips the handler to
        permanently inactive: any further log records are silently dropped
        from the buffer (the StreamHandler that prints to the terminal is
        unaffected - this handler is an *additional* sink, not a replacement).
      - drain() is called from CsvLogger.start() on the first log enable ('q').
        Records before that go into the buffer; records after are buffer-free.
        If logging is never enabled, the buffer just sits in memory and is
        garbage-collected on process exit - no file is touched.

    Discarding records after drain (rather than continuing to buffer) is
    deliberate: post-startup messages have other capture mechanisms (terminal,
    and per-tick CSV data), and indefinitely growing a buffer that may never
    be read is a leak waiting to happen.
    """

    def __init__(self):
        super().__init__()
        self._lines = []
        self._closed = False

    def emit(self, record: logging.LogRecord) -> None:
        if self._closed:
            return
        try:
            self._lines.append(self.format(record))
        except Exception:                                  # pragma: no cover
            self.handleError(record)

    def drain(self):
        """Return buffered lines and close the buffer permanently.

        Idempotent: a second call returns an empty list and is harmless, so
        re-toggling logging via 'q' (stop then start again) does not double-
        write the startup lines into a second log file.
        """
        lines, self._lines = self._lines, []
        self._closed = True
        return lines


class CsvLogger:
    """Manual CSV logger toggled by the operator.

    Lifecycle: start() opens the file and writes the fingerprint + header;
    write_tick(snapshot) appends one row (batched flush); stop() flushes and
    closes.  active is True between start() and stop().
    """

    HEADER = [
        'tick',              # per-row counter, resets to 0 when logging starts
        'time', 'mode', 'event',
        # Outer-loop commands and actual state
        # ROLL SETPOINT AND FEEDBACK.  BOTH the setpoint and the loop's
        # feedback are the bank PROXY, not the Euler angle: req_roll is a proxy
        # setpoint and the quantity it is differenced against is
        # bank_proxy_deg, not act_roll.  act_roll is logged as the raw Euler
        # attitude for reference and is NOT in the loop.
        'req_roll',          # commanded bank (deg), a bank-proxy setpoint
        'act_roll',          # Euler roll attitude (deg); see bank_proxy_deg for the feedback
        'req_pitch',         # commanded pitch setpoint (deg)
        'req_pitch_raw',     # raw in-plane pitch setpoint (deg)
        'act_pitch',
        # Airspeed->throttle loop.  asp_thr_pct is what reached the throttle
        # channel; g_throttle_pct remains the operator's setting.  Diverging
        # columns mean the loop is active; equal columns mean pass-through.
        'asp_en',            # airspeed-command loop active as flown (0/1)
        'asp_err_mps',       # commanded - measured airspeed (m/s)
        'asp_i_term',        # trim throttle held by the integrator (%)
        'asp_thr_pct',       # throttle % actually issued
        'asp_sat',           # output on a throttle clamp (0/1)
        # Cascade decomposition: P + I feeding req_roll/req_pitch
        # p_term_roll is the roll law's P contribution, in deg of bank.
        'p_term_roll',
        'pan_integrator',    # bearing-error integrator (output: deg of bank)
        'pitch_integrator',  # pitch-error integrator, deg x s (x Pitch Ki = deg/s of pitch rate)
        # Body-frame errors (controller's actual closed-loop signal)
        'steering_pan_deg', 'steering_tilt_deg', 'roll_align_err',
        'balloon_elev',
        'aoa_lat',           # bank-projected AoA lateral correction to pan (deg)
        # Vision diagnostics
        'det_norm_x', 'det_norm_y',
        'world_est_trust',
        # ptest diagnostics: raw aircraft state used for balloon direction
        'aircraft_lat', 'aircraft_lon', 'aircraft_rel_alt',
        # ptest injected balloon world NED position (zeros otherwise)
        'inj_pos_n', 'inj_pos_e', 'inj_pos_d',
        # Gimbal
        'bearing_pan', 'bearing_tilt',
        'meas_pan', 'meas_tilt',
        'gimbal_quat_x', 'gimbal_quat_y', 'gimbal_quat_z', 'gimbal_quat_w',
        # Nose-projection debug (red-dot mismatch diagnosis)
        'dbg_nose_px', 'dbg_nose_py', 'dbg_det_px', 'dbg_det_py',
        'dbg_balloon_body_x', 'dbg_balloon_body_y', 'dbg_balloon_body_z',
        'dbg_control_body_x', 'dbg_control_body_y', 'dbg_control_body_z',
        # pitch-debug: exact signals the body-plane pitch law consumes + velocity
        'dbg_aim_body_x', 'dbg_aim_body_y', 'dbg_aim_body_z',
        'dbg_theta_ip_deg',  # in-plane pitch angle the law acts on (deg)
        'dbg_theta_ip_body_deg',  # raw body-frame theta_ip, pre roll-stab (deg)
        'los_elev_deg',      # RAW LOS elevation, before the AoA estimate (deg)
        'vel_n', 'vel_e', 'vel_d',   # NED velocity (m/s) -> flight-path angle
        # Aircraft telemetry
        'airspeed', 'act_yaw',
        'roll_rate', 'pitch_rate', 'yaw_rate',
        'aircraft_quat_w', 'aircraft_quat_x', 'aircraft_quat_y', 'aircraft_quat_z',
        'true_bank_deg', 'true_pitch_deg', 'quat_fresh',
        # The dt the control law ACTUALLY consumed this tick (s).  Not the same
        # as diff('time'): 'time' is stamped when the row is built, control_dt
        # is measured immediately before _update_control, so a gap between them
        # is itself diagnostic.  1/control_dt is the achieved loop rate.
        # Attitude rate needs no column - mean(quat_fresh) already gives it.
        'control_dt',
        # Servo + PWM (downstream of ArduPlane rate PID)
        'servo_ch1', 'servo_ch2', 'servo_ch3', 'servo_ch4',
        'servo_ch5', 'servo_ch6', 'servo_ch7', 'servo_ch8',
        'pwm_ch1', 'pwm_ch2',
        'rate_cmd_pitch', 'rate_cmd_roll',
        # --- Lua path: the FC's own record, blank where it sent nothing --
        # Without these the CSV shows only what the AIRCRAFT did, with no record
        # of what the flight controller commanded to produce it.  Diagnosing a
        # disagreement from that is guesswork.
        'lua_omega_x', 'lua_omega_y',
        'lua_omega_x_presat', 'lua_omega_y_presat',
        'lua_roll_cmd_presat', 'lua_requested_roll', 'lua_requested_pitch',
        'lua_bank_lim_deg',
        'lua_roll_err_deg', 'lua_throttle',
        'lua_fc_millis', 'lua_cs_flags',
        'lua_accel_y', 'lua_accel_z',
        'lua_yaw_ff_coord', 'lua_yaw_ff_ari',
        'lua_lat_marker', 'lua_pan_err_deg', 'lua_yaw_rate_cmd_dps',
        'lua_yaw_point_dps', 'lua_pitch_turn_dps',         'fc_aoa', 'fc_ssa',
        'lua_gate', 
        'lua_trust', 'lua_dt_ms', 'lua_frames',
        # ArduPlane's speed_scaler on this tick: SCALING_SPEED / airspeed_EAS,
        # clamped as calc_speed_scaler() does. NOT a flag - it moves with speed,
        # because the FC's speed compensation is part of the plant this steering
        # commands.
        #
        # SQUARE IT to get the gain multiplier. AP_FW_Controller.cpp feeds the
        # rate PID desired*scaler*scaler and measured*scaler*scaler, so 0.71 at
        # 21 m/s is 0.51x gain and 1.25 at 12 m/s is 1.56x - a 3x span.
        # 0.0 means it could not be computed (no airspeed).
        'lua_scaler',
        'lua_seq', 'lua_seq_lag',
        # The scaling behind the surface percentages.  Without these a reading
        # like "-18.2%" cannot be checked: it is (pwm - trim)/(max - trim), and
        # if trim defaulted to 1500 when the airframe actually trims at 1427,
        # a NEUTRAL surface reads as 18% deflected.
        'servo1_trim', 'servo1_min', 'servo1_max',
        'servo2_trim', 'servo2_min', 'servo2_max',
        'servo4_trim', 'servo4_min', 'servo4_max', 'elevon_mixing',
        # Measured FC-side sensor rates: how often each value actually CHANGED,
        # not how often it was read.  A 50 Hz read of a 5 Hz signal is 5 Hz.
        'hz_quat', 'hz_gyro', 'hz_vel', 'hz_asp',
        # gimbal_age_ms: how stale the gimbal angles were when the FC used them.
        #   Multiply by body rate for the attitude-pairing error.
        # rtf: simulated seconds per wall second, measured over ONE interval on
        #   TWO clocks.  Dividing lua_dt_ms by the companion tick does NOT work.
        'gimbal_age_ms', 'rtf', 'gimbal_sim_age_ms',
        # Tunable gains snapshot (flight context: which gains were live this tick)
        'g_shallow_pan_p_gain', 'g_shallow_pan_ki', 'g_shallow_pan_windup',
        'g_pitch_ki', 'g_pitch_windup',
        'g_shallow_roll_gain', 'g_pitch_gain', 'g_pitch_turn_ff', 'g_yaw_point_kp', 'g_yaw_point_ki', 'g_yaw_point_lim', 'g_yaw_point_windup',
        'g_throttle_pct',
        # FC flight mode (HEARTBEAT custom_mode) — appended last for positional compat
        'fc_mode',
        # FC rate-loop internals (PID_TUNING, GCS_PID_MASK=3) - appended last
        'fc_roll_des', 'fc_roll_ach', 'fc_roll_ff', 'fc_roll_p', 'fc_roll_i', 'fc_roll_d',
        'fc_pitch_des', 'fc_pitch_ach', 'fc_pitch_ff', 'fc_pitch_p', 'fc_pitch_i', 'fc_pitch_d',
        'fc_yaw_des', 'fc_yaw_ach', 'fc_yaw_ff', 'fc_yaw_p', 'fc_yaw_i', 'fc_yaw_d',
        # --- control-law debug instrumentation ---
        'roll_cmd_presat',    # bank command BEFORE the output clip (deg)
        'roll_sat',           # 1 = roll output clip active this tick
        'd_term_roll',        # kd lead-term contribution to bank (deg)
        'd_term_pitch',       # kd damping contribution to pitch rate (deg/s)
        'pan_i_frozen',       # 1 = roll integrator froze (conditional integration)
        'pan_i_clamped',      # 1 = roll integrator pinned at +-max_ri
        'pitch_i_frozen',     # 1 = pitch integrator froze
        'pitch_i_clamped',    # 1 = pitch integrator pinned at +-max_pi
        'in_tol',             # 1 = flight path within ALIGN_TOL_DEG of the balloon (aligned_now)
        'align_tol_deg',      # the alignment tolerance in force (deg)
        'g_shallow_bank_lim', # bank-angle clip in force (deg)
        'g_shallow_pan_kd',   # roll lead-term gain in force
        'g_pitch_kd',         # pitch damping gain in force
        # Slant range from the aircraft to the balloon this tick, metres.  BLANK
        # when the balloon has no finite position: a direction anchor is at
        # infinity, and real vision is bearing-only so it has no range at all.
        # (The HUD shows "Infinity" for the same condition.)
        'balloon_range_m',
        # Closest distance at the CLOSEST POINT OF APPROACH, metres - "how close the
        # nose got the moment the plane passed the balloon".  Blank on every row
        # until the pass is detected, then holds that one value for the rest of
        # the run.  Sub-tick accurate: fitted from the vertex of range^2, which is
        # exactly quadratic in t.  See BalloonChaser._update_balloon_range.
        'pass_distance_m',
        # BANK KINEMATICS.  act_roll above is the Euler angle; the loop closes
        # on bank_proxy_deg instead, and roll_err_proxy_deg is the error the
        # setpoint->rate conversion acts on.
        'bank_proxy_deg',      # asin(sin(roll)cos(pitch)): the wing tip's angle below the horizon, the bank Bank Lim holds; the cascade's feedback
        'roll_err_proxy_deg',  # setpoint - feedback; x g_shallow_roll_gain = rate_cmd_roll
        # Exact split of d(bank_proxy)/dt into the half the roll command bought
        # and the half yaw rate delivered uninvited:
        #   bank_rate = ( cos(roll)cos(pitch)*p + sin(pitch)*r ) / cos(bank)
        # The coupling term reverses sign with pitch, so it damps the roll loop
        # nose-down and erodes it nose-up at one fixed gain.  A coupling column
        # rivalling the command column is the loop being fought, not mistuned.
        'bank_rate_cmd_dps',
        'bank_rate_coupling_dps',
        'upright',             # cos(roll)cos(pitch); negative = inverted
        'acro_roll_rate_lim',  # envelope rate_cmd_roll is clipped against (deg/s)
        # The alignment error - flight path against balloon.  Settles at zero on
        # both axes; steering_tilt_deg does not.
        'align_pan_deg', 'align_tilt_deg',
        'sideslip_deg', 'aoa_deg',
        # SITL truth beside the estimate. Blank on real hardware.
        'sim_roll_deg', 'sim_pitch_deg', 'sim_p_dps', 'sim_r_dps',
        # --- Background FOV estimator (DIAGNOSTIC ONLY, all 0 unless the
        #     FOV Estimator switch is on).  fov_est_deg is the horizontal FOV
        #     read out of motion vs pixel shift; m1 = aircraft-rotation-dominant
        #     windows, m2 = gimbal-slew-dominant.  fov_truth_deg is the SDF/
        #     CameraInfo ground truth, logged for comparison ONLY and NEVER used
        #     in any calculation.  fov_est_n = accepted windows (confidence).
        'fov_est_deg', 'fov_est_m1_deg', 'fov_est_m2_deg',
        'fov_truth_deg', 'fov_est_n',
        # --- Every ControlGains field the columns above do not already carry, all
        #     from the set flying this tick (test_csv_alignment fails if a gain
        #     has no column).
        'g_pitch_deadband_deg', 'g_pitch_kd_rate',
        'g_airspeed_cmd_enable', 'g_airspeed_cmd_mps', 'g_airspeed_cmd_kp',
        'g_airspeed_cmd_ki', 'g_airspeed_cmd_min_valid', 'g_pan_kd_rate',
        'g_aileron_override', 'g_aileron_deflection_pct', 'g_elevator_override',
        'g_elevator_deflection_pct', 'g_rudder_override', 'g_rudder_deflection_pct',
        'g_yaw_point_pitch_sched',
        # --- Saved ArduPlane parameters, named exactly as on the FC: the value the
        #     FC last confirmed, as the Gain Tuner holds it (empty until it answers).
        'PTCH2SRV_TCONST', 'PTCH2SRV_RMAX_UP', 'PTCH2SRV_RMAX_DN', 'PTCH_RATE_FF',
        'PTCH_RATE_P', 'PTCH_RATE_I', 'PTCH_RATE_D', 'PTCH_RATE_IMAX',
        'PTCH_RATE_FLTT', 'PTCH_RATE_FLTE', 'PTCH_RATE_FLTD', 'PTCH_RATE_SMAX',
        'RLL2SRV_TCONST', 'RLL2SRV_RMAX', 'RLL_RATE_FF', 'RLL_RATE_P', 'RLL_RATE_I',
        'RLL_RATE_D', 'RLL_RATE_IMAX', 'RLL_RATE_FLTT', 'RLL_RATE_FLTE',
        'RLL_RATE_FLTD', 'RLL_RATE_SMAX', 'YAW_RATE_FF', 'YAW_RATE_P', 'YAW_RATE_I',
        'YAW_RATE_D', 'YAW_RATE_IMAX', 'YAW_RATE_FLTT', 'YAW_RATE_FLTE',
        'YAW_RATE_FLTD', 'YAW_RATE_SMAX',
        # The FC's vertical velocity marker, deg: what the pitch law subtracts.
        'lua_vel_marker',
        # The pitch trim: its offset and attitude gains (lift and thrust are at
        # the end of the row), the trim elevator the FC asked for and the rate
        # request carrying it (part of lua_omega_y, never flown).
        'g_pitch_trim_offset', 'g_pitch_trim_attitude',
        'lua_pitch_trim_elev', 'lua_pitch_trim_dps',
        'lua_state2_seq', 'lua_fc_mode',
        'lua_q_w', 'lua_q_x', 'lua_q_y', 'lua_q_z',
        'lua_gyro_x_rads', 'lua_gyro_y_rads', 'lua_gyro_z_rads',
        'lua_vel_n_mps', 'lua_vel_e_mps', 'lua_vel_d_mps', 'lua_eas_mps',
        'lua_balloon_body_x', 'lua_balloon_body_y', 'lua_balloon_body_z',
        'lua_yaw_point_i_dps',
        'lua_coord_valid', 'lua_accel_valid', 'lua_lat_marker_valid',
        'lua_out_aileron', 'lua_out_elevator', 'lua_out_rudder', 'lua_out_throttle',
        'lua_pwm_aileron', 'lua_pwm_vtail_l', 'lua_pwm_vtail_r',
        'lua_est_valid',       # 1 = the FC's world-balloon estimate is valid
        'lua_excite_running',  # 1 = an EXCITE excitation is running on the FC
        'lua_nav_active',      # 1 = the script holds the controls this tick
        'g_pitch_trim_lift',   # pitch trim gain x cos(pitch) at PITCH_TRIM_VREF_MPS
        'g_pitch_trim_thrust', # pitch trim gain x throttle at PITCH_TRIM_VREF_MPS, sea-level air
        'lua_eas2tas',         # FC: true airspeed per EAS, sqrt(sea-level / local density)
        'lua_plane_off_nose_deg',   # FC: balloon's angle off the nose, 0..180 (logged, not flown)
        'lua_plane_err_deg',        # FC: roll-to-plane, deg still to roll to 12 or 6 o'clock round the nose
        'lua_plane_side',           # FC: roll-to-plane's end, 1 = 12 o'clock, -1 = 6
        'lua_plane_axis_up',        # FC: world-vertical part of the unit turn axis, 1 = heading change (logged, not flown)
        'g_plane_side_hold_deg',    # 12/6 hold band, deg past the wingtip line
        'g_plane_gain',             # roll-to-plane's roll-rate gain, deg/s per deg
        'g_plane_ff_gain',          # roll-to-plane's feedforward on the clock drift, 1 = match it
        'g_plane_ff_lpf_rate',      # low-pass rate on that drift, 1/s
        'lua_plane_drift_dps',      # FC: balloon's filtered drift round the clock, deg/s
        'lua_plane_ff_dps',         # FC: roll-to-plane feedforward, deg/s of roll
        'g_plane_steep_start_deg',  # roll-to-plane starts taking a share, deg of flight-path angle
        'g_plane_steep_full_deg',   # the path share is whole, deg of flight-path angle
        'g_plane_ki',               # roll-to-plane integrator gain, 1/s
        'g_plane_windup',           # its ceiling, fraction of the ACRO roll rate
        'lua_plane_path_share',     # FC: roll-to-plane's share of the roll command, from the flight-path angle; bank-to-turn has 1 - this
        'lua_plane_i_dps',          # FC: roll-to-plane integrator, deg/s of roll
        'lua_plane_i_frozen',       # FC: 1 = integrator held, the command saturated its way
        'lua_plane_path_deg',       # FC: angle the path share read, deg, + up
        'lua_plane_path_from_vel',  # FC: 1 = from the velocity, 0 = the nose's pitch stood in
        'lua_excite_value',         # FC: EXCITE output, deg/s: added to its axis's rate command (k), or replacing it (i)
        'g_yaw_coordination_ff',    # scale on the turning and rolling yaw the yaw command carries, 1 = the geometry
        'lua_yaw_coordination_dps', # FC: the coordination the yaw command carried, deg/s, before the ACRO_YAW_RATE ceiling
        'g_curve_flyer_enable',     # Curve Flyer on (1) or off (0)
        'g_tilt_back_travel_deg',   # Curve Planner: how far the tilt may come back, deg
        'lua_cf_state',             # FC: Curve Flyer 0 off, 1 waiting for the PLAN, 2 flying the turn, 3 turn complete, 4 no PLAN in time, 6 line held, 7 run-in, 8 dive
        'lua_cf_plan_id',           # FC: plan_id of the PLAN it accepted this chase, 0 for none
        'lua_cf_progress',          # FC: progress through the turn, or the dive's in cf_state 8, 0-1
        'lua_cf_eps_deg',           # FC: the flight path's angle off what the Curve Flyer aims the law at (the line in cf_state 6, 7 and 2 with a run-in, else the balloon), deg
        'lua_cf_bank_ref_deg',      # FC: the bank the Curve Flyer gives the law, in its measure (the schedule's in cf_state 2, toward the balloon across the wings in 6-8), deg, + right wing down
        'cf_plan_id',               # Curve Planner: plan_id of the PLAN for the chase the FC waited on
        'cf_plan_eps0_deg',         # Curve Planner: angle off the balloon (or the run-in line) the plan's progress counts from, deg
        # Curve Planner: the plan's bank schedule, deg, + right wing down, in
        # the measure of the law it was made for (cf_plan_law), knot k at
        # progress (k-1)/(CURVE_N_KNOTS-1); one column per knot.
        'cf_plan_knot_1_deg', 'cf_plan_knot_2_deg', 'cf_plan_knot_3_deg',
        'cf_plan_knot_4_deg', 'cf_plan_knot_5_deg',
        'cf_plan_s',                # Curve Planner: seconds it took to make the plan
        'g_cf_transfer_when_done',  # Transfer Control When Done: 1 the law flies after the turn as without a PLAN, 0 it holds the turn's line
        'lua_cf_hold_n',            # FC: the line the Curve Flyer aims the law at (cf_state 6, 7 and 2 with a run-in), world unit vector, north
        'lua_cf_hold_e',            # FC: ... east
        'lua_cf_hold_d',            # FC: ... down
        'g_plane_kd',               # roll-to-plane's derivative gain, deg/s of roll per deg/s of the clock error's rate
        'g_plane_kd_rate',          # low-pass rate on that error rate, 1/s
        'lua_plane_err_rate_dps',   # FC: roll-to-plane, the clock error's filtered rate, deg/s
        'lua_plane_d_dps',          # FC: roll-to-plane derivative term, deg/s of roll
        'g_cf_plan_wait_s',         # Plan Wait: how long a fresh chase holds for its PLAN, s
        'lua_path_turn_up_dps',     # FC: flight path's turn rate toward the canopy since the previous tick, deg/s
        'lua_path_turn_right_dps',  # FC: flight path's turn rate toward the right wing since the previous tick, deg/s
        'g_cf_approach_deg',        # Curve Planner: Approach Angle, the dive below the horizon a chase's turn ends on, deg
        'cf_plan_runin_climb_deg',  # Curve Planner: the run-in line's angle above the horizon, deg; blank for no run-in
        'cf_plan_runin_dep_deg',    # Curve Planner: the balloon's angle below the horizon that ends the run-in, deg
        'g_steering_law',  # Steering Law: 0 the cascade, 1 Lift Vector Pointing
        'g_att_k_path',  # Lift Vector Pointing: path turn rate per rad off the balloon, 1/s
        'g_att_path_rate_max_dps',  # Lift Vector Pointing: cap on the path turn rate, deg/s
        'g_att_lead_s',  # Lift Vector Pointing: how far ahead the path is read, s
        'g_att_k_roll',  # Lift Vector Pointing: roll about the path per rad of lift-line error, 1/s
        'g_att_k_side',  # Lift Vector Pointing: weight on the turn's sideways part
        'g_att_roll_ki',  # Lift Vector Pointing: roll integrator gain, 1/s
        'g_att_pitch_ki',  # Lift Vector Pointing: pitch integrator gain, 1/s
        'g_att_i_zone_deg',  # Lift Vector Pointing: angle off the balloon inside which the integrators run, deg
        'lua_pitch_trim_lift_term',  # FC: the pitch trim's lift term this tick: cos(pitch) under the cascade, the lift load under Lift Vector Pointing
        'lua_att_roll_i_dps',  # FC: Lift Vector Pointing: roll integrator, deg/s of roll about the path
        'lua_att_pitch_i_dps',  # FC: Lift Vector Pointing: pitch integrator, deg/s of the path's turn
        'lua_att_wx_dps',  # FC: Lift Vector Pointing: roll about the path, deg/s
        'lua_att_wy_dps',  # FC: Lift Vector Pointing: the path's turn in the lift plane, deg/s
        'lua_att_wz_dps',  # FC: Lift Vector Pointing: the path frame's turn about z' under gravity, deg/s
        'lua_att_bank_deg',  # FC: Lift Vector Pointing: bank round the path it read, the canopy's angle from up, deg, + right wing down
        'lua_att_band_lo_deg',  # FC: Lift Vector Pointing: lowest bank_proxy_deg the chase allows, deg: the bank limit, or the starting proxy beyond it
        'lua_att_band_hi_deg',  # FC: Lift Vector Pointing: highest bank_proxy_deg the chase allows, deg: the bank limit, or the starting proxy beyond it
        'lua_att_drift_dps',  # FC: Lift Vector Pointing: the bank's drift the roll cancels while the room holds it, and under a PLAN, deg/s
        'lua_att_theta_now_deg',  # FC: Lift Vector Pointing: the path's angle off the balloon as read, deg
        'lua_att_p_dps',  # FC: Lift Vector Pointing: body roll rate it asks for, before the ACRO clip, deg/s
        'lua_att_q_dps',  # FC: Lift Vector Pointing: body pitch rate it asks for, before the ACRO clip, deg/s
        'lua_att_theta_deg',  # FC: Lift Vector Pointing: the path's angle off the balloon, read att_lead_s ahead, deg
        'lua_att_u_y',  # FC: Lift Vector Pointing: turn direction across the path, toward the right wing
        'lua_att_u_z',  # FC: Lift Vector Pointing: turn direction across the path, toward the belly
        'lua_att_omega_dps',  # FC: Lift Vector Pointing: turn rate the path is asked for, the guidance's slowed where the room stops a pull from making it, deg/s
        'lua_att_f_y_mps2',  # FC: Lift Vector Pointing: force line, push toward the right wing, m/s^2
        'lua_att_f_z_mps2',  # FC: Lift Vector Pointing: force line, push toward the belly, m/s^2
        'lua_att_mu_deg',  # FC: Lift Vector Pointing: roll still to go for the lift side's end of the lift line to face the force line (under a PLAN, to its bank), deg
        'lua_att_side',  # FC: Lift Vector Pointing: lift side, 1 canopy (pull), -1 belly (push); blank under a PLAN
        'lua_att_w_f',  # FC: Lift Vector Pointing: the roll's weight under a PLAN, its fade near vertical, 0-1; blank without one
        'lua_att_n_lift',  # FC: Lift Vector Pointing: load it asks of the wing, g
        'lua_accel_x',  # FC: body longitudinal specific force, m/s^2 (Lift Vector Pointing's lead reads all three axes)
        'lua_att_room_lo_deg',  # FC: Lift Vector Pointing: farthest roll left from the bank now before bank_proxy_deg leaves the band or the aircraft is upside down, deg; -180 unlimited
        'lua_att_room_hi_deg',  # FC: Lift Vector Pointing: farthest roll right from the bank now before bank_proxy_deg leaves the band or the aircraft is upside down, deg; 180 unlimited
        'cf_plan_law',              # Curve Planner: the steering law the plan was made for, 0 the cascade, 1 Lift Vector Pointing
        'lua_att_omega_ask_dps',  # FC: Lift Vector Pointing: turn rate the guidance asks the path for, before the room slows it, deg/s
    ]

    def __init__(self, path: str = '',
                 startup_buffer: 'StartupBufferHandler | None' = None):
        self._fixed_path = bool(path)   # '' = stamp a fresh name on every start
        self._path = path
        self._fh = None
        self._writer = None
        self._rows_since_flush = 0
        self._tick_count = 0
        self._start_time = None     # time.monotonic() at start(), for elapsed()
        self.header_notes = []      # optional '# note:' lines (e.g. cam resolutions)
        # Optional handle on the in-memory startup-log buffer.  drained into the
        # CSV header on the first start() call (the first 'q' press) so the
        # unique startup printouts (FAST/POLLED, readbacks, verifications)
        # travel with the flight data.  drain() is idempotent and returns []
        # after the first call - re-toggling 'q' does not duplicate the lines.
        self._startup_buffer = startup_buffer

    @property
    def active(self) -> bool:
        return self._fh is not None

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def path(self) -> str:
        """The file this session is writing, or will write.

        Empty until start() stamps a name, so a caller that logs it BEFORE
        start() gets '' rather than an attribute error.
        """
        return self._path

    def elapsed(self, now: float) -> float:
        """Seconds since start(); 0.0 if not started.  Used for the CSV 'time'
        column.  (Replaces the controller's old _csv_start bookkeeping.)"""
        return (now - self._start_time) if self._start_time is not None else 0.0

    def start(self) -> None:
        """Open the file, write the fingerprint line + header.  Resets counters."""
        # Stamped at START - the moment logging began, which is what a log
        # filename should answer.  The previous log goes now, so one file per
        # kind stays on disk rather than a pile.
        if not self._fixed_path:
            from . import file_export as _fx
            self._path = _fx.stamped('flight_log', 'csv', _fx.run_stamp())
        # The new log is OPEN before the previous one goes, so there is never a
        # moment with no log on disk.  Flight logging is not resumable - each 'q'
        # session is a new procedure with its own stamp - so the previous
        # session's file is replaced, not accumulated.  Copy it first to keep it.
        self._fh = open(self._path, 'w', newline='')
        from . import file_export as _fx
        for _old in _fx.claim('flight_log_*.csv', self._path):
            logging.getLogger("balloon_chaser").info(
                '[CSV] replaced the previous log %s - Copy it before starting a '
                'new session to keep it.', _old)
        # Source-file fingerprint at top of CSV.  Prefixed '# ' so csv.reader
        # / pandas(comment='#') can skip it.  Written before the csv.writer
        # touches the file to keep the format unambiguous.
        self._fh.write(_comment_lines("code_sha256", PACKAGE_SHA256))
        # Drain the startup-log buffer into the head of the CSV.  These are the
        # unique one-shot startup messages (FAST/POLLED telemetry path,
        # parameter readbacks, configuration verifications, etc.) captured by
        # the StartupBufferHandler installed in main().  drain() is idempotent
        # and closes the buffer permanently - subsequent log output goes only
        # to the terminal as before.  If the operator never enables logging,
        # the buffer is simply not drained and is GC'd on process exit.
        if self._startup_buffer is not None:
            for line in self._startup_buffer.drain():
                self._fh.write(_comment_lines("log", line))
        # Optional diagnostic notes (e.g. camera native resolutions) captured into
        # the log file so they travel with the data instead of only the console.
        for note in getattr(self, 'header_notes', []):
            self._fh.write(_comment_lines("note", note))
        # lineterminator="\n": the default \r\n leaves a stray CR on the last
        # field for a non-Python reader.
        self._writer = csv.writer(self._fh, lineterminator="\n")
        self._writer.writerow(self.HEADER)
        self._rows_since_flush = 0
        self._tick_count = 0
        import time as _t
        self._start_time = _t.monotonic()

    def flush(self) -> None:
        """Flush buffered rows WITHOUT closing.  Idempotent and safe to call
        from another thread: the file-export buttons call it so a copy of a
        live flight_log.csv contains every row written so far, instead of
        losing up to _CSV_FLUSH_EVERY rows (~0.5 s at 50 Hz) still sitting in
        the buffer.  No-op when logging is not running."""
        fh = self._fh
        if fh is not None:
            try:
                fh.flush()
                self._rows_since_flush = 0
            except (ValueError, OSError):
                pass          # closed underneath us - nothing to flush

    def stop(self) -> None:
        """Flush and close.  Idempotent."""
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            self._writer = None

    def write_tick(self, s: TickSnapshot) -> None:
        """Append one row; the value order matches HEADER exactly (test_csv_alignment checks it).

        Flush every ~0.5 s rather than every tick: per-tick flush() at 50 Hz x
        76 columns causes a measurable stall on slow disks.  A dropped tail of
        up to ~0.5 s on a crash is acceptable for diagnostic logging; the
        toggle-off and the run() finally block both flush explicitly via stop().
        """
        if self._writer is None:
            return

        inj = s.injected_world_pos
        inj_n = inj[0] if inj is not None else 0.0
        inj_e = inj[1] if inj is not None else 0.0
        inj_d = inj[2] if inj is not None else 0.0

        self._writer.writerow([
            self._tick_count,
            f"{s.t:.3f}",
            s.mode,
            s.event,
            # Outer-loop commands and actual state
            _deg(s.requested_roll, ".2f"),
            f"{math.degrees(s.act_roll):.2f}",
            _deg(s.requested_pitch, ".2f"),
            "" if s.log_req_pitch_raw is None else f"{math.degrees(s.log_req_pitch_raw):.2f}",
            f"{math.degrees(s.act_pitch):.2f}",
            # Airspeed->throttle loop
            _opt(s.log_asp_en, "d"),
            _opt(s.log_asp_err_mps, ".2f"),
            _opt(s.log_asp_i_term, ".2f"),
            _opt(s.log_asp_thr_pct, ".2f"),
            _opt(s.log_asp_sat, "d"),
            # Cascade decomposition
            _opt(s.log_p_term_roll, ".2f"),
            _opt(s.pan_integrator, ".4f"),
            _opt(s.pitch_integrator, ".4f"),
            # Body-frame errors
            _opt(s.steering_pan_deg, ".2f"),
            _opt(s.steering_tilt_deg, ".2f"),
            f"{s.roll_align_err:.2f}",
            "" if s.log_balloon_elev is None else f"{s.log_balloon_elev:.2f}",
            "" if s.log_aoa_lat is None else f"{s.log_aoa_lat:.2f}",
            # Vision diagnostics
            f"{s.log_det_norm_x:.4f}",
            f"{s.log_det_norm_y:.4f}",
            _opt(s.world_est_trust, ".4f"),
            # ptest diagnostics - aircraft state used for projection
            f"{s.aircraft_lat:.7f}",
            f"{s.aircraft_lon:.7f}",
            f"{s.aircraft_relative_alt:.2f}",
            # ptest diagnostics - injected balloon world position (zero if none)
            f"{inj_n:.2f}",
            f"{inj_e:.2f}",
            f"{inj_d:.2f}",
            # Gimbal
            f"{s.gimbal_bearing_pan:.2f}",
            f"{s.gimbal_bearing_tilt:.2f}",
            f"{s.gimbal_meas_pan:.2f}",
            f"{s.gimbal_meas_tilt:.2f}",
            f"{s.gimbal_quat_x:.6f}", f"{s.gimbal_quat_y:.6f}",
            f"{s.gimbal_quat_z:.6f}", f"{s.gimbal_quat_w:.6f}",
            f"{s.dbg_nose_px}", f"{s.dbg_nose_py}",
            f"{s.dbg_det_px}", f"{s.dbg_det_py}",
            f"{s.dbg_balloon_body_x:.5f}", f"{s.dbg_balloon_body_y:.5f}",
            f"{s.dbg_balloon_body_z:.5f}",
            f"{s.dbg_control_body_x:.5f}", f"{s.dbg_control_body_y:.5f}",
            f"{s.dbg_control_body_z:.5f}",
            # pitch-debug instrumentation
            "" if s.dbg_aim_body_x is None else f"{s.dbg_aim_body_x:.5f}",
            "" if s.dbg_aim_body_y is None else f"{s.dbg_aim_body_y:.5f}",
            "" if s.dbg_aim_body_z is None else f"{s.dbg_aim_body_z:.5f}",
            _opt(s.dbg_theta_ip_deg, ".3f"),
            "" if s.dbg_theta_ip_body_deg is None else f"{s.dbg_theta_ip_body_deg:.3f}",
            "" if s.los_elev_deg is None else f"{s.los_elev_deg:.3f}",
            _opt(s.vel_n, ".3f"), _opt(s.vel_e, ".3f"), _opt(s.vel_d, ".3f"),
            # Aircraft telemetry
            f"{s.aircraft_airspeed:.1f}",
            f"{math.degrees(s.act_yaw):.2f}",
            f"{math.degrees(s.roll_rate):.3f}",
            f"{math.degrees(s.pitch_rate):.3f}",
            f"{math.degrees(s.yaw_rate):.3f}",
            f"{s.aircraft_quat_w:.6f}", f"{s.aircraft_quat_x:.6f}",
            f"{s.aircraft_quat_y:.6f}", f"{s.aircraft_quat_z:.6f}",
            f"{s.true_bank_deg:.2f}", f"{s.true_pitch_deg:.2f}", f"{s.quat_fresh}",
            f"{s.control_dt:.5f}",
            # Servo + PWM
            f"{s.servo_aileron}",
            f"{s.servo_elevator}",
            f"{s.servo_ch3}",
            f"{s.servo_rudder}",
            f"{s.servo_ch5}",
            f"{s.servo_ch6}",
            f"{s.servo_ch7}",
            f"{s.servo_ch8}",
            "" if s.last_pwm_ch1 is None else f"{s.last_pwm_ch1}",
            "" if s.last_pwm_ch2 is None else f"{s.last_pwm_ch2}",
            "" if s.log_rate_cmd_pitch is None else f"{s.log_rate_cmd_pitch:.3f}",
            "" if s.log_rate_cmd_roll is None else f"{s.log_rate_cmd_roll:.3f}",
            _opt(s.lua_omega_x, ".3f"),
            _opt(s.lua_omega_y, ".3f"),
            _opt(s.lua_omega_x_presat, ".3f"),
            _opt(s.lua_omega_y_presat, ".3f"),
            _opt(s.lua_roll_cmd_presat, ".3f"),
            _opt(s.lua_requested_roll, ".5f"),
            _opt(s.lua_requested_pitch, ".5f"),
            f"{s.lua_bank_lim_deg:.2f}",
            _opt(s.lua_roll_err_deg, ".3f"),
            _opt(s.lua_throttle, ".2f"),
            f"{s.lua_fc_millis}",
            _opt(s.lua_cs_flags, "d"),
            _opt(s.lua_accel_y, ".4f"),
            _opt(s.lua_accel_z, ".4f"),
            _opt(s.lua_yaw_ff_coord, ".3f"),
            _opt(s.lua_yaw_ff_ari, ".3f"),
            _opt(s.lua_lat_marker, ".4f"),
            _opt(s.lua_pan_err_deg, ".4f"),
            _opt(s.lua_yaw_rate_cmd_dps, ".4f"),
            _opt(s.lua_yaw_point_dps, ".4f"),
            _opt(s.lua_pitch_turn_dps, ".4f"),
            _opt(s.fc_aoa, ".3f"),
            _opt(s.fc_ssa, ".3f"),
            f"{s.lua_gate}",
            _opt(s.lua_trust, ".4f"),
            f"{s.lua_dt_ms}",
            f"{s.lua_frames}",
            _opt(s.lua_scaler, ".4f"),
            f"{s.lua_seq}",
            f"{s.lua_seq_lag}",
            f"{s.servo1_trim:.0f}", f"{s.servo1_min:.0f}", f"{s.servo1_max:.0f}",
            f"{s.servo2_trim:.0f}", f"{s.servo2_min:.0f}", f"{s.servo2_max:.0f}",
            f"{s.servo4_trim:.0f}", f"{s.servo4_min:.0f}", f"{s.servo4_max:.0f}",
            f"{s.elevon_mixing}",
            f"{s.hz_quat}", f"{s.hz_gyro}", f"{s.hz_vel}", f"{s.hz_asp}",
            f"{s.gimbal_age_ms}", f"{s.rtf:.4f}", f"{s.gimbal_sim_age_ms}",
            # Tunable gains snapshot
            f"{s.g_shallow_pan_p_gain:.4f}",
            f"{s.g_shallow_pan_ki:.4f}",
            f"{s.g_shallow_windup_limit_pan:.4f}",
            f"{s.g_pitch_ki:.4f}",
            f"{s.g_windup_limit_pitch:.4f}",
            f"{s.g_shallow_roll_gain:.4f}",
            f"{s.g_pitch_gain:.4f}",
            f"{s.g_pitch_turn_ff:.4f}",
            f"{s.g_yaw_point_kp:.4f}",
            f"{s.g_yaw_point_ki:.4f}",
            f"{s.g_yaw_point_lim:.4f}",
            f"{s.g_yaw_point_windup:.4f}",
            f"{s.g_throttle_pct:.2f}",
            f"{int(s.fc_mode)}",
            _opt(s.fc_roll_des, ".3f"), _opt(s.fc_roll_ach, ".3f"), _opt(s.fc_roll_ff, ".4f"),
            _opt(s.fc_roll_p, ".4f"), _opt(s.fc_roll_i, ".4f"), _opt(s.fc_roll_d, ".4f"),
            _opt(s.fc_pitch_des, ".3f"), _opt(s.fc_pitch_ach, ".3f"), _opt(s.fc_pitch_ff, ".4f"),
            _opt(s.fc_pitch_p, ".4f"), _opt(s.fc_pitch_i, ".4f"), _opt(s.fc_pitch_d, ".4f"),
            _opt(s.fc_yaw_des, ".3f"), _opt(s.fc_yaw_ach, ".3f"), _opt(s.fc_yaw_ff, ".4f"),
            _opt(s.fc_yaw_p, ".4f"), _opt(s.fc_yaw_i, ".4f"), _opt(s.fc_yaw_d, ".4f"),
            _opt(s.log_roll_cmd_presat, ".4f"),
            _opt(s.log_roll_sat, "d"),
            _opt(s.log_d_term_roll, ".4f"),
            _opt(s.log_d_term_pitch, ".4f"),
            _opt(s.log_pan_i_frozen, "d"),
            _opt(s.log_pan_i_clamped, "d"),
            _opt(s.log_pitch_i_frozen, "d"),
            _opt(s.log_pitch_i_clamped, "d"),
            f"{int(s.log_in_tol)}",
            f"{s.log_align_tol_deg:.4f}",
            f"{s.g_shallow_bank_lim:.4f}",
            f"{s.g_shallow_pan_kd:.5f}",
            f"{s.g_pitch_kd:.5f}",
            "" if s.balloon_range_m is None else f"{s.balloon_range_m:.3f}",
            "" if s.pass_distance_m is None else f"{s.pass_distance_m:.4f}",
            _opt(s.bank_proxy_deg, ".3f"),
            _opt(s.roll_err_proxy_deg, ".3f"),
            "" if s.bank_rate_cmd_dps is None else f"{s.bank_rate_cmd_dps:.3f}",
            "" if s.bank_rate_coupling_dps is None else f"{s.bank_rate_coupling_dps:.3f}",
            _opt(s.upright, ".4f"),
            f"{s.acro_roll_rate_lim:.2f}",
            "" if s.align_pan_deg is None else f"{s.align_pan_deg:.3f}",
            "" if s.align_tilt_deg is None else f"{s.align_tilt_deg:.3f}",
            "" if s.sideslip_deg is None else f"{s.sideslip_deg:.3f}",
            "" if s.aoa_deg is None else f"{s.aoa_deg:.3f}",
            "" if s.sim_roll_deg is None else f"{s.sim_roll_deg:.3f}",
            "" if s.sim_pitch_deg is None else f"{s.sim_pitch_deg:.3f}",
            "" if s.sim_p_dps is None else f"{s.sim_p_dps:.3f}",
            "" if s.sim_r_dps is None else f"{s.sim_r_dps:.3f}",
            # Background FOV estimator (diagnostic; 0 unless the switch is on).
            f"{s.fov_est_deg:.3f}",
            f"{s.fov_est_m1_deg:.3f}",
            f"{s.fov_est_m2_deg:.3f}",
            f"{s.fov_truth_deg:.3f}",
            f"{int(s.fov_est_n)}",
            f"{s.g_pitch_deadband_deg:.6g}",
            f"{s.g_pitch_kd_rate:.6g}",
            f"{int(s.g_airspeed_cmd_enable)}",
            f"{s.g_airspeed_cmd_mps:.6g}",
            f"{s.g_airspeed_cmd_kp:.6g}",
            f"{s.g_airspeed_cmd_ki:.6g}",
            f"{s.g_airspeed_cmd_min_valid:.6g}",
            f"{s.g_pan_kd_rate:.6g}",
            f"{s.g_aileron_override:.6g}",
            f"{s.g_aileron_deflection_pct:.6g}",
            f"{s.g_elevator_override:.6g}",
            f"{s.g_elevator_deflection_pct:.6g}",
            f"{s.g_rudder_override:.6g}",
            f"{s.g_rudder_deflection_pct:.6g}",
            f"{int(s.g_yaw_point_pitch_sched)}",
            "" if s.fc_param_ptch2srv_tconst is None else f"{s.fc_param_ptch2srv_tconst:.6g}",
            "" if s.fc_param_ptch2srv_rmax_up is None else f"{s.fc_param_ptch2srv_rmax_up:.6g}",
            "" if s.fc_param_ptch2srv_rmax_dn is None else f"{s.fc_param_ptch2srv_rmax_dn:.6g}",
            "" if s.fc_param_ptch_rate_ff is None else f"{s.fc_param_ptch_rate_ff:.6g}",
            "" if s.fc_param_ptch_rate_p is None else f"{s.fc_param_ptch_rate_p:.6g}",
            "" if s.fc_param_ptch_rate_i is None else f"{s.fc_param_ptch_rate_i:.6g}",
            "" if s.fc_param_ptch_rate_d is None else f"{s.fc_param_ptch_rate_d:.6g}",
            "" if s.fc_param_ptch_rate_imax is None else f"{s.fc_param_ptch_rate_imax:.6g}",
            "" if s.fc_param_ptch_rate_fltt is None else f"{s.fc_param_ptch_rate_fltt:.6g}",
            "" if s.fc_param_ptch_rate_flte is None else f"{s.fc_param_ptch_rate_flte:.6g}",
            "" if s.fc_param_ptch_rate_fltd is None else f"{s.fc_param_ptch_rate_fltd:.6g}",
            "" if s.fc_param_ptch_rate_smax is None else f"{s.fc_param_ptch_rate_smax:.6g}",
            "" if s.fc_param_rll2srv_tconst is None else f"{s.fc_param_rll2srv_tconst:.6g}",
            "" if s.fc_param_rll2srv_rmax is None else f"{s.fc_param_rll2srv_rmax:.6g}",
            "" if s.fc_param_rll_rate_ff is None else f"{s.fc_param_rll_rate_ff:.6g}",
            "" if s.fc_param_rll_rate_p is None else f"{s.fc_param_rll_rate_p:.6g}",
            "" if s.fc_param_rll_rate_i is None else f"{s.fc_param_rll_rate_i:.6g}",
            "" if s.fc_param_rll_rate_d is None else f"{s.fc_param_rll_rate_d:.6g}",
            "" if s.fc_param_rll_rate_imax is None else f"{s.fc_param_rll_rate_imax:.6g}",
            "" if s.fc_param_rll_rate_fltt is None else f"{s.fc_param_rll_rate_fltt:.6g}",
            "" if s.fc_param_rll_rate_flte is None else f"{s.fc_param_rll_rate_flte:.6g}",
            "" if s.fc_param_rll_rate_fltd is None else f"{s.fc_param_rll_rate_fltd:.6g}",
            "" if s.fc_param_rll_rate_smax is None else f"{s.fc_param_rll_rate_smax:.6g}",
            "" if s.fc_param_yaw_rate_ff is None else f"{s.fc_param_yaw_rate_ff:.6g}",
            "" if s.fc_param_yaw_rate_p is None else f"{s.fc_param_yaw_rate_p:.6g}",
            "" if s.fc_param_yaw_rate_i is None else f"{s.fc_param_yaw_rate_i:.6g}",
            "" if s.fc_param_yaw_rate_d is None else f"{s.fc_param_yaw_rate_d:.6g}",
            "" if s.fc_param_yaw_rate_imax is None else f"{s.fc_param_yaw_rate_imax:.6g}",
            "" if s.fc_param_yaw_rate_fltt is None else f"{s.fc_param_yaw_rate_fltt:.6g}",
            "" if s.fc_param_yaw_rate_flte is None else f"{s.fc_param_yaw_rate_flte:.6g}",
            "" if s.fc_param_yaw_rate_fltd is None else f"{s.fc_param_yaw_rate_fltd:.6g}",
            "" if s.fc_param_yaw_rate_smax is None else f"{s.fc_param_yaw_rate_smax:.6g}",
            _opt(s.lua_vel_marker, ".4f"),
            f"{s.g_pitch_trim_offset:.4f}",
            f"{s.g_pitch_trim_attitude:.4f}",
            _opt(s.lua_pitch_trim_elev, ".4f"),
            _opt(s.lua_pitch_trim_dps, ".4f"),
            _opt(s.lua_state2_seq, "d"), _opt(s.lua_fc_mode, "d"),
            _opt(s.lua_q_w, ".6f"), _opt(s.lua_q_x, ".6f"),
            _opt(s.lua_q_y, ".6f"), _opt(s.lua_q_z, ".6f"),
            _opt(s.lua_gyro_x_rads, ".5f"), _opt(s.lua_gyro_y_rads, ".5f"),
            _opt(s.lua_gyro_z_rads, ".5f"),
            _opt(s.lua_vel_n_mps, ".3f"), _opt(s.lua_vel_e_mps, ".3f"),
            _opt(s.lua_vel_d_mps, ".3f"), _opt(s.lua_eas_mps, ".3f"),
            _opt(s.lua_balloon_body_x, ".5f"), _opt(s.lua_balloon_body_y, ".5f"),
            _opt(s.lua_balloon_body_z, ".5f"),
            _opt(s.lua_yaw_point_i_dps, ".4f"),
            _opt(s.lua_coord_valid, "d"), _opt(s.lua_accel_valid, "d"),
            _opt(s.lua_lat_marker_valid, "d"),
            _opt(s.lua_out_aileron, ".2f"), _opt(s.lua_out_elevator, ".2f"),
            _opt(s.lua_out_rudder, ".2f"), _opt(s.lua_out_throttle, ".2f"),
            _opt(s.lua_pwm_aileron, "d"), _opt(s.lua_pwm_vtail_l, "d"),
            _opt(s.lua_pwm_vtail_r, "d"),
            _opt(s.lua_est_valid, "d"), _opt(s.lua_excite_running, "d"),
            _opt(s.lua_nav_active, "d"),
            f"{s.g_pitch_trim_lift:.4f}", f"{s.g_pitch_trim_thrust:.4f}",
            _opt(s.lua_eas2tas, ".5f"),
            _opt(s.lua_plane_off_nose_deg, ".3f"), _opt(s.lua_plane_err_deg, ".3f"),
            _opt(s.lua_plane_side, ".0f"), _opt(s.lua_plane_axis_up, ".4f"),
            f"{s.g_plane_side_hold_deg:.2f}", f"{s.g_plane_gain:.4f}",
            f"{s.g_plane_ff_gain:.4f}", f"{s.g_plane_ff_lpf_rate:.4f}",
            _opt(s.lua_plane_drift_dps, ".3f"), _opt(s.lua_plane_ff_dps, ".3f"),
            f"{s.g_plane_steep_start_deg:.2f}", f"{s.g_plane_steep_full_deg:.2f}",
            f"{s.g_plane_ki:.4f}", f"{s.g_plane_windup:.4f}",
            _opt(s.lua_plane_path_share, ".4f"), _opt(s.lua_plane_i_dps, ".3f"),
            _opt(s.lua_plane_i_frozen, ".0f"),
            _opt(s.lua_plane_path_deg, ".3f"), _opt(s.lua_plane_path_from_vel, ".0f"),
            _opt(s.lua_excite_value, ".4f"),
            f"{s.g_yaw_coordination_ff:.4f}",
            _opt(s.lua_yaw_coordination_dps, ".4f"),
            str(int(s.g_curve_flyer_enable)),
            f"{s.g_tilt_back_travel_deg:.4f}",
            _opt(s.lua_cf_state, ".0f"), _opt(s.lua_cf_plan_id, ".0f"),
            _opt(s.lua_cf_progress, ".4f"), _opt(s.lua_cf_eps_deg, ".3f"),
            _opt(s.lua_cf_bank_ref_deg, ".3f"),
            _opt(s.cf_plan_id, ".0f"), _opt(s.cf_plan_eps0_deg, ".3f"),
            _opt(_knot(s.cf_plan_knots_deg, 0), ".3f"), _opt(_knot(s.cf_plan_knots_deg, 1), ".3f"),
            _opt(_knot(s.cf_plan_knots_deg, 2), ".3f"), _opt(_knot(s.cf_plan_knots_deg, 3), ".3f"),
            _opt(_knot(s.cf_plan_knots_deg, 4), ".3f"),
            _opt(s.cf_plan_s, ".3f"),
            str(int(s.g_cf_transfer_when_done)),
            _opt(s.lua_cf_hold_n, ".5f"), _opt(s.lua_cf_hold_e, ".5f"),
            _opt(s.lua_cf_hold_d, ".5f"),
            f"{s.g_plane_kd:.4f}", f"{s.g_plane_kd_rate:.4f}",
            _opt(s.lua_plane_err_rate_dps, ".3f"), _opt(s.lua_plane_d_dps, ".3f"),
            f"{s.g_cf_plan_wait_s:.3f}",
            _opt(s.lua_path_turn_up_dps, ".3f"), _opt(s.lua_path_turn_right_dps, ".3f"),
            f"{s.g_cf_approach_deg:.3f}",
            _opt(s.cf_plan_runin_climb_deg, ".3f"), _opt(s.cf_plan_runin_dep_deg, ".3f"),
            str(int(s.g_steering_law)),
            f"{s.g_att_k_path:.4f}",
            f"{s.g_att_path_rate_max_dps:.4f}",
            f"{s.g_att_lead_s:.4f}",
            f"{s.g_att_k_roll:.4f}",
            f"{s.g_att_k_side:.4f}",
            f"{s.g_att_roll_ki:.4f}",
            f"{s.g_att_pitch_ki:.4f}",
            f"{s.g_att_i_zone_deg:.4f}",
            _opt(s.lua_pitch_trim_lift_term, ".5f"),
            _opt(s.lua_att_roll_i_dps, ".3f"),
            _opt(s.lua_att_pitch_i_dps, ".3f"),
            _opt(s.lua_att_wx_dps, ".3f"),
            _opt(s.lua_att_wy_dps, ".3f"),
            _opt(s.lua_att_wz_dps, ".3f"),
            _opt(s.lua_att_bank_deg, ".3f"),
            _opt(s.lua_att_band_lo_deg, ".3f"),
            _opt(s.lua_att_band_hi_deg, ".3f"),
            _opt(s.lua_att_drift_dps, ".3f"),
            _opt(s.lua_att_theta_now_deg, ".4f"),
            _opt(s.lua_att_p_dps, ".3f"),
            _opt(s.lua_att_q_dps, ".3f"),
            _opt(s.lua_att_theta_deg, ".4f"),
            _opt(s.lua_att_u_y, ".5f"),
            _opt(s.lua_att_u_z, ".5f"),
            _opt(s.lua_att_omega_dps, ".3f"),
            _opt(s.lua_att_f_y_mps2, ".4f"),
            _opt(s.lua_att_f_z_mps2, ".4f"),
            _opt(s.lua_att_mu_deg, ".3f"),
            _opt(s.lua_att_side, ".0f"),
            _opt(s.lua_att_w_f, ".4f"),
            _opt(s.lua_att_n_lift, ".4f"),
            _opt(s.lua_accel_x, ".4f"),
            _opt(s.lua_att_room_lo_deg, ".3f"),
            _opt(s.lua_att_room_hi_deg, ".3f"),
            _opt(s.cf_plan_law, ".0f"),
            _opt(s.lua_att_omega_ask_dps, ".3f"),
        ])

        self._rows_since_flush += 1
        self._tick_count += 1
        if self._rows_since_flush >= _CSV_FLUSH_EVERY:
            self._fh.flush()
            self._rows_since_flush = 0
