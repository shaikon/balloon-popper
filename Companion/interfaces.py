# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Sim-to-real seam.

Four Protocols that the controller talks to.  Each has a sim implementation
(adapters/sim_gazebo.py) and will have a real implementation
(adapters/real_hardware.py).

Design intent: BalloonChaser imports ONLY this module's abstractions.
Swapping sim for real — or for a test oracle, or a recorded-log replay — is
a matter of which adapter classes main.py constructs.  No `if sim: ... else:`
branches in the controller.

────────────────────────────────────────────────────────────────────────────
SYSTEM VIEW — why BalloonSensor is the spine of the whole thing
────────────────────────────────────────────────────────────────────────────
Today's `x` (live HSV vision) and `p` (synthetic injected balloon) modes look
like two different features.  They are not.  They are two SOURCES of the same
thing: a world-frame balloon direction with a trust value, which the cascade
chases.  The cascade does not care where that direction came from.

On the real airframe, the source becomes a coordinate streamed from a
companion computer.  At that point `x` and `p` differ only in which message
the sensor is subscribed to (live detections vs. a test anchor) — and that
distinction may vanish entirely.

So we make ONE seam: BalloonSensor.  Everything that can produce a balloon
implements it:

    HsvDetectionSensor    sim   — Gazebo gimbal frame + HSV thresholding
    SyntheticOracleSensor test  — anchored world balloon, bypasses the filter
                                   (the 'p' / ptest path)
    CompanionSensor       real  — detection feed from the companion computer
    LogPlaybackSensor     test  — replay a recorded session for regression

The controller's notion of "the balloon" is exactly `estimator.get()`, fed by
whichever sensor is mounted.  It never branches on sensor type.
────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations
import numpy as np
from typing import Protocol, Optional, Tuple, runtime_checkable


# -----------------------------------------------------------------------------
# AutopilotLink - what the controller needs from the flight controller.
# -----------------------------------------------------------------------------
#
# Today fulfilled by MAVLink/ArduPlane.  Owns the connection, the telemetry
# thread (sole reader), the parameter-handshake startup quirks, and the flight
# sequence (mission upload -> arm -> catapult-launch -> AUTO -> loiter).
#
# Attributes update continuously from telemetry; methods block unless noted.
@runtime_checkable
class AutopilotLink(Protocol):
    # -- Telemetry (updated by background thread) --
    attitude: np.ndarray            # (roll, pitch, yaw) radians
    rates: np.ndarray               # (rollspeed, pitchspeed, yawspeed) rad/s
    lat: float                      # degrees
    lon: float                      # degrees
    relative_alt: float             # metres AGL
    # Validity time of the current position fix, seconds since FC boot (None
    # before the first fix).  lat/lon/relative_alt refresh an order of magnitude
    # slower than the control loop reads them, so a consumer that must know
    # WHICH fix it holds - or when that fix was actually true - reads this
    # rather than the wall clock.  Optional: consumers fall back to comparing
    # the position itself when a link does not supply it.
    pos_time_boot_s: Optional[float]
    airspeed: float                 # m/s
    groundspeed: float              # m/s
    armed: bool
    prearm_healthy: bool
    current_mode: int               # ArduPilot custom_mode (4=ACRO, 12=LOITER, ...)
    home_lat: Optional[float]
    home_lon: Optional[float]

    # -- FC configuration (set by read_fc_config) --
    pitch_lim_up: float             # PTCH_LIM_MAX_DEG, degrees positive
    pitch_lim_dn: float             # PTCH_LIM_MIN_DEG, degrees negative
    acro_roll_rate: float           # ACRO_ROLL_RATE, deg/s
    acro_pitch_rate: float          # ACRO_PITCH_RATE, deg/s

    # -- Servo telemetry (for HUD) --
    servo_aileron: int              # CH1 raw PWM
    servo_elevator: int             # CH2 raw PWM
    servo_ch3: int
    servo_ch4: int
    servo_ch5: int
    servo_ch6: int
    servo_ch7: int
    servo_ch8: int

    # -- SERVO trim & range (for HUD normalisation) --
    servo1_min: float
    servo1_trim: float
    servo1_max: float               # SERVO1_*
    servo2_min: float
    servo2_trim: float
    servo2_max: float               # SERVO2_*

    # -- Helper: aircraft world position in NED, or None if no GPS lock --
    def aircraft_world_pos_ned(self) -> Optional[np.ndarray]:
        """[north, east, down] metres relative to home, or None.  Used by the
        sensor's oracle path (ground-anchored balloon) and by diagnostics."""
        ...

    # -- Lifecycle --
    def start_telemetry(self) -> None:
        """Start the telemetry thread.  Idempotent."""
        ...

    def request_high_rate_streams(self) -> None:
        """Ask the FC for ATTITUDE/VFR_HUD/SERVO_OUTPUT_RAW at 50 Hz."""
        ...

    def read_fc_config(self) -> None:
        """Pull envelope params, servo ranges, ACRO rates from the FC and CHECK the
        saved parameters this code depends on.  Writes none of them: any
        difference is listed and startup halts for the operator's Enter."""
        ...

    def send_gcs_heartbeat(self) -> None:
        """Sysid handshake - must be sent >=1 Hz or RC_CHANNELS_OVERRIDE
        gets silently dropped on ArduPlane."""
        ...

    # -- Pre-flight --
    def wait_until_prearm_ok(self) -> None: ...
    def arm(self) -> None: ...
    def upload_mission(self, lat: float, lon: float, alt: float) -> None: ...

    # -- Flight sequence --
    def catapult_launch(self, goal_alt: float, final_mode=(10, 'AUTO')) -> None:
        """Blocks until goal_alt reached.  Uses FBWA + raw RC override.

        The name is the SIMULATION mechanism, not the field one.  The real Talon
        is HAND LAUNCHED; this FBWA climb is the closest thing SITL can do to
        that.  Only the getting-airborne phase differs - see the implementation
        in mavlink_link.py for what the approximation does and does not
        reproduce.
        """
        ...
    def fly_to_waypoint(self, lat: float, lon: float, threshold_m: float = 50) -> None: ...
    def start_loiter(self) -> None: ...

    # -- Tracking --
    def set_mode_acro(self, hold=None) -> None:
        """Switch to ACRO and wait for HEARTBEAT confirmation.  `hold`, when
        given, re-sends the caller's stick overrides once ACRO is confirmed."""
        ...
    def set_mode_loiter(self, keep_operator_params: bool = False) -> None: ...

    def hold_level_sticks(self, throttle_frac: float) -> None:
        """One RC override: roll, pitch and rudder centred, throttle at
        throttle_frac (0..1)."""
        ...

    def release_rc_overrides(self) -> None:
        """Send all-zero RC override so the FC regains stick authority."""
        ...

    # -- Shutdown --
    def shutdown(self) -> None:
        """End-of-run hook.  Changes no FC parameter.  Idempotent."""
        ...


# -----------------------------------------------------------------------------
# GimbalDriver - what the controller / sensor needs from the gimbal hardware.
# -----------------------------------------------------------------------------
#
# The sensor commands desired pan/tilt (visual servo) and reads back the
# gimbal's angle.  The trust formula uses the gimbal's slew rate as a physical
# bound, so the driver exposes it.
#
# Sim: Gazebo joint controllers don't return measured angle; measured_* falls
# back to commanded.  has_readback=False.
# Real: modern gimbals expose measured angle (GIMBAL_DEVICE_ATTITUDE_STATUS).
# When has_readback=True, the alpha_C trust factor collapses (the read-back
# angle IS the truth - no commanded-vs-actual divergence to penalise).
class GimbalDriver(Protocol):
    @property
    def measured_pan_deg(self) -> float: ...
    @property
    def measured_tilt_deg(self) -> float: ...
    @property
    def commanded_pan_deg(self) -> float: ...
    @property
    def commanded_tilt_deg(self) -> float: ...
    @property
    def slew_rate_dps(self) -> float:
        """Used by the trust formula's alpha_C factor."""
        ...
    @property
    def has_readback(self) -> bool:
        """True iff measured_* reflects sensor feedback (not commanded angle).
        Sim drivers: False.  Real drivers with angle telemetry: True."""
        ...

    def command(self, pan_deg: float, tilt_deg: float) -> None:
        """Set desired gimbal pointing.  Clipped to +-pan_max/tilt_max internally."""
        ...

    def reset(self) -> None:
        """Slew to (0, 0)."""
        ...


# -----------------------------------------------------------------------------
# BalloonSensor - the spine.  Single source of "where is the balloon?".
# -----------------------------------------------------------------------------
#
# Two paths, one output.
#
#   LIVE PATH:    update_per_tick() pumps the sensor (pull a frame / drain a
#                 socket / advance a playback cursor) and may drive the gimbal
#                 as a side effect.  measurement() then returns the body-frame
#                 bearing + trust, which the controller folds into the
#                 WorldBalloonEstimator (a filter).
#
#   ORACLE PATH:  set_oracle_world_{position,direction}() anchors an exact
#                 world-frame balloon for cascade validation (the ptest).
#                 While the oracle is active, measurement() returns the oracle
#                 projected into the CURRENT body frame with trust=1.0 - so the
#                 controller can choose to snap the estimator directly to it,
#                 bypassing the filter.  oracle_active tells the controller to
#                 do that snap rather than the normal filtered update.
#
# The cascade is identical in both paths: it tracks estimator.get().  The only
# difference is whether the estimator was fed a filtered live measurement or
# snapped to an oracle.  This is the whole point of the restructure: 'x' and
# 'p' are the SAME control path with different sensor configuration.
#
# update_per_tick takes aircraft state as ARGUMENTS rather than the sensor
# holding a link reference.  This keeps the sensor's dependency on "the world"
# explicit and lets sensors with no autopilot at all (LogPlaybackSensor) exist.
class BalloonSensor(Protocol):
    # -- Live measurement path --
    def update_per_tick(self,
                        dt: float,
                        aircraft_att: np.ndarray,
                        aircraft_pos_ned: Optional[np.ndarray],
                        aircraft_rates: np.ndarray) -> None:
        """Pump the sensor for one tick.  Side effects allowed (gimbal slew).

        Args:
            dt:                tick duration (s)
            aircraft_att:      (roll, pitch, yaw) radians - for body<->world lift
            aircraft_pos_ned:  [N, E, D] metres, or None if no GPS - for the
                               ground-anchored oracle's bearing computation
            aircraft_rates:    (p, q, r) rad/s - for the trust formula's alpha_C
        """
        ...

    def measurement(self) -> Optional[Tuple[np.ndarray, float]]:
        """Returns (body_frame_unit_vec, trust in [0,1]) or None.

        None  -> no valid measurement this tick; estimator holds state.
        trust -> 0 means "detection present but known-unreliable".

        When oracle_active is True this returns the oracle projected into the
        current body frame with trust=1.0 (see oracle path above).
        """
        ...

    def detection_pixel_for_hud(self) -> Optional[Tuple[int, int]]:
        """Last detection pixel in the gimbal frame, or None.  HUD only."""
        ...

    def detection_norm(self) -> Tuple[float, float]:
        """Normalised detection offset from frame centre, each in [-1, 1], as
        (norm_x, norm_y).  (0.0, 0.0) when there is no detection or the gimbal
        is frozen.  Logged to CSV (det_norm_x/y) and is the alpha_A input the
        controller passes to world_est_trust on the live path."""
        ...

    # -- Oracle path (cascade-validation test; the 'p' / ptest) --
    def set_oracle_world_position(self, pos_ned: np.ndarray) -> None:
        """Anchor a fixed world-NED point.  Bearing changes as aircraft moves
        (balloon chaser dynamics).  Used when a test ray hits the ground."""
        ...
    def set_oracle_position_fn(self, fn) -> None:
        """PULL form of the above: hand over callable() -> world-NED position.

        Evaluated ONCE per control tick (top of update_per_tick) and cached, so
        every consumer within a tick sees the same position and that position is
        current as of that tick.  Preferred for a MOVING oracle balloon, whose
        producer runs on its own thread: a pushed snapshot beats against the
        control loop and goes stale on a third of ticks.  Optional - a sensor
        without it still works, and producers fall back to pushing."""
        ...
    def set_oracle_world_direction(self, dir_unit: np.ndarray) -> None:
        """Anchor a fixed world-frame direction (balloon at infinity / sky case).
        Bearing independent of aircraft position."""
        ...
    def clear_oracle(self) -> None: ...
    @property
    def oracle_active(self) -> bool: ...
    def oracle_world_dir(self, aircraft_pos_ned: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """Current world-frame unit direction to the oracle balloon, or None if
        no oracle.  For ground-anchored oracles, computed from aircraft_pos_ned;
        for sky oracles, the fixed direction.  Used by the controller to snap
        the estimator and by the HUD to project the chevron."""
        ...

    # -- Activation knob --
    def set_active(self, enabled: bool, manual_search: bool = False) -> None:
        """Enable/disable the sensor.  When disabled, measurement() returns
        None and the gimbal is not driven by detections.

        manual_search (the 'l' vision tracking) arms an operator-aim search: while
        enabled but before the first detection the gimbal follows the operator's
        mouse, and the first detection then latches vision ownership for the rest
        of the lock.  Left False (balloon chaser, ptest), the sensor drives the gimbal
        from vision immediately with no operator-aim phase.  Sensors that never
        slew (oracle) ignore it."""
        ...
    @property
    def active(self) -> bool: ...

    # -- Gimbal lock ('g' key): freeze the slew, INDEPENDENT of active --
    @property
    def gimbal_locked(self) -> bool: ...
    def set_gimbal_locked(self, locked: bool) -> None: ...

    # -- Auto Gimbal switch: gate the l/x tracking slew (NOT 'g', NOT mouse) --
    def set_auto_gimbal(self, on: bool) -> None:
        """When False, the sensor must not auto-slew the gimbal while vision-
        tracking (active).  Operator mouse aiming and the 'g' freeze are
        unaffected.  No-op for sensors that never slew (oracle)."""
        ...

    # -- Operator gimbal aim + detection-lost reprojection hand-off --
    def set_mouse(self, x: int, y: int) -> None:
        """Operator mouse position in the gimbal frame (non-tracking aim).
        No-op for sensors that don't slew a gimbal from a mouse (oracle)."""
        ...
    def set_last_world_dir(self, world_dir) -> None:
        """Controller hands the current world estimate each tick so a live
        sensor can keep the gimbal pointed when detection is lost.  No-op for
        the oracle sensor."""
        ...


# -----------------------------------------------------------------------------
# Camera - minimal frame-source interface.
# -----------------------------------------------------------------------------
#
# Both the nose camera and gimbal camera implement this.  The HUD doesn't care
# whether the frame comes from gz-transport, V4L2, GStreamer, or a video file.
class Camera(Protocol):
    @property
    def received_first_frame(self) -> bool: ...
    def render(self) -> np.ndarray:
        """Return the latest frame as a BGR ndarray (height, width, 3)."""
        ...
