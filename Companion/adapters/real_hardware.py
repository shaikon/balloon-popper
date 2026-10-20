# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Real-hardware adapter stubs.

When you have your real gimbal and companion computer chosen, fill these in
according to the protocols you settle on.  Three classes, three Protocols
from `Companion/interfaces.py`:

  PhysicalGimbalDriver       → GimbalDriver
  CompanionDetectionSensor   → BalloonSensor
  V4l2NoseCamera             → Camera

Open questions to settle BEFORE implementing:

  1. Gimbal protocol.  Options:
     (a) MAVLink GIMBAL_MANAGER_SET_PITCHYAW (modern, FC-routed)
     (b) MAVLink MOUNT_CONTROL (deprecated but ubiquitous)
     (c) Proprietary serial (Gremsy SBGC, Align, Storm32) over UART
     (d) Companion computer drives gimbal directly, FC just gets readback

     Recommendation: (a) if your FC firmware supports it, since you already
     have MAVLink wired up.  (c) gives lower latency but adds a serial port.

  2. Gimbal readback.  Does your gimbal expose measured angle?
     MAVLink: GIMBAL_DEVICE_ATTITUDE_STATUS at ~10 Hz.
     Most modern gimbals do; if yours does, set has_readback=True and the
     α_C trust factor drops out (see estimator.world_est_trust).

  3. Detection protocol from companion computer.  Options:
     (a) UDP JSON: simple, language-agnostic, easy to debug
     (b) ROS 2 topic: idiomatic if your companion runs ROS already
     (c) MAVLink LANDING_TARGET / CAMERA_TRACKING_IMAGE_STATUS: goes over
         your existing MAVLink link, FC can route it
     (d) Shared memory: lowest latency, hardest to debug

     Recommendation: (a) over loopback or USB-over-IP between flight
     computer and companion.  Trivial to implement, trivial to record/replay
     for offline testing.

  4. Detection payload contents (minimum):
     - timestamp (for staleness check; reject older than ~100ms)
     - either pixel (x, y) in known image dimensions, OR already-computed
       body-frame bearing (pan, tilt)
     - detector confidence ∈ [0, 1]
     - optionally bounding box (for HUD overlay)

  5. Who drives the gimbal?
     - If the FCC (this Python process) drives the gimbal, then
       CompanionDetectionSensor only receives detection info, and the
       sensor's gimbal slew loop lives here (same pattern as the sim).
     - If the companion drives the gimbal (e.g. it's doing its own visual
       servoing), then the sensor receives both detection AND current
       gimbal angle from the companion, and PhysicalGimbalDriver is just
       a readback proxy.

  6. Camera feed for HUD.  The control law doesn't need a frame on the FCC
     at all if the companion does all detection.  But it's nice to have for
     debugging.  V4L2 via OpenCV's VideoCapture is the path of least
     resistance if you have an HDMI-to-CSI bridge or USB camera capture.
     Skip if you don't need it.
"""

from __future__ import annotations
import numpy as np
from typing import Optional, Tuple


class PhysicalGimbalDriver:
    """Implements GimbalDriver against the real gimbal.

    Implementation choice depends on the answer to Q1 above.  For MAVLink
    GIMBAL_MANAGER_SET_PITCHYAW (the recommended path), this class takes
    a reference to the AutopilotLink and uses link.master to send the
    MAVLink commands.  Subscribes to GIMBAL_DEVICE_ATTITUDE_STATUS via
    link's message queue to populate measured_pan_deg / measured_tilt_deg.
    """
    def __init__(self, link, slew_rate_dps: float):
        # link: AutopilotLink (the MAVLink one — used for sending commands
        #                      and subscribing to gimbal telemetry)
        # slew_rate_dps: from your gimbal's datasheet
        raise NotImplementedError(
            "PhysicalGimbalDriver: pick a protocol from the file header "
            "comments and implement.")

    @property
    def measured_pan_deg(self) -> float: raise NotImplementedError
    @property
    def measured_tilt_deg(self) -> float: raise NotImplementedError
    @property
    def commanded_pan_deg(self) -> float: raise NotImplementedError
    @property
    def commanded_tilt_deg(self) -> float: raise NotImplementedError
    @property
    def slew_rate_dps(self) -> float: raise NotImplementedError
    @slew_rate_dps.setter
    def slew_rate_dps(self, v: float) -> None:
        # The Gain Tuner routes its "Gimbal Slew" control here (controller's
        # ParamStore writes gimbal.slew_rate_dps).  Implement a setter so the
        # operator can tune the value that feeds the trust formula's alpha_C.
        raise NotImplementedError
    @property
    def has_readback(self) -> bool: return True   # most real gimbals do
    @property
    def measured_read_mono(self) -> float:
        # time.monotonic() when the measured angles were read: GUIDE carries
        # their age, the error in pairing them with the FC's attitude.
        raise NotImplementedError

    def command(self, pan_deg: float, tilt_deg: float) -> None: raise NotImplementedError
    def reset(self) -> None: raise NotImplementedError


class CompanionDetectionSensor:
    """Implements BalloonSensor.  Receives detections from the companion
    computer over <protocol-TBD>; runs the gimbal visual-servo loop if
    the FCC owns the gimbal (see Q5).

    Trust formula on real hardware:
      - α_A: still applies — pixel offset from frame centre.  The companion
        should report detection pixel or already-normalised offset.
      - α_C: drops out if gimbal.has_readback is True (the angle being read
        back IS the truth).  Otherwise same formula as sim.
      - α_D (NEW for real): detector confidence from the companion.  Multiply
        in as another physics-derived factor.
    """
    def __init__(self, gimbal: 'PhysicalGimbalDriver', link,
                 detection_endpoint: str):
        # detection_endpoint: e.g. "udp://0.0.0.0:14660" or similar
        raise NotImplementedError(
            "CompanionDetectionSensor: pick a protocol from the file header "
            "comments and implement.")

    def update_per_tick(self, dt: float) -> None: raise NotImplementedError
    def measurement(self) -> Optional[Tuple[np.ndarray, float]]: raise NotImplementedError
    def detection_pixel_for_hud(self) -> Optional[Tuple[int, int]]: raise NotImplementedError
    def inject_synthetic_world_pos(self, world_pos_ned: np.ndarray) -> None: raise NotImplementedError
    def inject_synthetic_world_dir(self, world_dir_unit: np.ndarray) -> None: raise NotImplementedError
    def clear_synthetic(self) -> None: raise NotImplementedError
    @property
    def synthetic_active(self) -> bool: raise NotImplementedError
    def set_vision_tracking(self, enabled: bool) -> None: raise NotImplementedError
    @property
    def vision_tracking(self) -> bool: raise NotImplementedError


class V4l2NoseCamera:
    """Implements Camera against a /dev/videoN device via OpenCV VideoCapture.
    Optional — only needed if you want a live nose-cam feed in the HUD
    during flight test.  Skip if not."""
    def __init__(self, device: str = "/dev/video0",
                 width: int = 1280, height: int = 720):
        raise NotImplementedError(
            "V4l2NoseCamera: implement with cv2.VideoCapture + a background "
            "drain thread to avoid frame backlog.")

    @property
    def received_first_frame(self) -> bool: raise NotImplementedError
    def render(self) -> np.ndarray: raise NotImplementedError
