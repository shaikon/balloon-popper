# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Gazebo simulation adapters.

Implements the four interface Protocols against Gazebo + gz-transport + OpenCV
HSV detection:

    GazeboCamera          -> Camera        (subscribes to a gz image topic)
    GzGimbalDriver        -> GimbalDriver   (publishes /gimbal/{pan,tilt}/cmd_pos;
                                            no readback, so measured = commanded)
    HsvDetectionSensor    -> BalloonSensor   (the live 'x' path: HSV chevron
                                            detection + visual-servo gimbal slew)
    SyntheticOracleSensor -> BalloonSensor   (the test 'p' path: anchored world
                                            balloon, bypasses the gimbal entirely)

SYSTEM-VIEW NOTE
----------------
HsvDetectionSensor and SyntheticOracleSensor are TWO sensors implementing ONE
protocol.  The controller mounts one of them and never branches on which.  On
the real airframe a CompanionSensor (real_hardware.py) replaces HsvDetectionSensor
with an identical interface; the oracle path can stay for cascade validation or
be replaced by a companion test-mode message.  This is the whole point of the
restructure: 'x' and 'p' are the same control path, different sensor.
"""
from __future__ import annotations
import logging
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from typing import Optional, Tuple

import cv2
import numpy as np

from ..constants import (
    GIMBAL_PAN_MAX, GIMBAL_TILT_MAX, GIMBAL_HZ, GIMBAL_SLEW_RATE,
    GIMBAL_DT_MAX_S, GIMBAL_FOV_DEG,
    BALLOON_HSV_LOWER, BALLOON_HSV_UPPER, BALLOON_MIN_AREA,
)
from ..geometry import (euler_to_rotation_matrix, pan_tilt_to_vector,
                        aspect_fy_scale, project_world_vec_to_gimbal_pixel)

log = logging.getLogger("balloon_chaser")


# -----------------------------------------------------------------------------
# gz-transport bindings (version varies by Gazebo release).
# -----------------------------------------------------------------------------
_gz_node_class = None
_gz_image_class = None
_gz_double_class = None
_gz_pose_v_class = None
_gz_caminfo_class = None

for _transport_ver, _msgs_ver in [('13', '10'), ('12', '9'), ('11', '8')]:
    try:
        _transport_mod = __import__(f'gz.transport{_transport_ver}', fromlist=['Node'])
        _msgs_mod      = __import__(f'gz.msgs{_msgs_ver}.image_pb2',  fromlist=['Image'])
        _double_mod    = __import__(f'gz.msgs{_msgs_ver}.double_pb2', fromlist=['Double'])
        _gz_node_class   = _transport_mod.Node
        _gz_image_class  = _msgs_mod.Image
        _gz_double_class = _double_mod.Double
        # Pose_V (vector of poses) - reads the true gimbal link pose back from
        # the PosePublisher.  Optional: if unavailable, the readback stays off and
        # measured_* falls back to commanded.
        try:
            _pose_v_mod    = __import__(f'gz.msgs{_msgs_ver}.pose_v_pb2', fromlist=['Pose_V'])
            _gz_pose_v_class = _pose_v_mod.Pose_V
        except ImportError:
            _gz_pose_v_class = None
        # CameraInfo carries the TRUE intrinsics (fx, fy, cx, cy) Gazebo renders
        # with - the authoritative source for the projection, instead of deriving
        # them from the nominal horizontal_fov.  Optional: if unavailable, the HUD
        # falls back to the nominal-FOV derivation.
        try:
            _caminfo_mod   = __import__(f'gz.msgs{_msgs_ver}.camera_info_pb2', fromlist=['CameraInfo'])
            _gz_caminfo_class = _caminfo_mod.CameraInfo
        except ImportError:
            _gz_caminfo_class = None
        log.info(f"gz.transport{_transport_ver} bindings loaded OK")
        break
    except ImportError:
        continue

if _gz_node_class is None:
    raise ImportError(
        "\n\nCould not find gz-transport Python bindings!\n"
        "Install them with:\n"
        "  sudo apt install python3-gz-transport13 python3-gz-msgs10\n"
        "or for older Gazebo versions:\n"
        "  sudo apt install python3-gz-transport12 python3-gz-msgs9\n"
    )

GAZEBO_CAMERA_TOPIC = (
    '/world/runway/model/talon_balloon_chaser'
    '/link/chase_cam_link/sensor/chase_camera/image'
)
GAZEBO_GIMBAL_CAMERA_TOPIC = (
    '/world/runway/model/talon_balloon_chaser'
    '/link/gimbal_cam_link/sensor/gimbal_camera/image'
)
# PosePublisher feeds link world-poses (model-relative) onto this world topic.
# We read gimbal_cam_link's orientation from it to recover the TRUE gimbal pose.
GAZEBO_DYNAMIC_POSE_TOPIC = '/world/runway/dynamic_pose/info'
GIMBAL_CAM_LINK_NAME = 'gimbal_cam_link'

# camera_info topic for the nose (chase) camera - same path as its image topic with
# 'camera_info' in place of 'image'.  Carries the true intrinsic matrix K.
GAZEBO_NOSE_CAMERA_INFO_TOPIC = (
    '/world/runway/model/talon_balloon_chaser'
    '/link/chase_cam_link/sensor/chase_camera/camera_info'
)

# Single shared gz-transport Node - only one Node per process is reliable.
_gz_shared_node = _gz_node_class()


class NoseCameraIntrinsics:
    """Reads the nose camera's intrinsics and native render resolution from
    Gazebo's camera_info topic.

    Gazebo publishes a CameraInfo message carrying the intrinsic matrix K (row-
    major: [fx 0 cx; 0 fy cy; 0 0 1]) and the native render width/height.  The
    native resolution is what the HUD needs: frames are resized to the display
    resolution before projection, and the vertical scale correction is computed
    from the native-vs-display aspect ratio.

    DISPLAY ONLY: consumed solely by the HUD nose-camera projection.  The control
    path never reads this - it works in body-frame angles and is FOV-agnostic.
    """

    def __init__(self, topic=GAZEBO_NOSE_CAMERA_INFO_TOPIC):
        self.topic = topic
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.native_w = None
        self.native_h = None
        self._lock = threading.Lock()
        if _gz_caminfo_class is None:
            log.info("CameraInfo bindings unavailable; nose intrinsics will fall "
                     "back to nominal FOV.")
            return
        try:
            _gz_shared_node.subscribe(_gz_caminfo_class, topic, self._on_info)
            log.info(f"Subscribed to nose camera_info: {topic}")
        except Exception as e:   # pragma: no cover - depends on live gz
            log.warning(f"Could not subscribe to camera_info ({e}); "
                        "nose intrinsics fall back to nominal FOV.")

    def _on_info(self, msg):
        # gz.msgs.CameraInfo: the intrinsic matrix K is row-major
        # [fx 0 cx 0 fy cy 0 0 1].  Different gz versions expose it as
        # msg.intrinsics.k or msg.projection.p or a top-level k - try in order.
        if self.fx is not None:
            return
        k = None
        for getter in (
            lambda: list(msg.intrinsics.k),
            lambda: list(msg.k),
            lambda: list(msg.projection.p),  # P = [fx 0 cx Tx; 0 fy cy Ty; ...]
        ):
            try:
                cand = getter()
                if cand and len(cand) >= 6 and cand[0] > 0 and cand[4] > 0:
                    k = cand
                    break
            except Exception:
                continue
        if k is None:
            return
        with self._lock:
            self.fx, self.cx = float(k[0]), float(k[2])
            self.fy, self.cy = float(k[4]), float(k[5])
        # Capture the native render dimensions the intrinsics correspond to.
        wpx = hpx = None
        for getter in (lambda: (int(msg.width), int(msg.height)),
                       lambda: (int(msg.image_width), int(msg.image_height))):
            try:
                wpx, hpx = getter()
                if wpx and hpx:
                    break
            except Exception:
                continue
        self.native_w, self.native_h = wpx, hpx
        log.info(f"Nose camera intrinsics from Gazebo: "
                 f"fx={self.fx:.1f} fy={self.fy:.1f} "
                 f"cx={self.cx:.1f} cy={self.cy:.1f} "
                 f"(fy/fx={self.fy/self.fx:.4f})"
                 + (f" native {wpx}x{hpx}" if wpx else ""))

    @property
    def have_intrinsics(self):
        return self.fx is not None and self.fy is not None


# -----------------------------------------------------------------------------
# SdfManifest  -  the sim-side analogue of the ArduPlane parameter gate
# -----------------------------------------------------------------------------
EXPECTED_MODEL_NAME = 'talon_balloon_chaser'
DEFAULT_MODEL_SDF = '~/talon_sim/models/talon_balloon_chaser/model.sdf'


def resolve_model_sdf(override=None, model_name=EXPECTED_MODEL_NAME):
    """Path of the ACTIVE model SDF, resolved the way Gazebo resolves
    model://<name>: an explicit override (arg or the VP_MODEL_SDF env var) wins,
    then each entry of the gz resource-path env vars is searched for
    <dir>/<name>/model.sdf, and finally the project's default install path.
    Returns the first that exists, or None."""
    cands = []
    if override:
        cands.append(override)
    if os.environ.get('VP_MODEL_SDF'):
        cands.append(os.environ['VP_MODEL_SDF'])
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH',
                'GAZEBO_MODEL_PATH'):
        for d in os.environ.get(var, '').split(os.pathsep):
            if d:
                cands.append(os.path.join(d, model_name, 'model.sdf'))
    cands.append(DEFAULT_MODEL_SDF)
    for c in cands:
        p = os.path.expanduser(c)
        if os.path.isfile(p):
            return p
    return None


class SdfManifest:
    """Startup manifest of the ACTIVE model SDF - the sim-side analogue of the
    ArduPlane parameter gate (fc_param_manifest.py + verify_param_manifest).

    The FC gate reads the flight controller's live parameters and checks the
    aircraft is the one this build was frozen against.  This does the same for
    the SIMULATOR: at launch it reads the parameters of the SDF Gazebo is flying
    - FROM THE FILE, not from a runtime topic - records every camera and gimbal
    parameter it finds, and checks the identity (right model, gimbal camera
    present).  A swapped or renamed SDF is caught and reported rather than
    silently mis-scored.

    GROUND TRUTH, COMPARISON ONLY.  Like the FC identity values these numbers
    describe what is flying; unlike gains they are NEVER fed into a calculation.
    The gimbal camera's hfov_deg is exposed for ONE purpose - the fov_truth_deg
    CSV column that scores the FOV ESTIMATE - and is never handed to
    FovEstimator, the control law, or the steering path.  A mismatch here WARNS
    and never raises: this is a diagnostic, and a missing or wrong SDF must not
    ground the aircraft.  Only the FC gate, which guards against flying the wrong
    airframe, refuses to start.
    """

    def __init__(self, path=None, expected_model=EXPECTED_MODEL_NAME):
        self.expected_model = expected_model
        self.path = resolve_model_sdf(override=path, model_name=expected_model)
        self.model_name = None
        self.cameras = []          # list of param dicts, one per camera sensor
        self.gimbal_joints = []
        self.hfov_deg = None       # gimbal camera - the ground truth
        self.vfov_deg = None
        self._parse()

    def _parse(self):
        if not self.path:
            return
        try:
            model = ET.parse(self.path).getroot().find('model')
            self.model_name = model.get('name')
            for link in model.findall('link'):
                for sensor in link.findall('sensor'):
                    if sensor.get('type') != 'camera':
                        continue
                    cam = sensor.find('camera')
                    hfov = float(cam.findtext('horizontal_fov'))
                    img = cam.find('image')
                    w = int(img.findtext('width'))
                    h = int(img.findtext('height'))
                    clip = cam.find('clip')
                    self.cameras.append({
                        'link': link.get('name'), 'sensor': sensor.get('name'),
                        'hfov_deg': math.degrees(hfov),
                        # Gazebo derives the vertical FOV from the horizontal one
                        # and the image aspect ratio.
                        'vfov_deg': math.degrees(
                            2.0 * math.atan(math.tan(hfov / 2.0) * h / w)),
                        'width': w, 'height': h,
                        'update_rate': float(sensor.findtext('update_rate', '0') or 0),
                        'near': float(clip.findtext('near')) if clip is not None else 0.0,
                        'far': float(clip.findtext('far')) if clip is not None else 0.0,
                    })
            for joint in model.findall('joint'):
                if joint.get('name') not in ('gimbal_pan_joint', 'gimbal_tilt_joint'):
                    continue
                axis = joint.find('axis')
                lim = axis.find('limit') if axis is not None else None
                self.gimbal_joints.append({
                    'name': joint.get('name'),
                    'axis': (axis.findtext('xyz').strip()
                             if axis is not None and axis.findtext('xyz') else ''),
                    'lower_deg': (math.degrees(float(lim.findtext('lower')))
                                  if lim is not None and lim.findtext('lower') else None),
                    'upper_deg': (math.degrees(float(lim.findtext('upper')))
                                  if lim is not None and lim.findtext('upper') else None),
                })
            gc = next((c for c in self.cameras
                       if c['sensor'] == 'gimbal_camera'), None)
            if gc is not None:
                self.hfov_deg = gc['hfov_deg']
                self.vfov_deg = gc['vfov_deg']
        except Exception as e:                        # noqa: BLE001
            log.warning("SDF manifest: could not parse %s (%s); "
                        "FOV ground truth is off.", self.path, e)

    def verify_and_report(self):
        """Log the staged manifest at launch, mirroring verify_param_manifest:
        Stage 1 the file resolved and parsed, Stage 2 identity, Stage 3 the full
        record.  Warns on any mismatch; never raises (diagnostic only)."""
        log.info('=== SDF MANIFEST (active model, from file) ===')
        if not self.path:
            log.warning('  no model SDF resolved - set VP_MODEL_SDF or '
                        'GZ_SIM_RESOURCE_PATH; FOV ground truth is off.')
            return
        log.info('  file: %s', self.path)
        if self.model_name is None:
            log.warning('  Stage 1: SDF did not parse - no manifest.')
            return
        # Stage 2 - identity.
        if self.model_name != self.expected_model:
            log.warning("  Stage 2 IDENTITY: model is '%s', expected '%s' - is "
                        "this the aircraft you think it is?",
                        self.model_name, self.expected_model)
        else:
            log.info("  Stage 2 IDENTITY: model '%s' OK", self.model_name)
        if self.hfov_deg is None:
            log.warning("  Stage 2 IDENTITY: no 'gimbal_camera' sensor found - "
                        "FOV ground truth unavailable.")
        # Stage 3 - record everything found.
        for c in self.cameras:
            log.info("  camera %s/%s: hfov=%.3f vfov=%.3f deg  %dx%d @ %.0f Hz  "
                     "clip[%.2f, %.1f]", c['link'], c['sensor'], c['hfov_deg'],
                     c['vfov_deg'], c['width'], c['height'], c['update_rate'],
                     c['near'], c['far'])
        for j in self.gimbal_joints:
            log.info("  joint %s: axis %s  limit[%.0f, %.0f] deg", j['name'],
                     j['axis'], j['lower_deg'] or 0.0, j['upper_deg'] or 0.0)
        if self.hfov_deg is not None:
            log.info("  GROUND TRUTH gimbal_camera hfov=%.3f deg - comparison "
                     "only, NEVER used in any calculation.", self.hfov_deg)

    @property
    def have_truth(self):
        return self.hfov_deg is not None


# -----------------------------------------------------------------------------
# project_to_gimbal_pixel
# Module-level (the sim's vision pipeline uses it for synthetic-detection
# rendering and last-known-balloon reprojection).  FOV-dependent; HUD-only +
# sim-only.  NOT in the control path.  Thin wrapper around the shared
# projection helper in geometry - same pinhole math as every HUD overlay.
# -----------------------------------------------------------------------------
def project_to_gimbal_pixel(world_dir, aircraft_att, gimbal_pan_deg,
                            gimbal_tilt_deg, gw, gh, fy_scale=1.0):
    """Project a world-frame unit vector onto the gimbal camera image plane,
    given aircraft attitude and gimbal orientation.  Returns (px, py) clamped
    to image bounds, or None if behind the gimbal.

    fy_scale corrects the vertical native->display resize; pass 1.0 to leave
    the projection square.
    """
    status, raw_px, raw_py = project_world_vec_to_gimbal_pixel(
        world_dir, aircraft_att, gimbal_pan_deg, gimbal_tilt_deg,
        gw, gh, GIMBAL_FOV_DEG, fy_scale)
    if status == 'behind':
        return None
    px = int(max(0, min(gw - 1, raw_px)))
    py = int(max(0, min(gh - 1, raw_py)))
    return (px, py)


def camera_fy_scale(cam):
    """Vertical focal-length correction (native_aspect/display_aspect) for a
    GazeboCamera whose native render resolution differs from its display size.
    DISPLAY ONLY.  Falls back to the nominal 4:3->16:9 ratio (0.75) when the
    camera hasn't reported a native resolution yet."""
    return aspect_fy_scale(
        getattr(cam, 'native_w', None), getattr(cam, 'native_h', None),
        getattr(cam, 'width', None), getattr(cam, 'height', None),
        fallback=0.75)


# -----------------------------------------------------------------------------
# GazeboCamera
# -----------------------------------------------------------------------------
class GazeboCamera:
    """Subscribes to a Gazebo camera topic and provides frames to OpenCV.
    Implements the Camera protocol."""

    def __init__(self, topic=GAZEBO_CAMERA_TOPIC, width=1280, height=720):
        self.topic = topic
        self.width = width
        self.height = height
        self.native_w = None
        self.native_h = None
        self.latest_frame = np.zeros((height, width, 3), dtype=np.uint8)
        self._lock = threading.Lock()
        self._received = False
        # DISPLAY GATE.  False = keep the subscription but skip the per-frame
        # colour convert + resize, which is ~97% of the decode cost (0.98 ms of
        # 1.01 ms at 320x240 -> 1280x720).  A plain bool: written by the control
        # thread, read here on the gz transport callback thread.  Attribute
        # read/write is atomic under the GIL, so no lock is needed and none is
        # taken - taking one here would put the transport thread in contention
        # with the render thread for _lock every frame.
        self.decode_enabled = True

        ok = _gz_shared_node.subscribe(_gz_image_class, topic, self._on_image)
        if not ok:
            raise RuntimeError(
                f"Failed to subscribe to Gazebo topic:\n  {topic}\nIs Gazebo running?")
        log.info(f"Subscribed to Gazebo camera topic:\n  {topic}")

    def _on_image(self, msg):
        try:
            # ALWAYS record arrival and the native geometry, even when decoding
            # is off: run() blocks on received_first_frame at startup (so a
            # camera that starts disabled would otherwise hang the boot), and
            # the HUD's aspect correction needs native_w/h if the view is ever
            # switched back on.  Both are a few integer stores.
            if not self._received:
                self.native_w, self.native_h = int(msg.width), int(msg.height)
                cam_id = ('chase/NOSE' if 'chase' in self.topic
                          else 'gimbal' if 'gimbal' in self.topic
                          else self.topic)
                stretched = (msg.width != self.width or msg.height != self.height)
                log.info(f"[GazeboCamera] {cam_id} render {msg.width}x{msg.height} "
                         f"-> display {self.width}x{self.height}"
                         + (" (aspect-corrected)" if stretched else "")
                         + ("" if self.decode_enabled else "  [decode OFF]"))
                self._received = True
            if not self.decode_enabled:
                return                      # skip cvtColor + resize entirely
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            with self._lock:
                self.latest_frame = frame
                # _received is set above, before the decode gate, so it is not
                # repeated here.
        except Exception as e:
            log.warning(f"[GazeboCamera] Frame error: {e}")

    def render(self):
        with self._lock:
            return self.latest_frame.copy()

    @property
    def received_first_frame(self):
        return self._received


# -----------------------------------------------------------------------------
# GzGimbalDriver  <-  _configure_gimbal_mode + _gimbal_publish_loop
# -----------------------------------------------------------------------------
def _quat_to_rotation_matrix(x, y, z, w):
    """Unit-quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def gimbal_quat_to_pan_tilt(q, q_ref):
    """Recover gimbal pan/tilt (controller convention: right/up=+) from the
    gimbal_cam_link orientation quaternion q, relative to the centered reference
    q_ref (captured LIVE when the gimbal is commanded to pan=tilt=0).

    SOLVED from a 26-point calibration sweep (calibrate --calibrate-gimbal):
    the delta rotation in the centered frame, D = R(q_ref)^T @ R(q), is exactly
        D = Rz(-pan) @ Ry(-tilt)
    i.e. pan is a rotation about the centered-frame Z axis and tilt about its Y
    axis, both negated relative to the controller's right/up=+ convention.
    Extracting the Z-then-Y Euler angles and negating:
        pan  = -atan2(D[1,0], D[0,0])
        tilt = -atan2(-D[2,0], D[2,2])
    This reproduces all 26 calibration points (pure-pan, pure-tilt, and diagonal)
    to within 0.07 deg - essentially the gimbal's settling precision.  Centered
    -> (0, 0), so the nose is never spuriously 'behind' at startup.

    Requires a live q_ref captured this session (a stale reference is in a
    different frame); returns (0, 0) if none yet.
    """
    if q_ref is None:
        return (0.0, 0.0)

    D = _quat_to_rotation_matrix(*q_ref).T @ _quat_to_rotation_matrix(*q)
    pan = -math.degrees(math.atan2(D[1, 0], D[0, 0]))
    tilt = -math.degrees(math.atan2(-D[2, 0], D[2, 2]))
    return (pan, tilt)


class GzGimbalDriver:
    """Publishes pan/tilt commands to the Gazebo joint position controllers.
    Implements the GimbalDriver protocol.

    Pose readback: subscribes to the PosePublisher's dynamic-pose topic and reads
    gimbal_cam_link's true orientation, so measured_* returns the ACTUAL physical
    gimbal pose (not the commanded value).  The Gazebo JointPositionController PID
    lags the command during slews, so steering must use the true pose.  When the
    readback is live, has_readback is True (the trust formula drops its no-readback
    discount).  If the Pose_V binding or topic is unavailable, it falls back to
    measured == commanded with has_readback False (previous behaviour).

    The background publish thread is decoupled from the main loop so blocking
    gz-transport publish() calls cannot stall control.
    """

    def __init__(self, slew_rate_dps=GIMBAL_SLEW_RATE):
        self._commanded_pan = 0.0    # degrees, internal convention right/up=+
        self._commanded_tilt = 0.0
        self._slew_rate = slew_rate_dps

        # --- Pose readback state ---
        self._readback_lock = threading.Lock()
        self._gimbal_quat = None       # latest (x,y,z,w) for gimbal_cam_link
        self._quat_ref = None          # centered reference quaternion (calibrated)
        self._readback_active = False  # True once we have parsed >=1 pose msg
        self._gimbal_read_mono = None  # time.monotonic() of the latest pose, None before one
        self._pending_ref_capture = True  # capture ref ONCE at startup (settled, centred)
        # The centered reference is captured exactly once - at startup, after the
        # gimbal has settled at centre - and reused thereafter.  Centre is a fixed
        # physical pose, so the gimbal returns to it on every 'r' and measured pan
        # reads exactly 0 once re-centred.  The capture is gated on the readback
        # going STILL so we don't sample the reference while the gimbal is still
        # settling at boot.
        self._ref_still_count = 0          # consecutive near-identical poses
        self._REF_STILL_DEG = 0.15         # per-step change below this = "still"
        self._REF_STILL_HITS = 4           # this many still steps -> settled

        log.info("Setting up gimbal gz-transport publishers...")
        self._pan_pub = _gz_shared_node.advertise('/gimbal/pan/cmd_pos', _gz_double_class)
        self._tilt_pub = _gz_shared_node.advertise('/gimbal/tilt/cmd_pos', _gz_double_class)
        log.info("  /gimbal/pan/cmd_pos  ready")
        log.info("  /gimbal/tilt/cmd_pos ready")

        # Subscribe to true gimbal pose (best-effort: only if Pose_V is available).
        if _gz_pose_v_class is not None:
            try:
                _gz_shared_node.subscribe(
                    _gz_pose_v_class, GAZEBO_DYNAMIC_POSE_TOPIC, self._on_pose_v)
                log.info(f"  pose readback subscribed: {GAZEBO_DYNAMIC_POSE_TOPIC}")
            except Exception as e:                       # noqa: BLE001
                log.warning(f"  pose readback subscribe FAILED ({e}); "
                            "falling back to commanded==measured")
        else:
            log.warning("  Pose_V binding unavailable; gimbal readback disabled "
                        "(measured==commanded)")

        self._thread = threading.Thread(
            target=self._publish_loop, daemon=True, name="gimbal")
        self._thread.start()

    def _on_pose_v(self, msg):
        """gz-transport callback: extract gimbal_cam_link's orientation from a
        Pose_V message and store it.  Runs on the transport thread - keep it
        cheap and lock only briefly."""
        # SIM-TIME STAMP of this pose.  Pose_V carries a header with Gazebo's
        # simulated clock; it was being discarded.  The FC differences it against
        # its own millis() to get the gimbal-angle age in SIMULATED ms - the
        # quantity that multiplies body rate to give the C-02 pairing error.
        _stamp_ms = 0
        try:
            _h = getattr(msg, 'header', None)
            _st = getattr(_h, 'stamp', None) if _h is not None else None
            if _st is not None:
                _stamp_ms = int(getattr(_st, 'sec', 0) * 1000
                                + getattr(_st, 'nsec', 0) // 1_000_000)
        except Exception:                              # noqa: BLE001
            _stamp_ms = 0
        try:
            for p in msg.pose:
                if p.name == GIMBAL_CAM_LINK_NAME:
                    o = p.orientation
                    q = (o.x, o.y, o.z, o.w)
                    with self._readback_lock:
                        prev_q = self._gimbal_quat
                        self._gimbal_quat = q
                        self._readback_active = True
                        self._gimbal_stamp_ms = _stamp_ms
                        self._gimbal_read_mono = time.monotonic()
                        # Capture the centered reference only when the gimbal is
                        # commanded to centre AND has actually STOPPED slewing
                        # (its readback quaternion has gone still).  A fixed time
                        # guess can fire while the gimbal is still a degree or two
                        # short of home, baking that error into the reference and
                        # leaving the recenter slightly off.  Detecting "stopped
                        # moving" from the readback adapts to the real settle
                        # dynamics.  Requires the command to be centred and a few
                        # consecutive near-identical poses (a brief still hold).
                        if (self._pending_ref_capture
                                and abs(self._commanded_pan) < 0.5
                                and abs(self._commanded_tilt) < 0.5):
                            still = False
                            if prev_q is not None:
                                # angular change between consecutive poses (deg)
                                dot = abs(sum(a * b for a, b in zip(q, prev_q)))
                                dot = max(-1.0, min(1.0, dot))
                                dtheta = math.degrees(2.0 * math.acos(dot))
                                if dtheta < self._REF_STILL_DEG:
                                    self._ref_still_count += 1
                                    still = self._ref_still_count >= self._REF_STILL_HITS
                                else:
                                    self._ref_still_count = 0
                            else:
                                self._ref_still_count = 0
                            if still:
                                self._quat_ref = q
                                self._pending_ref_capture = False
                                self._ref_still_count = 0
                                log.info("  gimbal pose reference captured (settled at centre)")
                    break
        except Exception:                                # noqa: BLE001
            pass  # never let a malformed message crash the transport thread

    def _true_pan_tilt(self):
        """Return (pan, tilt) in degrees from the readback, or None if unavailable."""
        with self._readback_lock:
            q = self._gimbal_quat
            qref = self._quat_ref
        if q is None or qref is None:
            return None
        return gimbal_quat_to_pan_tilt(q, qref)

    @property
    def measured_read_mono(self):
        """time.monotonic() when the measured angles were read, or None before
        the first pose.  GUIDE carries their age from it: the FC pairs the angles
        with an on-chip attitude, so that age is the pairing error."""
        return self._gimbal_read_mono

    @property
    def measured_quat(self):
        """Raw gimbal_cam_link quaternion (x,y,z,w) from the readback, or None.
        Diagnostic only - lets the log capture the true pose source so the
        pan/tilt conversion signs can be verified offline."""
        with self._readback_lock:
            return self._gimbal_quat

    def _publish_loop(self):
        """Publish current gimbal angles to Gazebo at GIMBAL_HZ.  Negate because
        internal convention is right/up=positive, Gazebo joint convention is
        the opposite."""
        while True:
            pan_msg = _gz_double_class()
            tilt_msg = _gz_double_class()
            pan_msg.data = math.radians(-self._commanded_pan)
            tilt_msg.data = math.radians(-self._commanded_tilt)
            self._pan_pub.publish(pan_msg)
            self._tilt_pub.publish(tilt_msg)
            time.sleep(1.0 / GIMBAL_HZ)

    def command(self, pan_deg, tilt_deg):
        self._commanded_pan = float(np.clip(pan_deg, -GIMBAL_PAN_MAX, GIMBAL_PAN_MAX))
        self._commanded_tilt = float(np.clip(tilt_deg, -GIMBAL_TILT_MAX, GIMBAL_TILT_MAX))

    def reset(self):
        # Command the gimbal back to its known absolute centre.  The centered
        # reference is the one captured once at startup; it is not recaptured
        # here, because the gimbal returns to the same physical centre every time,
        # so measured pan reads 0 once it has re-centred.
        self._commanded_pan = 0.0
        self._commanded_tilt = 0.0

    @property
    def commanded_pan_deg(self): return self._commanded_pan
    @property
    def commanded_tilt_deg(self): return self._commanded_tilt
    @property
    def measured_pan_deg(self):
        pt = self._true_pan_tilt()
        return pt[0] if pt is not None else self._commanded_pan
    @property
    def measured_tilt_deg(self):
        pt = self._true_pan_tilt()
        return pt[1] if pt is not None else self._commanded_tilt
    @property
    def slew_rate_dps(self): return self._slew_rate
    @slew_rate_dps.setter
    def slew_rate_dps(self, v): self._slew_rate = float(v)
    @property
    def has_readback(self):
        # True only once we have a live pose AND a calibrated reference, so the
        # trust formula drops its no-readback discount only when measured_* is
        # genuinely the true pose.
        with self._readback_lock:
            return self._readback_active and self._quat_ref is not None


# -----------------------------------------------------------------------------
# _hsv_detect
# -----------------------------------------------------------------------------
def _hsv_detect(frame):
    """Detect the balloon by HSV colour thresholding.  Returns (cx, cy) or None.

    BALLOON COLOUR - SIMULATION vs THE COMPETITION FIELD:
    In simulation the balloon is a yellow chevron, simply because that is the
    object readily available to place in the Gazebo world; the colour and shape
    are a matter of convenience.  The competition's rule is different: pop the
    RED balloons and leave the yellow ones alone.  So for the field this
    detector's HSV band (BALLOON_HSV_LOWER/UPPER) is set to the red balloons
    instead of the simulation yellow, and yellow is excluded as a non-balloon.
    The detection geometry, gimbal servoing and steering downstream are
    unchanged - only what colour counts as "the balloon" differs between the
    simulator and the field.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALLOON_HSV_LOWER, BALLOON_HSV_UPPER)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < BALLOON_MIN_AREA:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


# -----------------------------------------------------------------------------
# HsvDetectionSensor  <-  the live 'x' path of _update_gimbal + _control_balloon_chaser
# -----------------------------------------------------------------------------
class HsvDetectionSensor:
    """Live vision sensor: HSV chevron detection + visual-servo gimbal slew.
    Implements BalloonSensor.

    Owns the gimbal slew loop (drives the GzGimbalDriver toward the detection
    pixel) and produces the body-frame measurement from the gimbal angles.
    When no detection is present it reprojects the last known world balloon
    (passed in via update_per_tick) to keep the gimbal pointed.

    Measurement source (matches source _control_balloon_chaser): the body-frame bearing
    is pan_tilt_to_vector(gimbal_pan, gimbal_tilt) - read from the gimbal angles,
    NOT reconstructed from pixels.  The pixel norm survives only as the trust
    signal (alpha_A).  Trust is computed by the controller via world_est_trust;
    the sensor exposes the pieces it needs (det_norm, body_rate, slew, readback)
    through measurement()'s trust value... see note in measurement().
    """

    def __init__(self, camera: GazeboCamera, gimbal: GzGimbalDriver, link):
        self._cam = camera
        self._gimbal = gimbal
        self._link = link            # AutopilotLink - for has_readback / rates
        self._active = False
        self._gimbal_locked = True   # starts locked; toggled only via set_gimbal_locked ('g'/'l'/'x')
        self._auto_gimbal = True     # Auto Gimbal switch: gates the l/x tracking slew only
        # Manual-search vision tracking ('l'): the operator aims the gimbal by mouse
        # until vision first detects the chevron, at which point _vision_found
        # latches and vision owns the gimbal for the rest of the lock.  Both are
        # activated by set_active(..., manual_search=True) and cleared on deactivate;
        # balloon chaser ('x') and ptest leave manual_search False and so track from the
        # first tick with no operator-aim phase.
        self._manual_search = False
        self._vision_found = False
        self._last_slew_time = time.monotonic()

        # Detection state (refreshed each update_per_tick)
        self._det_pixel: Optional[Tuple[int, int]] = None
        self._det_norm_x = 0.0
        self._det_norm_y = 0.0
        self._have_detection = False

        # Last known world balloon (for detection-lost reprojection).  The
        # controller passes the current estimate in via update_per_tick.
        self._last_world_dir: Optional[np.ndarray] = None

        # Mouse aim point (operator gimbal aiming when not actively vision-tracking).
        # Defaults to frame centre.
        self._mouse_x = camera.width // 2
        self._mouse_y = camera.height // 2

        # Oracle (ptest) state - coexists with the live pipeline (see oracle
        # methods below).  None when not running a synthetic-injection test.
        self._oracle_world_pos: Optional[np.ndarray] = None
        self._oracle_world_dir: Optional[np.ndarray] = None
        # Optional PULL source for the oracle position (see set_oracle_position_fn):
        # when set, it is evaluated once per tick and caches into _oracle_world_pos.
        self._oracle_pos_fn = None

    # -- BalloonSensor interface --
    def set_active(self, enabled: bool, manual_search: bool = False) -> None:
        # Vision detection on/off.  INDEPENDENT of the gimbal lock: turning
        # vision on does NOT unlock the gimbal.  The callers that want the gimbal
        # to slew unlock it EXPLICITLY via set_gimbal_locked(False) - the 'l'
        # vision tracking and 'x' balloon chaser.  Callers that activate the sensor WITHOUT
        # wanting gimbal motion - ptest, which drives the cascade from the oracle
        # and freezes the gimbal - therefore leave it locked.  'g' stays the only
        # manual lock toggle.  (set_gimbal_locked(False) resets _last_slew_time,
        # so unlocking through it still avoids a stale first-slew step.)
        #
        # manual_search (the 'l' vision tracking) arms an operator-aim search: until
        # vision first detects the chevron the gimbal follows the mouse, then the
        # first detection latches _vision_found and vision owns it for the rest
        # of the lock.  The latch is cleared on every (de)activation so a fresh
        # lock always starts searching.
        self._active = bool(enabled)
        self._manual_search = bool(enabled and manual_search)
        self._vision_found = False
        if not enabled:
            # l-OFF clears the detection so the HUD stops drawing a stale marker.
            self._det_pixel = None
            self._have_detection = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def gimbal_locked(self) -> bool:
        return self._gimbal_locked

    def set_gimbal_locked(self, locked: bool) -> None:
        """Freeze/unfreeze the gimbal slew ('g' key).  Independent of active."""
        self._gimbal_locked = bool(locked)
        if not locked:
            self._last_slew_time = time.monotonic()

    def set_auto_gimbal(self, on) -> None:
        """Auto Gimbal switch: when False, suppress the AUTOMATIC l/x tracking
        slew.  Independent of 'g' (gimbal_locked) and of operator mouse aiming."""
        self._auto_gimbal = bool(on)

    # Oracle (ptest) support.  In sim, the SAME mounted sensor serves both live
    # HSV vision ('x') and synthetic-injection ptest ('p') - exactly as the
    # source did (one object held both _synthetic_active and the live pipeline).
    # When an oracle is set, oracle_active is True and the controller snaps the
    # estimator to oracle_world_dir(), bypassing the (empty) HSV filter.
    def set_oracle_world_position(self, pos_ned):
        self._oracle_world_pos = np.asarray(pos_ned, dtype=float)
        self._oracle_world_dir = None
        self._oracle_pos_fn = None          # an explicit push overrides a pull
    def set_oracle_position_fn(self, fn):
        """PULL form of set_oracle_world_position: hand over a callable that
        returns the balloon's world NED position right now.

        Evaluated ONCE per control tick, at the top of update_per_tick, and
        cached - so every consumer in a tick (the cascade, the range/closest-
        approach fit, the HUD reprojection) sees the SAME position, and that
        position is the balloon's true one at that tick rather than whatever a
        producer thread last managed to push.

        The pushed form left the balloon stale on ~31% of control ticks, because
        the injector's 50 Hz publish thread and the 50 Hz control loop are
        independent free-running timers that beat against each other.  Pulling
        removes the producer's timing from the problem entirely."""
        self._oracle_pos_fn = fn
        self._oracle_world_dir = None
        self._latch_oracle_position()       # active immediately, not next tick
    def _latch_oracle_position(self):
        """Evaluate the oracle position function, if one is set.  Failure leaves
        the last good position standing rather than dropping the oracle
        mid-flight, which would hand the cascade a None balloon."""
        fn = self._oracle_pos_fn
        if fn is None:
            return
        try:
            p = fn()
        except Exception:
            log.exception("[ORACLE] position callback failed")
            return
        if p is not None:
            self._oracle_world_pos = np.asarray(p, dtype=float)
    def set_oracle_world_direction(self, dir_unit):
        self._oracle_world_pos = None
        self._oracle_pos_fn = None          # a direction anchor has no position
        self._oracle_world_dir = np.asarray(dir_unit, dtype=float)
    def clear_oracle(self):
        self._oracle_world_pos = None
        self._oracle_world_dir = None
        self._oracle_pos_fn = None
        self._det_pixel = None
        self._have_detection = False
    @property
    def oracle_active(self) -> bool:
        return self._oracle_world_pos is not None or self._oracle_world_dir is not None
    @property
    def oracle_world_pos(self):
        """The oracle balloon's world NED POSITION, or None when the balloon has no
        finite position - i.e. a direction anchor (at infinity), or no oracle at
        all (real vision, which is bearing-only and yields no range).  This is
        the sole source of a balloon RANGE; see BalloonChaser._update_balloon_range."""
        return self._oracle_world_pos
    def oracle_world_dir(self, aircraft_pos_ned):
        """World-frame unit direction to the oracle balloon, or None.
        Ground case: direction from aircraft to the fixed NED point (balloon chaser
        dynamics).  Sky case: the fixed direction."""
        if self._oracle_world_pos is not None:
            if aircraft_pos_ned is None:
                return None
            vec = self._oracle_world_pos - aircraft_pos_ned
            n = float(np.linalg.norm(vec))
            if n < 1e-6:
                return np.array([1.0, 0.0, 0.0])
            return vec / n
        if self._oracle_world_dir is not None:
            return self._oracle_world_dir
        return None

    def set_mouse(self, x: int, y: int) -> None:
        """Operator mouse position in the gimbal frame (for non-tracking aim)."""
        self._mouse_x = x
        self._mouse_y = y

    def set_last_world_dir(self, world_dir) -> None:
        """Controller hands the current estimate so detection-lost reprojection
        keeps the gimbal pointed at the last known location."""
        self._last_world_dir = world_dir

    def update_per_tick(self, dt, aircraft_att, aircraft_pos_ned, aircraft_rates):
        """Pull a frame, detect, slew the gimbal toward the detection pixel.
        Mirrors the source _update_gimbal vision-tracking branch + slew.

        While an oracle is active (ptest), the gimbal is frozen and no live
        detection is formed - the controller drives the cascade from the oracle
        direction directly.  The HUD still shows a reprojected chevron via
        detection_pixel_for_hud()."""
        # FIRST thing in the tick: refresh a pulled oracle position, so the
        # cascade, the range/closest-approach fit and the HUD reprojection below
        # all work from one position, evaluated now.
        self._latch_oracle_position()
        if self.oracle_active:
            # Reproject the oracle into the gimbal frame so the operator sees the
            # balloon marker drift toward centre as the cascade converges, but do
            # NOT slew or form a measurement: the gimbal is frozen in ptest.
            wd = self.oracle_world_dir(aircraft_pos_ned)
            if wd is not None:
                frame = self._cam.render()
                gh, gw = frame.shape[:2]
                self._det_pixel = project_to_gimbal_pixel(
                    wd, aircraft_att, self._gimbal.commanded_pan_deg,
                    self._gimbal.commanded_tilt_deg, gw, gh,
                    camera_fy_scale(self._cam))
            self._have_detection = False
            self._det_norm_x = 0.0
            self._det_norm_y = 0.0
            return

        frame = self._cam.render()
        gh, gw = frame.shape[:2]
        gcx, gcy = gw / 2, gh / 2

        if self._active:
            detection = _hsv_detect(frame)
            if detection is not None:
                self._vision_found = True   # first valid detection latches the lock
            # A manual-search vision tracking is SEARCHING until that latch: the operator
            # aims the gimbal by mouse and no reprojected marker is shown.
            searching = self._manual_search and not self._vision_found

            if detection is not None:
                self._det_pixel = detection
                self._have_detection = True
            elif (not searching) and self._last_world_dir is not None:
                # Detection lost after the lock latched: reproject last known
                # world balloon so the gimbal keeps pointing.  Not a real
                # observation - flagged so the controller doesn't fold it back
                # into the estimate.
                detection = project_to_gimbal_pixel(
                    self._last_world_dir, aircraft_att,
                    self._gimbal.commanded_pan_deg,
                    self._gimbal.commanded_tilt_deg, gw, gh,
                    camera_fy_scale(self._cam))
                self._det_pixel = detection
                self._have_detection = False
            else:
                self._det_pixel = None
                self._have_detection = False

            if searching:
                # Operator aims the gimbal by mouse until vision finds the balloon.
                self._det_norm_x = (self._mouse_x - gcx) / (gw / 2)
                self._det_norm_y = -(self._mouse_y - gcy) / (gh / 2)
            elif detection is not None:
                det_x, det_y = detection
                self._det_norm_x = (det_x - gcx) / (gw / 2)
                self._det_norm_y = -(det_y - gcy) / (gh / 2)
            else:
                self._det_norm_x = 0.0
                self._det_norm_y = 0.0
        else:
            # Operator aiming via mouse (gimbal not vision-tracking).
            self._det_pixel = None
            self._have_detection = False
            self._det_norm_x = (self._mouse_x - gcx) / (gw / 2)
            self._det_norm_y = -(self._mouse_y - gcy) / (gh / 2)

        # Slew the gimbal toward the (normalised) balloon offset.  Auto Gimbal OFF
        # suppresses only the AUTOMATIC vision-tracking slew; OPERATOR MOUSE
        # aiming (sensor inactive, or manual-search 'l' tracking still searching)
        # and the 'g' freeze are unaffected.
        operator_aim = (not self._active
                        or (self._manual_search and not self._vision_found))
        if not self._gimbal_locked and (self._auto_gimbal or operator_aim):
            now = time.monotonic()
            slew_dt = now - self._last_slew_time
            self._last_slew_time = now
            if slew_dt > GIMBAL_DT_MAX_S:
                slew_dt = GIMBAL_DT_MAX_S
            new_pan = (self._gimbal.commanded_pan_deg
                       + self._det_norm_x * self._gimbal.slew_rate_dps * slew_dt)
            new_tilt = (self._gimbal.commanded_tilt_deg
                        + self._det_norm_y * self._gimbal.slew_rate_dps * slew_dt)
            self._gimbal.command(new_pan, new_tilt)
        else:
            self._last_slew_time = time.monotonic()

    def measurement(self):
        """Return (body_vec, trust) or None.

        body_vec: pan_tilt_to_vector of the gimbal angles (FOV-independent;
                  matches source _control_balloon_chaser).
        trust:    world_est_trust(det_norm, body_rate, slew, has_readback).

        Only returns a measurement when there is a REAL detection this tick
        (self._have_detection).  A detection-lost reprojection keeps the gimbal
        pointed but is NOT a measurement (returning it would be circular - it
        was generated FROM the estimate).
        """
        if not self._active or not self._have_detection:
            return None
        # Local import avoids a module-level cycle (estimator imports nothing
        # from adapters, but keep the dependency direction explicit here).
        from ..estimator import world_est_trust
        body_vec = pan_tilt_to_vector(
            self._gimbal.measured_pan_deg, self._gimbal.measured_tilt_deg)
        rates = aircraft_rates_dps(self._link)
        body_rate_abs = max(abs(rates[0]), abs(rates[1]))
        trust = world_est_trust(
            self._det_norm_x, self._det_norm_y,
            body_rate_abs, self._gimbal.slew_rate_dps,
            gimbal_has_readback=self._gimbal.has_readback)
        return (body_vec, trust)

    def detection_pixel_for_hud(self):
        return self._det_pixel

    def detection_norm(self):
        return (self._det_norm_x, self._det_norm_y)

    @property
    def have_detection(self):
        """True when a live HSV detection exists this tick (diagnostic/display)."""
        return self._have_detection

    def balloon_body_from_detection(self):
        """Return the balloon's body-frame direction derived from the IMAGE
        PROCESSING result (the detection pixel), or None if there is no live
        detection this tick.

        This is the true measured balloon direction, NOT merely where the gimbal
        is aiming: it takes the gimbal pan/tilt and ADDS the detection's angular
        offset within the gimbal frame (from the normalised detection pixel and
        the camera FOV).  When the detection sits dead-centre this equals the
        gimbal aim; when the balloon is off-centre in the gimbal image the extra
        offset places the body vector on the actual balloon.
        """
        if not self._active or not self._have_detection:
            return None
        # Normalised detection: _det_norm_x in [-1,1] across half-width to the
        # right, _det_norm_y in [-1,1] across half-height up.  Convert to angular
        # offsets via the pinhole tan-mapping.  HORIZONTAL: the edge of the frame
        # is exactly half the horizontal FOV, so _det_norm_x*tan(half_fov) is the
        # tangent of the pan offset.  VERTICAL: the frame's vertical angular
        # extent is NOT the horizontal FOV - the native render is resized
        # non-uniformly to the display frame, so the top edge corresponds to the
        # native vertical half-FOV, not half_fov.  The per-axis correction is
        # (gh/gw)/fy_scale, where fy_scale = native_aspect/display_aspect; this
        # reduces to native_h/native_w and leaves the horizontal axis untouched.
        half_fov = math.radians(GIMBAL_FOV_DEG / 2.0)
        fy_scale = camera_fy_scale(self._cam)
        gw, gh = self._cam.width, self._cam.height
        v_scale = (gh / gw) / fy_scale if (gw and fy_scale) else 0.75
        # tan-based mapping (matches the pinhole projection used to draw markers).
        off_pan_deg = math.degrees(math.atan(self._det_norm_x * math.tan(half_fov)))
        off_tilt_deg = math.degrees(math.atan(
            self._det_norm_y * math.tan(half_fov) * v_scale))
        # Use COMMANDED gimbal angles as the base: the visual servo drives these
        # to centre the detection, so they reflect where the gimbal is aimed and
        # are always live.  (measured_* depends on the pose readback, which can
        # read ~0 before its reference is captured - that made the projected
        # balloon collapse to straight-ahead even with a valid off-axis detection.)
        pan = self._gimbal.commanded_pan_deg + off_pan_deg
        tilt = self._gimbal.commanded_tilt_deg + off_tilt_deg
        return pan_tilt_to_vector(pan, tilt)


def aircraft_rates_dps(link):
    """Helper: link.rates is (p, q, r) in rad/s -> degrees/s tuple."""
    r = link.rates
    return (math.degrees(float(r[0])),
            math.degrees(float(r[1])),
            math.degrees(float(r[2])))


# -----------------------------------------------------------------------------
# SyntheticOracleSensor  <-  the 'p' / ptest path
# -----------------------------------------------------------------------------
class SyntheticOracleSensor:
    """Cascade-validation sensor: an anchored world balloon, no real vision.
    Implements BalloonSensor.

    The old ptest path.  An oracle is set via set_oracle_world_{position,
    direction}().  measurement() projects the oracle into the current body
    frame with trust=1.0; the controller snaps the estimator to the exact
    oracle direction (bypassing the filter) when oracle_active is True.

    The gimbal is NOT driven: its physical pointing has no role when the oracle
    feeds the cascade directly.  detection_pixel_for_hud reprojects the oracle so the operator
    still sees the chevron drift toward centre as the cascade converges.
    """

    def __init__(self, camera: GazeboCamera, link):
        self._cam = camera
        self._link = link
        self._active = True   # an oracle sensor is "active" by construction
        self._world_pos: Optional[np.ndarray] = None   # ground case (NED)
        self._world_dir: Optional[np.ndarray] = None    # sky case (unit dir)
        self._pos_fn = None                             # pull source (see below)
        self._last_att = np.array([0.0, 0.0, 0.0])

    # -- oracle anchoring --
    def set_oracle_world_position(self, pos_ned):
        self._world_pos = np.asarray(pos_ned, dtype=float)
        self._world_dir = None
        self._pos_fn = None
    def set_oracle_position_fn(self, fn):
        """PULL form - see HsvDetectionSensor.set_oracle_position_fn.  Evaluated
        once per tick in update_per_tick so every consumer in a tick agrees."""
        self._pos_fn = fn
        self._world_dir = None
        self._latch_oracle_position()
    def _latch_oracle_position(self):
        fn = self._pos_fn
        if fn is None:
            return
        try:
            p = fn()
        except Exception:
            log.exception("[ORACLE] position callback failed")
            return
        if p is not None:
            self._world_pos = np.asarray(p, dtype=float)
    def set_oracle_world_direction(self, dir_unit):
        self._world_pos = None
        self._pos_fn = None
        self._world_dir = np.asarray(dir_unit, dtype=float)
    def clear_oracle(self):
        self._world_pos = None
        self._world_dir = None
        self._pos_fn = None
    @property
    def oracle_active(self):
        return self._world_pos is not None or self._world_dir is not None
    @property
    def oracle_world_pos(self):
        """World NED position of the oracle balloon, or None when it is anchored
        as a direction (at infinity).  See HsvDetectionSensor.oracle_world_pos."""
        return self._world_pos

    def oracle_world_dir(self, aircraft_pos_ned):
        """World-frame unit direction to the oracle balloon, or None.
        Ground case: direction from aircraft to the fixed point (balloon chaser
        dynamics).  Sky case: the fixed direction."""
        if self._world_pos is not None:
            if aircraft_pos_ned is None:
                return None
            vec = self._world_pos - aircraft_pos_ned
            n = float(np.linalg.norm(vec))
            if n < 1e-6:
                return np.array([1.0, 0.0, 0.0])
            return vec / n
        if self._world_dir is not None:
            return self._world_dir
        return None

    # -- BalloonSensor interface --
    def set_active(self, enabled, manual_search=False): self._active = bool(enabled)  # oracle never slews, so manual_search is a no-op
    @property
    def active(self): return self._active
    @property
    def gimbal_locked(self): return True   # gimbal frozen during ptest
    def set_gimbal_locked(self, locked): pass   # no-op: oracle never slews the gimbal
    def set_auto_gimbal(self, on): pass         # no-op: oracle never slews the gimbal
    def set_mouse(self, x, y): pass            # no-op: oracle ignores operator mouse aim
    def set_last_world_dir(self, world_dir): pass  # no-op: oracle doesn't reproject

    def update_per_tick(self, dt, aircraft_att, aircraft_pos_ned, aircraft_rates):
        self._latch_oracle_position()   # one evaluation per tick, shared by all consumers
        self._last_att = aircraft_att
        self._last_pos = aircraft_pos_ned

    def measurement(self):
        """Project the oracle into the current body frame, trust=1.0.

        The controller checks oracle_active and snaps the estimator to
        oracle_world_dir() directly, so this body-frame measurement is mostly
        for interface uniformity - but it's exact, so a controller that chose
        to filter it would still converge.
        """
        wd = self.oracle_world_dir(getattr(self, '_last_pos', None))
        if wd is None:
            return None
        R = euler_to_rotation_matrix(*self._last_att)
        body_vec = R @ wd
        return (body_vec, 1.0)

    def detection_pixel_for_hud(self):
        wd = self.oracle_world_dir(getattr(self, '_last_pos', None))
        if wd is None:
            return None
        gw, gh = self._cam.width, self._cam.height
        return project_to_gimbal_pixel(wd, self._last_att, 0.0, 0.0, gw, gh,
                                       camera_fy_scale(self._cam))

    def detection_norm(self):
        # Oracle path: gimbal is frozen, norms are zero (matches source ptest
        # behaviour where _detection_norm_* were zeroed while synthetic active).
        return (0.0, 0.0)
