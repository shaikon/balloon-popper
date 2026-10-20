# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
World-frame balloon direction estimator with measurement-trust weighting.

Purpose: decouple the nose controller from the gimbal/vision sensor chain.
The estimator takes the noisy per-tick body->world reconstruction as a
measurement, and produces a clean, slowly-drifting world-frame balloon
estimate by low-pass filtering it with a trust-weighted complementary filter.
The nose controller reads ONLY the filtered estimate.

The trust factor collapses to zero when the measurement is known to be
unreliable: detection near frame edge (gimbal lagging), or aircraft body rate
exceeding gimbal slew rate (commanded gimbal angle diverging from physical
angle).  When trust is zero, the estimator holds state - the controller
dead-reckons on the last good estimate.

`world_est_trust` takes an optional `gimbal_has_readback` kwarg.  When True (a
real gimbal exposes measured-angle telemetry), alpha_C collapses to 1.0 - the
read-back angle IS the truth regardless of slew lag, so there is no
commanded-vs-actual divergence to penalise.  Default False (the sim gimbal has
no readback) applies the two-factor formula.
"""
from __future__ import annotations
import math
from typing import Optional
import numpy as np


WORLD_EST_ALPHA_NOM = 0.5   # 1/s - nominal filter bandwidth (time constant = 2 s)


class WorldBalloonEstimator:
    """World-frame balloon direction estimator with measurement-trust weighting.

    State: unit vector on S^2 representing the balloon's direction in world NED.
    Update: complementary filter with an adaptive trust gain that collapses
    to zero when the current measurement is unreliable.

    The estimator does not know anything about the specific sensor chain -
    it just receives (world-frame measurement, trust, dt) and produces an
    updated estimate.  The caller is responsible for computing the trust.
    """

    def __init__(self):
        self._estimate: Optional[np.ndarray] = None   # (3,) unit vector or None

    def reset(self) -> None:
        """Clear the estimate.  Call on mode entry / exit."""
        self._estimate = None

    def get(self) -> Optional[np.ndarray]:
        """Return current estimate (np.ndarray, unit) or None if uninitialised."""
        return self._estimate

    def snap_to(self, world_vec) -> Optional[np.ndarray]:
        """Force the estimate directly to a world-frame direction, bypassing the
        filter.  Used by the oracle path (cascade-validation test): when a test
        anchor is active, the controller snaps the estimate to the exact known
        balloon instead of filtering a measurement into it.  Normalises
        defensively; a near-zero vector clears the estimate.

        (This replaces the source's `set_direct`, which was deleted in the
        cleanup pass.  Re-introduced because the oracle path needs it - but now
        it lives behind the BalloonSensor seam, not poked from the injector.)
        """
        if world_vec is None:
            self._estimate = None
            return None
        n = np.linalg.norm(world_vec)
        self._estimate = (np.asarray(world_vec, dtype=float) / n) if n > 1e-9 else None
        return self._estimate

    def update(self, measurement, trust, dt, alpha_nom=WORLD_EST_ALPHA_NOM):
        """Fold a world-frame measurement into the estimate.

        Args:
            measurement: np.ndarray (3,) unit vector in world frame, or None
                         if no valid measurement this tick.
            trust:       scalar in [0, 1].  How much to believe the measurement.
                         0 = no update; 1 = full-rate filter bandwidth.
            dt:          tick duration (s).
            alpha_nom:   nominal filter bandwidth (1/s).

        Returns:
            current estimate (np.ndarray unit vector) or None if still
            uninitialised and no measurement was provided.
        """
        if measurement is None or trust <= 0.0:
            # No update - hold state.  If uninitialised, stay that way.
            return self._estimate

        # Normalise measurement defensively.
        n = np.linalg.norm(measurement)
        if n < 1e-9:
            return self._estimate
        z = measurement / n

        if self._estimate is None:
            # Cold start: snap to first valid measurement.
            self._estimate = z
            return self._estimate

        # Complementary filter on the sphere.  For small steps, slerp ~= lerp +
        # renormalise, which is what we do here - cheap and numerically fine
        # for the filter bandwidths we're using.
        # Effective blend weight = alpha_nom * trust * dt, clamped to [0, 1].
        w = min(1.0, max(0.0, alpha_nom * trust * dt))
        blended = (1.0 - w) * self._estimate + w * z
        bn = np.linalg.norm(blended)
        if bn < 1e-9:
            # Degenerate (shouldn't happen with small w) - keep old.
            return self._estimate
        self._estimate = blended / bn
        return self._estimate


def world_est_trust(det_norm_x, det_norm_y, body_rate_abs_dps,
                    gimbal_slew_rate_dps, *, gimbal_has_readback=False):
    """Compute the trust weight in [0, 1] for the current measurement.

    Combines two independent physics-derived factors:

      alpha_A = (1 - |det_norm|)^2
          Gimbal-tracking-quality signal.  When the balloon sits at the
          image centre the gimbal has caught up to it and the gimbal
          pan/tilt is an accurate body-frame bearing.  When the balloon
          drifts toward the image edge the gimbal is lagging - pan/tilt
          no longer equals the balloon's true body-frame direction.
          Quadratic falloff weights centred-tracking heavily.

      alpha_C = max(0, 1 - |body_rate| / gimbal_slew_rate)
          When the airframe rotates faster than the gimbal can slew,
          the commanded gimbal angle diverges from physical pointing.
          Measurements during this regime are corrupted by an unknown
          lag bias.  Pure physics.

          REAL-AIRFRAME NOTE: if the gimbal exposes measured-angle
          telemetry (gimbal_has_readback=True), this factor collapses to
          1.0.  The read-back angle IS the physical pointing regardless of
          how fast the airframe rotated - there is no commanded-vs-actual
          divergence to penalise.  alpha_A still applies as a "how centred
          is the detection" quality signal.  Default False reproduces the
          sim's exact two-factor behaviour.

    alpha_B (gimbal-command-rate saturation) is deliberately omitted in the
    initial implementation - alpha_C covers the same failure via the upstream
    cause (aircraft rate), and keeping the trust formula simple makes it
    easier to reason about in early flight tests.  Can be added later.

    Args:
        det_norm_x, det_norm_y: detection position in frame, +-1 at edges.
        body_rate_abs_dps:      max of |roll_rate|, |pitch_rate| in deg/s.
        gimbal_slew_rate_dps:   gimbal servo slew limit, deg/s.
        gimbal_has_readback:    True if the gimbal reports its measured angle.

    Returns:
        trust in [0, 1].
    """
    # alpha_A: quadratic falloff with off-centre pixel magnitude.  The
    # measurement (pan_tilt_to_vector of gimbal angles) is exact when
    # the gimbal has centred the balloon and degrades smoothly as the
    # balloon drifts toward the frame edge.  No camera-optics knowledge
    # required - the pixel norm is a dimensionless residual.
    off_axis_mag = math.sqrt(det_norm_x * det_norm_x + det_norm_y * det_norm_y)
    off_axis_mag = min(1.0, off_axis_mag)
    alpha_a = (1.0 - off_axis_mag) ** 2

    # alpha_C: fraction of gimbal capacity unused by aircraft rotation.
    if gimbal_has_readback:
        alpha_c = 1.0
    elif gimbal_slew_rate_dps <= 0:
        alpha_c = 0.0
    else:
        alpha_c = max(0.0, 1.0 - body_rate_abs_dps / gimbal_slew_rate_dps)

    return alpha_a * alpha_c
