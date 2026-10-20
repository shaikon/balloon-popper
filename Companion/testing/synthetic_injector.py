# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
SyntheticBalloonInjector - anchors a world-space balloon for control-law testing.

Source: SyntheticBalloonInjector 852-977.

REFACTOR: the source poked controller internals directly
(ctrl._injected_world_pos = ...).  Now it talks to the BalloonSensor's oracle
interface (set_oracle_world_position / set_oracle_world_direction), and reads
aircraft state through the AutopilotLink.  No controller-internal access.

The anchoring math (longitude/latitude -> world dir -> ground/sky decision -> NED
anchor) feeds the result to sensor.set_oracle_*().  WHERE the balloon sits is a
LONGITUDE / LATITUDE bearing, deliberately not called pan/tilt: pan and tilt are
reserved throughout this codebase for MOTION or DESIRED motion, and a balloon's
anchor is a position.  Longitude runs around the aircraft (0 ahead, +90 right,
180 astern); latitude runs up and down (+90 zenith, 0 level, -90 nadir).  It is
NOT WGS84 - the aircraft's real geographic position is link.lat / link.lon and is
a different thing entirely.

The bearing -> direction conversion is the SAME spherical map the real sensor
uses (geometry.pan_tilt_to_vector - the gimbal-encoder body direction), so the
test path exercises the real system's geometry, not a private one.  The world
direction uses a MIXED reference (roll & pitch ignored): LATITUDE referenced to
the world horizon (gravity), LONGITUDE referenced to the aircraft's current
heading (so longitude 0 = straight ahead, no spurious turn).  Every (lon, lat) is
a valid direction over the FULL sphere (C-04) - nothing is clamped or rejected,
and astern is reachable either way (lon=180,lat=0 around the side, or
lon=0,lat=180 over the top).  The error-logging thread reads the steering errors
through a small callback the test provides (the controller exposes them), keeping
the injector decoupled from controller internals.
"""
from __future__ import annotations
import logging
import math
import threading
import time

import numpy as np

from ..geometry import (pan_tilt_to_vector, euler_to_rotation_matrix,
                        wrap180)

log = logging.getLogger("balloon_chaser")


class SyntheticBalloonInjector:
    """Anchors a balloon in world space and feeds it to a BalloonSensor's oracle.

    WHERE the balloon sits is given as a LONGITUDE / LATITUDE bearing.  These name
    a POSITION, which is why they are not called pan/tilt: in this codebase pan
    and tilt describe MOTION or DESIRED motion (the nose-to-balloon error the
    cascade closes, the gimbal's aim, the balloon's direction of travel), never
    where something is.

      LONGITUDE  around the aircraft.  0 straight ahead, +90 right, 180 astern,
                 -90 left.  Referenced to the aircraft's HEADING at the moment of
                 injection, so 0 means "straight ahead" whatever compass course
                 you happened to be flying.
      LATITUDE   up/down.  +90 straight up, 0 level with the horizon, -90 straight
                 down.  Referenced to the WORLD horizon (gravity), so it means the
                 same thing whatever attitude the aircraft is in.

    NOT WGS84.  This is a bearing on a sphere centred on the aircraft, sharing the
    familiar globe convention only by analogy.  The aircraft's real geographic
    position lives on the link (link.lat / link.lon) and is a different thing.

    On start() the pair is converted to a world-frame unit direction with the SAME
    spherical map the real sensor uses (geometry.pan_tilt_to_vector), under a
    MIXED reference that deliberately ignores roll & pitch: LATITUDE is world-
    referenced, LONGITUDE heading-referenced.  EVERY (lon, lat) pair is a valid
    direction - the full sphere, with no restricted region, no budget coupling,
    nothing clamped or rejected (C-04).  Both are full-range rotation angles and
    either can reach any hemisphere: astern is lon=180,lat=0 OR lon=0,lat=180
    (the same point); behind-and-up 45 is lon=180,lat=+45 or lon=0,lat=+135.  The
    one intrinsic degeneracy is the pole (lat=+-90 = zenith/nadir), where
    longitude has no effect because a heading is meaningless pointed straight up
    or down.  Then:

    HOW FAR along that bearing the balloon sits is decided by ONE rule, with no
    modes and no special cases:

        r = min(init_range, ground-intersection distance)

    The ground-intersection distance is relative_alt / down-component for a
    bearing that descends, and infinite for one that does not.  So:

      - A finite init_range places the balloon there, floating in the air if the
        ground is farther away than that.
      - A range past the ground is CAPPED to it: a balloon cannot start
        underground.
      - inf is the largest possible range, not an exception.  Descending, the
        cap lands it on the ground, altitude-adaptive with no number to type.
        Level or upward, nothing caps it, so it is a DIRECTION anchor: a fixed
        world bearing the aircraft can never reach.

    `speed` controls only whether the balloon TRANSLATES, never where it starts.
    Motion is ignored while the range is infinite - nothing at infinity has a
    position to move - and the operator's switch and speed are left set, so they
    apply once the range is finite.  The travel direction is a separate
    (vel_pan, vel_tilt) bearing; that one is motion, hence the pan/tilt naming.

    start() returns True on success, False on precondition failure.
    """
    _HZ = 50

    def __init__(self, link, sensor, lon_deg, lat_deg,
                 speed=0.0, init_range=float('inf'),
                 vel_pan_deg=0.0, vel_tilt_deg=0.0, move_enable_fn=None,
                 speed_fn=None, vel_dir_fn=None, force_direction=False):
        """
        Args:
            link:               AutopilotLink (attitude, position, alt).
            sensor:             BalloonSensor with the oracle interface.
            lon_deg, lat_deg:   WHERE to anchor the balloon - spherical longitude
                                (0 ahead, +90 right, 180 astern) and latitude
                                (+90 up, 0 level, -90 down).  See the class
                                docstring; not WGS84.  ANY pair is valid (full
                                sphere); both wrap to (-180, +180] for display.
                                This is a world-space direction for the oracle,
                                NOT a servo command, so the gimbal's travel limits
                                do not bound it.
            speed:              m/s.  0 -> stationary.  > 0 -> translates in a
                                straight line along (vel_pan, vel_tilt).  Does
                                not affect where the balloon starts; ignored while
                                init_range is infinite.  Only the STARTING speed
                                when speed_fn is supplied - see speed_fn.
            init_range:         m along the (lon, lat) bearing, or float('inf').
                                The balloon sits at min(init_range, ground
                                distance) - see the class docstring.  A finite
                                result needs GPS lock + above-home; a direction
                                anchor needs neither.
            force_direction:    Anchors a pure DIRECTION whatever the geometry,
                                ignoring init_range and the ground.  For the
                                MEASUREMENT rigs - the autotune sweep and
                                plant-ID.  Both run cells to lat=-90, which would
                                otherwise be reachable ground points: the sweep
                                flies THROUGH them, corrupting settle and jitter,
                                and plant-ID picks up the parallax of a near
                                balloon as if it were bank authority.  A direction
                                anchor also needs no GPS fix or above-home
                                altitude, so a descending cell cannot fail its
                                precondition.  Leave False for the ptest, which
                                keeps one distance rule.
            vel_pan_deg,
            vel_tilt_deg:       WHICH WAY a moving balloon travels.  Still called
                                pan/tilt because it describes MOTION, not position.
                                Same spherical frame as the lon/lat anchor bearing
                                (pan heading-relative, tilt horizon-referenced),
                                frozen at start().  vel_tilt 0 = level.
            move_enable_fn:     callable() -> bool, polled LIVE, or None.  The
                                master on/off for translation.
            speed_fn:           callable() -> m/s, polled LIVE, or None.  Lets the
                                operator change POINT_BALLOON_SPEED mid-run; the
                                balloon continues from where it is at the new speed
                                (see set_speed).  None (the sweep / plant-ID) keeps
                                `speed` frozen for the whole run, which is what
                                makes those runs bit-repeatable.  The direction of
                                travel is frozen at start() either way, so a live
                                speed change never moves the balloon off its line.
        """
        self._link = link
        self._sensor = sensor
        self._force_direction = bool(force_direction)
        # Simple linear motion (constant speed).  Frozen at start() into a fixed
        # world-frame line, so the trajectory is deterministic and repeatable.
        self._speed = float(speed)
        self._init_range = float(init_range)
        self._vel_pan = wrap180(float(vel_pan_deg))
        self._vel_tilt = wrap180(float(vel_tilt_deg))
        # _moving  = this is a POSITION anchor, so a position is published every
        #            tick and motion is possible.  False for a DIRECTION anchor
        #            (a balloon at infinity cannot translate).
        # _move_on = whether it is actually translating right now.  Live: the
        #            operator flips POINT_BALLOON_MOVE_ENABLE mid-test and
        #            move_enable_fn reports it (see set_moving).
        self._moving = False
        self._move_enable_fn = move_enable_fn
        self._speed_fn = speed_fn
        # Polled LIVE, exactly like speed_fn.  Returns (pan_deg, tilt_deg).
        # Polling it live (not freezing it at injection) is what lets the bearing
        # slider move the BALLOON mid-run rather than the tuner alone.
        self._vel_dir_fn = vel_dir_fn
        # Initial state comes from the LIVE switch when one was supplied (the
        # ptest), else from speed alone (the sweep / plant-ID, which pass neither
        # and must stay stationary).  Reading the switch here rather than having
        # the caller zero the speed is what lets a run that STARTS stationary be
        # set moving later: the speed is already loaded, just not applied.
        if move_enable_fn is not None:
            self._move_on = bool(move_enable_fn()) and self._speed > 0.0
        else:
            self._move_on = self._speed > 0.0
        self._vel_dir = None           # unit world NED velocity direction (frozen at start)
        # THE CURRENT MOTION LEG, as ONE immutable tuple:
        #     (pos0, vel_world, vel_ground, t0)
        #     pos(t) = pos0 + vel_world * (t - t0)      [see _position_at]
        #
        # Held as a single tuple rather than four attributes because the leg is
        # read from the CONTROL thread (the sensor evaluates position_now() once
        # per tick) and written from the injector thread (a move/speed change
        # starts a new leg).  Four separate stores are not atomic together: a
        # reader landing mid-write could pair the NEW pos0 with the OLD t0 and
        # compute a position tens of metres out for one tick - a step straight
        # into the roll law, which is exactly the class of glitch this module's
        # design notes exist to prevent.  One tuple store IS atomic in CPython,
        # so a reader sees either the whole old leg or the whole new one.
        # _position_at snapshots it into a local before doing any arithmetic.
        self._leg = (None, None, None, None)
        self._running = False
        self._thread = None
        # Every (lon, lat) is valid, so there is nothing to clamp, reject, or
        # verify - just wrap to (-180, +180] for tidy logging.  The direction is
        # built from these (wrapping is direction-preserving) in start().
        self._lon = wrap180(float(lon_deg))
        self._lat = wrap180(float(lat_deg))

    @property
    def lon(self):
        """Anchored LONGITUDE, deg in (-180, +180].

        Where the balloon IS, measured around the aircraft: 0 straight ahead,
        +90 right, 180 astern, -90 left.  Not a WGS84 coordinate - this is a
        bearing on a sphere centred on the aircraft."""
        return self._lon

    # ---- current-leg accessors (read-only views onto the _leg snapshot) -----
    # Read-only on purpose: the leg must only ever change through _begin_leg,
    # which swaps all four values in one atomic tuple store.  Assigning one of
    # these individually would reintroduce exactly the torn read the tuple exists
    # to prevent.
    @property
    def _pos0(self):
        """World NED start point of the current leg."""
        return self._leg[0]

    @property
    def _vel_world(self):
        """World NED velocity of the current leg (zero while motion is off)."""
        return self._leg[1]

    @property
    def _vel_ground(self):
        """Full-speed horizontal slide velocity used once the balloon lands."""
        return self._leg[2]

    @property
    def _t0(self):
        """Monotonic start time of the current leg."""
        return self._leg[3]

    @property
    def lat(self):
        """Anchored LATITUDE, deg in (-180, +180].

        How high the balloon sits: +90 straight up, 0 level, -90 straight down.
        Values beyond +-90 are valid over-the-top bearings (lat=180 is astern,
        the same direction as lon=180, lat=0).  Not a WGS84 coordinate."""
        return self._lat

    def start(self):
        """Anchor balloon in world space.  Returns True on success, False on
        precondition failure (caller aborts the test cleanly)."""
        # Mixed reference frame, deliberately ignoring the aircraft's roll & pitch
        # ("imagine it's flying straight & level" at the instant of injection):
        #   - LATITUDE is WORLD-frame: its downward component lies along WORLD down
        #     (gravity), so the dive angle is referenced to the world horizon,
        #     independent of how the aircraft is pitched/banked.
        #   - LONGITUDE is referenced to the aircraft's current HEADING (yaw):
        #     longitude 0 means "straight ahead", so a lon-0 balloon commands NO
        #     turn regardless of
        #     which way the nose happened to point when the test started. (In world
        #     frame, longitude 0 would mean due North and force a spurious turn.)
        # Implementation: build the lon/lat direction (x=forward, y=right, z=down)
        # with the same spherical map the real sensor uses, then rotate it about
        # world-down by the HEADING ONLY (roll=pitch=0).  Yaw rotation leaves the
        # down (z) component untouched, so latitude stays world-referenced while
        # longitude becomes heading-relative.  Every (lon, lat) is valid, so
        # there is nothing to clamp or check here.
        _, _, yaw = self._link.attitude
        R_heading = euler_to_rotation_matrix(0.0, 0.0, yaw)   # heading only
        v = pan_tilt_to_vector(self._lon, self._lat)
        world_dir = R_heading.T @ v   # heading frame -> world NED

        # Velocity DIRECTION is frozen here for the whole run, in the same mixed
        # frame as the bearing.  Only the on/off state changes later (set_moving),
        # so every leg of a toggled run runs along one fixed world-frame line.
        # Kept so a LIVE bearing change resolves in the same mixed frame this
        # one did.  Re-reading the aircraft's heading later would silently
        # redefine what the bearing means halfway through a run.
        self._R_heading = R_heading
        self._vel_dir = R_heading.T @ pan_tilt_to_vector(self._vel_pan, self._vel_tilt)

        # HOW FAR:  r = min(init_range, ground-intersection distance).
        # The ground bounds every range, so an infinite one is just the largest.
        ac_pos = None
        relative_alt = None
        d_ground = float('inf')
        if self._force_direction:
            # Sweep only - see the constructor docstring.  Bypasses the rule,
            # so it needs no GPS fix.
            r = float('inf')
        else:
            descending = bool(world_dir[2] > 1e-6)
            # A finite point needs the aircraft's world position; the ground
            # needs its height above home.  A direction anchor needs neither.
            # Required on a descending bearing even for a finite range: without
            # the altitude the cap cannot fire, and the balloon would be buried.
            if descending or math.isfinite(self._init_range):
                ac_pos = self._link.aircraft_world_pos_ned()
                if ac_pos is None:
                    log.warning(
                        "[PTEST] No GPS lock; cannot place a balloon at a finite "
                        "range or work out where the ground is.")
                    return False
                relative_alt = self._link.relative_alt
                if relative_alt <= 0.0:
                    log.warning(
                        "[PTEST] Aircraft not above home altitude "
                        "(rel_alt=%.1f m); cannot anchor balloon.", relative_alt)
                    return False
                if descending:
                    # Ground at down=0 in NED.  Ray: ac_pos + t*world_dir, with
                    # ac_pos.down = -relative_alt.  Solve -relative_alt +
                    # t*world_dir[2] = 0.
                    d_ground = relative_alt / float(world_dir[2])
            r = min(self._init_range, d_ground)
            if r < self._init_range:
                # A finite request that could not be honoured is a warning.
                # Capping inf is the rule working normally on every press, so it
                # is only info.
                if math.isinf(self._init_range):
                    log.info("[PTEST] range=inf on a descending bearing -> ground "
                             "at %.0f m (rel_alt %.0f m).", r, relative_alt)
                else:
                    log.warning(
                        "[PTEST] init_range %.0f m along this bearing is below "
                        "ground (ground is %.0f m away at rel_alt %.0f m); capping "
                        "the balloon to the ground.",
                        self._init_range, r, relative_alt)

        # ---- WHAT that resolves to: a bearing, or a point ------------------
        if math.isinf(r):
            # DIRECTION anchor: a fixed world bearing, never reached, so the
            # aircraft holds a steady constant-angle attitude on it (a sustained
            # lock) instead of overflying anything.
            self._sensor.set_oracle_world_direction(np.asarray(world_dir, dtype=float))
            log.info("[PTEST] DIRECTION balloon anchored at world bearing "
                     "(%.3f, %.3f, %.3f)  [range=inf, elev=%+.1f deg below "
                     "horizon]", world_dir[0], world_dir[1], world_dir[2],
                     math.degrees(math.asin(max(-1.0, min(1.0, world_dir[2])))))
            # Nothing at infinity has a position to translate, so motion is
            # ignored as if the switch were off.  The operator's cells are left
            # set and apply once the range is finite.  Logged loudly: with the
            # inf default this is the common case, and silence would read as a
            # bug.
            if self._move_on:
                log.warning(
                    "[PTEST] balloon motion IGNORED: the range is infinite, so "
                    "this balloon is a direction and has no position to move. "
                    "Set a finite Distance m to make it move (speed %.1f m/s is "
                    "kept and will apply then).", self._speed)
                self._move_on = False
        else:
            # POSITION anchor at r metres.  Always begins a motion leg: with
            # _move_on False the velocity is zero, so it republishes the same
            # point, and set_moving can start it mid-test from where it sits.
            pos0 = np.asarray(ac_pos, dtype=float) + world_dir * r
            self._begin_leg(pos0, moving=True)
            if self._move_on:
                log.info("[PTEST] MOVING balloon: start NED (%.1f, %.1f, %.1f) "
                         "range=%.1f m; velocity (%.2f, %.2f, %.2f) m/s = %.1f m/s "
                         "along vel bearing (pan=%+.1f, tilt=%+.1f)",
                         pos0[0], pos0[1], pos0[2], r,
                         self._vel_world[0], self._vel_world[1], self._vel_world[2],
                         self._speed, self._vel_pan, self._vel_tilt)
            else:
                log.info("[PTEST] STATIONARY balloon anchored at NED "
                         "(%.1f, %.1f, %.1f) range=%.1f m%s",
                         pos0[0], pos0[1], pos0[2], r,
                         " (on the ground)" if r == d_ground else "")

        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="synth-inject")
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._sensor.clear_oracle()

    def _begin_leg(self, pos, moving):
        """Start a motion leg at `pos`: rebase the closed form and publish.

        pos(t) = pos0 + vel_world * (t - t0).  Every change of motion state
        starts a NEW leg from the CURRENT position, so position is continuous
        across a toggle and the closed form stays exact within each leg (no
        integration drift, which is the property the whole design rests on).

        All four leg values are swapped in ONE tuple store so a control-thread
        reader can never see a half-updated leg (see the note in __init__)."""
        pos0 = np.asarray(pos, dtype=float)
        vel_world = self._vel_dir * (self._speed if self._move_on else 0.0)
        # Full-speed horizontal velocity for sliding once it reaches the flat
        # ground: the velocity's compass heading kept, re-scaled to the whole
        # speed so the ground track runs at the configured speed (not just the
        # horizontal slice of a dive).  Purely-vertical motion has no heading,
        # so it lands and stays put.
        vh = vel_world.copy()
        vh[2] = 0.0
        nh = float(np.linalg.norm(vh))
        vel_ground = (vh / nh * self._speed) if nh > 1e-9 else np.zeros(3)
        self._leg = (pos0, vel_world, vel_ground, time.monotonic())
        self._moving = moving
        self._publish_position()

    def _publish_position(self):
        """Hand the balloon's world position to the sensor's oracle.

        PREFERRED (pull): hand over position_now, which the sensor evaluates ONCE
        per control tick.  The balloon is then exact at the instant the controller
        uses it - no staleness at all.  This matters because the injector's own
        thread and the 50 Hz control loop are independent free-running timers: a
        pushed snapshot was measured unchanged on ~31% of control ticks (and
        double-stepped on another ~33%), which both mistimed the balloon and fed
        the closest-approach fit a sequence that was not the quadratic it assumes.

        FALLBACK (push): a sensor without the pull hook gets a snapshot, exactly
        as before, and the injector thread keeps refreshing it.  Kept so any
        BalloonSensor implementation (and the test fakes) still works."""
        if hasattr(self._sensor, 'set_oracle_position_fn'):
            self._sensor.set_oracle_position_fn(self.position_now)
        else:
            self._sensor.set_oracle_world_position(self._position_at(
                time.monotonic() - self._leg[3]))

    def position_now(self):
        """The balloon's TRUE world NED position at this instant.

        Safe to call from any thread: it snapshots the leg tuple once and works
        from the closed form, so it neither blocks the caller nor can observe a
        partially-updated leg."""
        pos0, _, _, t0 = self._leg
        if pos0 is None:
            return None
        return self._position_at(time.monotonic() - t0)

    def _rebase(self, on, speed, vel_pt=None):
        """Adopt a new (motion on/off, speed) from the CURRENT position.

        The single leg boundary both set_moving and set_speed go through.  The
        balloon keeps its position and continues from there, so neither control
        can ever teleport it, and each leg stays an exact closed form.

        Called only from the injector thread (the _loop poll, or a test driving
        by hand), so _leg/_move_on/_speed have one writer and need no lock."""
        pt_same = vel_pt is None or (vel_pt[0] == self._vel_pan
                                     and vel_pt[1] == self._vel_tilt)
        if on == self._move_on and speed == self._speed and pt_same:
            return True                          # nothing changed
        was_on, was_speed = self._move_on, self._speed
        was_pt = (self._vel_pan, self._vel_tilt)
        here = self._position_at(time.monotonic() - self._leg[3])
        self._move_on, self._speed = on, speed
        if not pt_same:
            # Through the SAME leg boundary as speed: the balloon keeps its
            # position and turns from there, so a bearing change can never
            # teleport it and each leg stays an exact closed form.
            self._vel_pan, self._vel_tilt = wrap180(vel_pt[0]), wrap180(vel_pt[1])
            self._vel_dir = self._R_heading.T @ pan_tilt_to_vector(
                self._vel_pan, self._vel_tilt)
        self._begin_leg(here, moving=True)
        if on != was_on:
            log.info("[PTEST] balloon motion %s at NED (%.1f, %.1f, %.1f), "
                     "speed %.1f m/s", "RESUMED" if on else "PAUSED",
                     self._leg[0][0], self._leg[0][1], self._leg[0][2], speed)
        if not pt_same:
            log.info("[PTEST] balloon bearing/elev %.1f/%.1f -> %.1f/%.1f deg"
                     "%s", was_pt[0], was_pt[1], self._vel_pan, self._vel_tilt,
                     "" if on else " (motion is OFF, so not applied yet)")
        if speed != was_speed:
            log.info("[PTEST] balloon speed %.1f -> %.1f m/s at NED "
                     "(%.1f, %.1f, %.1f)%s", was_speed, speed,
                     self._leg[0][0], self._leg[0][1], self._leg[0][2],
                     "" if on else " (motion is OFF, so not applied yet)")
        return True

    def set_moving(self, on):
        """Turn balloon motion on/off DURING a run (the live POINT_BALLOON_MOVE_ENABLE
        switch).  Returns True if the request applies to this balloon.

        The balloon keeps its current position and starts/stops from there, so
        the switch never teleports it.  Direction-anchored balloons (sky bearings)
        cannot move at all - a balloon at infinity has no position to translate -
        so this reports False and changes nothing."""
        if not self._moving:
            return False
        return self._rebase(bool(on), self._speed)

    def set_speed(self, speed):
        """Change the balloon's speed DURING a run (the live POINT_BALLOON_SPEED
        cell).  Returns True if the request applies to this balloon.

        Same leg-boundary mechanism as set_moving - and for the same reason.
        Pausing IS a speed change (to zero), so treating a speed edit as
        anything more exotic would have been the inconsistency.  What stays
        frozen at start() is the DIRECTION of travel (_vel_dir), which is what
        makes the run repeatable: every leg of a run, at whatever speed, lies
        along the one world-frame line chosen at injection.  Speed only sets how
        fast the balloon travels along it.

        A speed set while motion is OFF is stored and takes effect on resume -
        which is what lets a run pressed with speed 0 be given a speed and then
        started, instead of reporting 'motion RESUMED' and sitting still."""
        if not self._moving:
            return False
        return self._rebase(self._move_on, float(speed))

    def _position_at(self, elapsed):
        """World NED position of the moving balloon at `elapsed` seconds.  A
        straight line at constant speed while airborne; once the ideal line
        reaches the flat ground (down=0) the balloon keeps sliding along the
        surface at the SAME full speed in the velocity's horizontal heading (the
        dive's vertical part is redirected into horizontal, so the ground track
        runs at the configured speed, not just the horizontal slice of the dive).
        Ground assumed flat at down=0, matching the anchor and the Gazebo map.

        Reads the leg ONCE into locals: this is called from the control thread
        (via position_now) as well as the injector thread, and every value below
        must come from the SAME leg."""
        pos0, vel_world, vel_ground, _ = self._leg
        pos = pos0 + vel_world * elapsed
        if pos[2] <= 0.0:
            return pos                          # at or above ground: straight line
        vz = vel_world[2]
        if vz <= 1e-9:                          # not actually descending: clamp
            pos[2] = 0.0
            return pos
        t_hit = -pos0[2] / vz                   # time the ideal line reaches down=0
        touch = pos0 + vel_world * t_hit
        pos = touch + vel_ground * (elapsed - t_hit)
        pos[2] = 0.0                            # stay exactly on the surface
        return pos

    def _loop(self):
        """Record nose-to-balloon errors for scoring (the angles the controller
        is closing), and poll the LIVE operator controls (move on/off, speed).

        POSITION is not pushed from here: the sensor holds position_now and
        evaluates it once per control tick, so the controller sees the balloon's
        exact position at the instant it uses it.  See _publish_position for the
        fallback when a sensor has no pull hook."""
        nxt = time.monotonic()
        push_fallback = not hasattr(self._sensor, 'set_oracle_position_fn')
        while self._running:
            if self._moving:
                # Live operator controls, polled HERE so every mutation of the
                # leg happens on this one thread.  Both are read first and
                # applied together, so a tick that changes both costs one leg
                # boundary, not two.  _rebase is a no-op when nothing changed.
                try:
                    on = (bool(self._move_enable_fn()) if self._move_enable_fn
                          is not None else self._move_on)
                    speed = (float(self._speed_fn()) if self._speed_fn
                             is not None else self._speed)
                    vel_pt = (self._vel_dir_fn() if self._vel_dir_fn
                              is not None else None)
                    self._rebase(on, speed, vel_pt)
                except Exception:
                    log.exception("[PTEST] live control poll failed")
                if push_fallback:
                    # No pull hook on this sensor: keep publishing the true
                    # position every tick.  There is deliberately no close-range
                    # hold - the injector is the test's source of TRUTH, and a
                    # hold made it publish a position it knew to be stale (by
                    # speed x hold-time), then teleport on release.  The genuine
                    # singularity (exact coincidence) is handled where it
                    # belongs, in oracle_world_dir's n < 1e-6 branch.
                    self._sensor.set_oracle_world_position(
                        self._position_at(time.monotonic() - self._leg[3]))
            nxt += 1.0 / self._HZ
            slp = nxt - time.monotonic()
            if slp > 0:
                time.sleep(slp)
            else:
                nxt = time.monotonic()
