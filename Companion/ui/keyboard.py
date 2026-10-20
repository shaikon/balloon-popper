# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""
Keyboard + mouse handler for the OpenCV windows.

Thin dispatcher: maps keys to PUBLIC controller methods.  All controller-state
mutation (mode entry, align-counter setup, thread spawning, _reset_to_idle) lives
on the controller as public methods (enter_straight, enter_balloon_chaser, etc.).  This
file is purely key->action mapping + the arrow/controller-autotune routing.

Arrow keys are decoded from the FULL key code (`_arrow_direction`), never its
low byte, so no letter reads as an arrow.  Branch bodies are single calls to
controller methods, because that logic touches controller state and belongs
there.
"""
from __future__ import annotations
import logging

import cv2

from .controller_autotune import ControllerAutotune

log = logging.getLogger("balloon_chaser")

# Each arrow key as cv2.waitKeyEx reports it from each HighGUI backend: GTK's
# keyval (modifier state rides in the bits above 16, so only the low 16 bits are
# compared), Windows' virtual-key code shifted into bits 16 and up, and Qt's key
# code (modifier flags above bit 24 masked off).  The low byte alone is not
# enough: a GTK arrow's low byte is 'Q', 'R', 'S' or 'T', so Shift or Caps Lock
# would turn q, r, s and t into arrows.
_GTK_KEY_MASK = 0xFFFF
_QT_KEY_MASK = 0x01FFFFFF
_ARROWS = (
    # direction  GTK keyval  Windows    Qt
    ('left',     0xFF51,     0x250000,  0x01000012),
    ('up',       0xFF52,     0x260000,  0x01000013),
    ('right',    0xFF53,     0x270000,  0x01000014),
    ('down',     0xFF54,     0x280000,  0x01000015),
)
# The Gain Tuner's key queue carries arrows as these (GTK keyvals): out of the
# printable range it carries characters in, so the two cannot meet.
TUNER_ARROW_CODES = {'Left': 0xFF51, 'Up': 0xFF52, 'Right': 0xFF53, 'Down': 0xFF54}


def _arrow_direction(raw_key):
    """'up' / 'down' / 'left' / 'right' for an arrow key's full code, else None."""
    for direction, gtk, win, qt in _ARROWS:
        if ((raw_key & _GTK_KEY_MASK) == gtk or raw_key == win
                or (raw_key & _QT_KEY_MASK) == qt):
            return direction
    return None


class KeyboardHandler:
    """Routes key/mouse events to the controller and controller-autotune."""

    def __init__(self, controller, controller_autotune: ControllerAutotune):
        self._c = controller
        self._controller_autotune = controller_autotune

    def gimbal_mouse_callback(self, event, x, y, flags, param):
        """Capture mouse position inside the Gimbal Camera window."""
        if event == cv2.EVENT_MOUSEMOVE:
            self._c.set_gimbal_mouse(x, y)

    def handle_key(self):
        """Process one key from the OpenCV windows.

        Modes (all post-launch modes use ACRO; catapult-launch is the only FBWA
        path): loiter / straight / balloon chaser / manual / ptest.  Mode-to-mode
        transitions are unconditional.  Overlay keys (g/l/q/r) work in any mode.
        """
        # cv2.waitKey ALWAYS runs: it is what pumps the HighGUI event loop, so
        # skipping it would freeze any open camera window even when the key came
        # from elsewhere.
        raw_key = cv2.waitKeyEx(1)
        if raw_key == -1:
            # No OpenCV window has a key for us - which is the normal case when
            # the camera views are switched off, since waitKey can only deliver
            # keys to an OpenCV window.  Fall back to the Gain Tuner, which is
            # already open and queues keystrokes for exactly this reason.
            tuner = self._controller_autotune
            raw_key = tuner.take_key() if tuner is not None else -1
            if raw_key == -1:
                return
        key = raw_key & 0xFF

        # Arrow keys: incremental virtual-balloon adjustment (-> manual mode).
        arrow_direction = _arrow_direction(raw_key)
        if arrow_direction is not None:
            self._c.handle_arrow(arrow_direction)
            return

        # Let the controller autotune consume the key first (digits, enter, escape, backspace).
        if self._controller_autotune.handle_key(key):
            return

        # Backtick - reset all controller-autotune params to defaults.
        if key == ord('`'):
            self._controller_autotune.reset_to_defaults()
            return

        # Mode keys.
        if key == ord('s'):
            self._c.toggle_straight()
        elif key == ord('x'):
            self._c.toggle_balloon_chaser()
        elif key == ord('p'):
            self._c.toggle_ptest()
        elif key == ord('t'):
            self._c.toggle_autotune()
        elif key == ord('y'):
            self._c.toggle_cell_autotune()
        elif key == ord('i'):
            self._c.toggle_plant_id()
        elif key == ord('k'):
            self._c.toggle_freq_id()
        elif key == ord('e'):
            self._c.toggle_trim_map()
        # Overlay keys.
        elif key == ord('g'):
            self._c.toggle_gimbal_lock()
        elif key == ord('l'):
            self._c.toggle_vision_tracking()
        elif key == ord('q'):
            self._c.toggle_csv_logging()
        elif key == ord('r'):
            self._c.reset_gimbal_center()
        elif key == ord('c'):
            self._c.run_gimbal_calibration()
