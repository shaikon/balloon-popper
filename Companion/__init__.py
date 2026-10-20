# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""Vision balloon chaser package."""

# Public surface — what users of the package import.
# Adapters and UI are imported by main.py, not re-exported here.

from .controller import BalloonChaser
from .interfaces import AutopilotLink, GimbalDriver, BalloonSensor, Camera
from .control_laws import ControlGains

__all__ = [
    "BalloonChaser",
    "AutopilotLink", "GimbalDriver", "BalloonSensor", "Camera",
    "ControlGains",
]
