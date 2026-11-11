"""Viewer module (VIE): probes, event timeline, body geometry and the replay cursor, served through the facade
(VIE-014). Headless: no display library here in this milestone (invariant 9)."""

from mechsim.viewer.geometry import BodyGeometry, ShapeGeometry, body_geometry
from mechsim.viewer.probes import Probe, ProbeColumn, ProbeError, probe
from mechsim.viewer.replay import CursorState, ReplayCursor, ReplayError
from mechsim.viewer.timeline import TimelineEntry, category_of, timeline

__all__ = [
    "BodyGeometry",
    "CursorState",
    "Probe",
    "ProbeColumn",
    "ProbeError",
    "ReplayCursor",
    "ReplayError",
    "ShapeGeometry",
    "TimelineEntry",
    "body_geometry",
    "category_of",
    "probe",
    "timeline",
]
