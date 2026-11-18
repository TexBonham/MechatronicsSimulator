"""Native adapter (PHY-020): the single Python entry to the own engine's binding.

Invariant 5: ``mechsim._core_ext.engine`` is imported only from ``mechsim/physics/native/`` (and tests). Stepping
is the C++ ``NativeAdapter`` (``make_physics_adapter("native")``); this package translates the neutral model
(``translate``, ``prepare``) and re-exports the engine types tests drive directly.
"""

from mechsim.physics.native._engine import engine

MultibodyModel = engine.MultibodyModel
MultibodyProvider = engine.MultibodyProvider
Transmission = engine.Transmission
TransmissionType = engine.TransmissionType
TransmissionOutput = engine.TransmissionOutput
PowerFlow = engine.PowerFlow

__all__ = [
    "MultibodyModel",
    "MultibodyProvider",
    "PowerFlow",
    "Transmission",
    "TransmissionOutput",
    "TransmissionType",
]
