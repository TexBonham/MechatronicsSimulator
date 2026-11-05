"""Native adapter skeleton (PHY-020, a later brief): the single Python entry to the own engine's binding.

Invariant 5: ``mechsim._core_ext.engine`` is imported only from ``mechsim/physics/native/`` (and tests); this module
re-exports what the adapter will translate the neutral model into and nothing else yet.
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
