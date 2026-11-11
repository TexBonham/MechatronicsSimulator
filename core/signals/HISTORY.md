# core/signals: history

## 2026-11-11: Round 4 close sweep

- Stepper loss of synchronism, rationale (SIG-033, brief M2-A3): without the brake drive, a shaft spinning past the motor was re-synced every tick with the tangent's rest moving along at theta + omega h, so the drive sustained the motion (X2's G7 runaway). The rule stays in the contract under Stepper.
- Brake slip rest (SIG-020, brief M2-C2): the old rest theta + omega h + alpha h^2 at K_b grew 2.7x per step at K_b A0 h^2 = 100; the Coulomb rest theta + omega h + s T_b / K_s with K_s = 1e-5 K_b replaced it. The rule stays in the contract under Brake drive bound.
