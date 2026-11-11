# core/physics/mujoco: history

## 2026-11-11: Round 4 close sweep

- Joint drive start-state spring (PHY-034, brief M2-C2): the old start-state spring (eq_data = rest c, d = 1 - 1 / (K_d A0 h^2)) left a stepper's rest at theta + omega h, standing omega h ahead of the joint, so its braking drive drove G10's x axis to 0.78 m/s under slip. Replaced by the end-state form under Joint drives; the derivation stays in the contract.
