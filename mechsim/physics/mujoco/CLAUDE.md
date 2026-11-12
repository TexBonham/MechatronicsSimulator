# mechsim/physics/mujoco/: contract

Translation half of the MuJoCo bridge (ADR 0014): neutral model to MJCF text, build time only. Stepping is
`core/physics/mujoco/`. The `mujoco` Python package is not a dependency; tests check the XML structure.

| Name | Does |
|---|---|
| `translate(model, step=None, payloads=None) -> str` | MJCF text, byte-identical for the same inputs; `step` (s) overrides `run.step`; `payloads` maps payload body to carrier body |
| `translate_report(model, step=None, payloads=None) -> Translation(mjcf, report)` | Same text plus the mapping report: one line per approximation or omission |
| `supported_elements()` | Element kinds mapped; mirrors `supported_kinds` in `core/physics/mujoco/src/mujoco_adapter.cpp` (test-checked) |
| `UnmappableElementError(element, reason)` | Raised for anything without an exact mapping, naming the element (PHY-017, PHY-019) |

## Mapping

| Neutral | MJCF |
|---|---|
| Body | `<body>` at the joint's position/orientation in the parent; `<inertial>` com, mass, `fullinertia`; `inertiafromgeom=false` |
| revolute / prismatic | `hinge` / `slide`; limits `range` (both bounds), friction `frictionloss`, `damping`, `stiffness`, rest `springref` |
| fixed | no joint (welded) |
| planar | slides `.x`, `.y` along the engine's plane basis u, w of the axis, then hinge `.theta` |
| spherical / free | `ball` / `free`; damping and friction must be uniform over the DOFs; free only under world |
| sphere, capsule, box, cylinder, plane | geoms of the same kind (half sizes); plane only on world |
| convex_hull | `<mesh vertex=...>` asset, geom `mesh` |
| Contacts | all geoms `contype=conaffinity=0`; one `<pair>` per geom pair on different, non-excluded bodies not joined by a non-free joint (skips reported): friction mu, `solref = (-K/m, -C/m)` with m the bodies' reduced mass (world infinite), `solimp` 0.99 |
| point constraint | `<connect>` between sites at both anchors |
| hinge constraint | two connects: anchors, and anchors + 0.1 m along each unit axis |
| Lumped member | engine midpoint-spring chain (ENG-022): segments `<m>.segment<j>` with hinges `<m>.node<j>` (k = E I / l, node damping) at x_j = (j + 1/2) l; pinned root adds `<m>.pin` (free hinge, span l/2); roller end adds `<m>.roller` slide body and a connect from the tip |
| belt_pulley, lead_screw, gear_pair | reflection, driven = c * shaft (c = r, lead / 2 pi, 1 / ratio): shaft body not emitted; driven joint gains armature J / c^2 (J about the shaft axis), damping D / c^2, frictionloss f / abs(c); efficiency applied by the C++ adapter; stiffness treated as rigid (reported); shaft mass, off-axis inertia and shapes not carried (reported) |
| gravity, step, iteration cap | `<option gravity timestep iterations integrator=implicitfast>` |
| transmission backlash b | play band b / abs(c) on the shaft's drive rest, numeric [13]; applied by the C++ adapter; reported (no impact dynamics, torque-driven shafts pass without play) |
| payload (`payloads` argument) | free body under world plus `<weld name="<payload>.attach" body1=payload body2=carrier>` at equality softness, active; `mechsim:payloads` entry; reported |
| Equality and limit softness | `solref = (2 step, 1)`, MuJoCo's stability floor |
| Structural members, signals | not physics: emit nothing |

Unmappable: one-sided joint limits, non-uniform ball/free damping or friction, free joint below a body, a static
transmission slip parameter (slip is the injected fault, PHY-006), self-locking, a payload that is not a free body
under world or is a shaft, a payload carrier that is world, itself, unknown or a shaft, a shaft body that carries a body, member or constraint, a spring or limit on a shaft
joint, a shaft joint that is itself driven, one shaft for two transmissions, any unknown joint, shape, constraint or
transmission type.

Known limits: contact `solref` sets the contact's dynamic response, not its static penetration (MuJoCo's impedance
does); joint stiffness is explicit under implicitfast, so a stiff member node with small segment inertia is unstable
at 1 ms (`omega h > 2`) and ends the run (invariant 7).

## Metadata `<custom>`

`mechsim:elements` (kinds present), `mechsim:bodies`, `mechsim:joints` (`name:type`), `mechsim:constraints`
(`name:type`), `mechsim:members` (`name:parent:root:end:segments`), `mechsim:transmissions`
(`name:shaft_body:parent:driven_joint`), `mechsim:payloads` (`payload:carrier`), numeric `mechsim:member:<name>` = [l, position, orientation], numeric
`mechsim:transmission:<name>` = [c, efficiency, J, joint position, orientation, unit axis, backlash / abs(c)]. The C++ adapter reads only these and MuJoCo's own arrays.
Generated MJCF names join a neutral name and a suffix with `.`, which neutral names cannot contain.

## Section index

| Section | File | Line |
|---|---|---|
| Joints | translate.py | 138 |
| Shapes | translate.py | 223 |
| Lumped members | translate.py | 255 |
| Constraints and transmissions | translate.py | 336 |
| Payloads | translate.py | 540 |
| Contacts | translate.py | 575 |
| Model | translate.py | 642 |
