Own-engine adapter: implements `mechsim::kernel::PhysicsAdapter` over `core/engine/` (`MultibodyProvider`,
`SemiImplicitEuler`), parsing the neutral model JSON and mapping `StepHealth` onto `PhysicsHealth`; `PreparedModel::engine_artifact` is empty here.
