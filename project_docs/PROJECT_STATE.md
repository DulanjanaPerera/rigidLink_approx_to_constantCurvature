# Project State

**Last updated:** 2026-10-21  
**Current milestone:** Pressure-driven rigid-link model with distributed gravity compensation and closed-loop bending control

## Start Here

This is the authoritative summary of the project's current state. At the start of a new Codex session, read this file first, then open only the specialized documents relevant to the task:

- `MODEL_ARCHITECTURE.md` for the current signal flow and frame definitions.
- `DECISIONS.md` for accepted modeling choices and their rationale.
- `OPEN_ISSUES.md` for unresolved work.
- `VALIDATION_RESULTS.md` for observed behavior and numerical checks.
- `TECHNICAL_NOTES.md` for equations and implementation details.
- `PROJECT_LOG.md` only when historical context is needed; it is append-only.

After significant modeling, control, or validation work, update this file in place and append a dated entry to `PROJECT_LOG.md`. Update the specialized document affected by the work. Do not add log entries for trivial file edits.

## Objective

Approximate a single constant-curvature continuum-arm module with a serial rigid-link model in MATLAB/Simulink/Simscape. Three pneumatic muscles generate a bending moment at the module tip. The model converts pressures or configuration commands into rigid-joint torques while preserving the continuum arm's transformation order, pressure limits, gravity response, and measured bending behavior.

## Current Mechanical Model

- Continuum length: `L = 0.29 m`.
- Physical representation: 30 disks and 29 rigid-link intervals (`N = 29`).
- Each interval uses four revolute transformations:
  1. `Rz(theta)`
  2. `Ry(phi/(2N))`
  3. translation `Tz(L/N)`
  4. `Ry(phi/(2N))`
  5. `Rz(-theta)`
- The rigid model therefore contains 116 revolute joints: 58 theta-axis joints and 58 phi-axis joints.
- The paired theta joints are position-controlled with opposite signs to select the bending plane without accumulating twist.
- The paired phi joints are torque-controlled. The current simplified multi-torque arrangement calculates 29 values, one for the first phi joint in each interval, and applies each value to both phi joints of that interval. Exact per-joint gravity compensation would require 58 independently calculated phi torques.

## Current Actuation and Control Direction

The intended next architecture is pressure-driven:

1. Three muscle pressures are the primary command.
2. Pressure differences produce bending moments `Mx` and `My`.
3. The moment direction produces the theta position command.
4. The moment wrench is mapped to the rigid joints with `tau = J.'*W`.
5. Joint-dependent gravity compensation is added.
6. A PD/PID bending-angle correction may be added as an extra bending moment before the Jacobian mapping.

The measured configuration is used to evaluate the Jacobian and gravity geometry. A Unit Delay or Zero-Order Hold is required on Simscape feedback signals to avoid an algebraic loop.

## Active Modeling Choices

- Current pneumatic pressure range: `0 <= p_i <= 350000 Pa` (3.5 bar).
- Muscle effective area: `A = pi*(0.013/2)^2 m^2`.
- Muscle radial offset: `r = 0.013 m`.
- The active torque wrench contains bending moments only: `W = [Mx; My; 0; 0; 0; 0]`.
- The axial muscle force `A*sum(p)` is currently omitted from joint-torque mapping because the backbone is assumed to support the common axial load. Including it previously dominated the calculated joint torque and caused nonphysical holding behavior.
- Simscape world gravity is `[0; 0; -9.80665] m/s^2`.
- The nominal Simscape module pose is horizontal, while the kinematic model's undeformed backbone is along its local `+z` axis. Gravity conversion must include the fixed Simscape-to-kinematic frame alignment described in `MODEL_ARCHITECTURE.md`.
- At nearly zero bending moment, theta is undefined. Hold the previous valid theta (with wrapping/unwrapping handled explicitly) rather than evaluating a noisy `atan2` direction.

## Relevant Current Files

- `s20260831_config2torque_sim_chatgpt.slx` — earlier configuration-to-torque reference model.
- `s20260910_config2torque_sim_chatgpt.slx` — later configuration-to-torque model.
- `s20260910_config2torque_sim_chatgpt_multi_torque.slx` — multi-torque/gravity development model.
- `s20260910_config2torque_sim_pressure_torque.slx` — pressure-driven torque model.
- `snake_single_module_library_torque_multi.slx` — multi-torque Simscape module library.
- `completeArm_Jacobian.m`, `Jth_Jacobian.m`, `linkTransformation.m` — rigid-link kinematics and Jacobian.
- `pressure2force_moment_config.m` — pressure-to-force/moment/configuration mapping.
- `estimateThetaPhi_CC.m` — constant-curvature configuration estimation from a measured rotation matrix.
- `check_tau_significance.m` — diagnostic decomposition of joint torque.
- `save_log_simulation.m`, `Meeting_plots.m` — simulation signal extraction and plotting.
- `Stiffness_cal.m`, `setting_simulink_parameters.m` — parameter calculations and workspace values.

## Known State and Limitations

- With gravity disabled, the module was observed to bend with approximately constant curvature.
- With gravity enabled, curvature became nonuniform unless joint-dependent gravity effects were addressed.
- Removing axial force from the active wrench restored sensible return motion and allowed torque gain 1 in the tested model.
- Replacing direct/Memory feedback with Unit Delay or Zero-Order Hold eliminated the algebraic-loop-related slowdown in the tested arrangement.
- Several stiffness values remain in the repository for different experiments (`K = 1e-5`, `K = 0.19223750`, joint stiffness `11.14977500`, and diagnostic value `60`). Their meanings and active usage must be reconciled before quantitative validation.
- Simulink `.slx` internals have not been fully validated from this documentation setup; current block connections and signs must be confirmed in MATLAB/Simulink.

## Immediate Next Steps

1. Confirm the exact inputs, outputs, dimensions, sample times, and signs of the MATLAB Function block in the latest pressure-driven model.
2. Reconcile continuum bending stiffness, per-joint stiffness, damping, disk masses, and inertias with the Simscape parameters.
3. Validate gravity-frame conversion for horizontal, upward, and downward base orientations.
4. Compare the 29-torque duplicated approximation with a full 58-phi-joint calculation.
5. Log pressure, desired/measured theta and phi, pressure torque, gravity torque, controller correction, and final applied torque.
6. Tune the phi PD/PID controller with saturation and anti-windup only after the open-loop pressure and gravity models agree with expected static behavior.

