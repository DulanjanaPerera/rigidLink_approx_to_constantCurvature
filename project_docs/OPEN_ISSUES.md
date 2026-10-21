# Open Issues

Keep only active unresolved items here. When an item is resolved, remove it from this file and record the result in `PROJECT_LOG.md`, `DECISIONS.md`, or `VALIDATION_RESULTS.md` as appropriate.

## High Priority

### O-001 — Reconcile Stiffness Parameters

The repository contains multiple values serving different or unclear roles, including `K = 1e-5`, `K = 0.19223750`, joint stiffness `11.14977500`, and diagnostic joint stiffness `60`. Identify for each value whether it is continuum bending stiffness, individual revolute-joint stiffness, a Simscape spring setting, or a legacy test value. Establish one parameter source of truth with units.

### O-002 — Verify Current Simulink Block Interface

Confirm the exact current MATLAB Function block inputs/outputs, vector dimensions, sample times, and downstream routing in `s20260910_config2torque_sim_pressure_torque.slx` and `s20260910_config2torque_sim_chatgpt_multi_torque.slx`. Binary `.slx` contents were not executed during documentation initialization.

### O-003 — Validate Gravity Frame and Torque Sign

The fixed `R_MK` alignment is analytically consistent with the reported nominal orientations, but end-to-end signs must be checked against Simscape joint conventions. In the horizontal case, verify that gravity compensation opposes the load rather than reinforcing it. Log load torque and compensation torque separately.

### O-004 — Match Mass and Inertia Parameters

Gravity compensation must use the same disk/link masses, centers of mass, and inertias as the Simscape solids. Replace any provisional equal-mass assumption with values extracted from the actual model.

### O-005 — Decide Between 29 and 58 Phi Torques

The current 29-output scheme duplicates the first phi-joint torque at the second phi joint of each interval. Quantify the error relative to a 58-output calculation, especially under gravity and large curvature. Retain 29 outputs only if the approximation meets an explicit tolerance.

## Control and Numerical Issues

### O-006 — Theta at Zero Bending Moment

`theta = atan2(-Mx, My)` is undefined when `sqrt(Mx^2+My^2)` is nearly zero. Implement a previous-valid-angle hold, angle unwrapping, and an appropriate moment threshold. Confirm that pressure noise does not cause theta jumps.

### O-007 — Phi Tracking Controller

Tune the PD/PID controller after feedforward and gravity validation. Document gains, sample time, derivative filtering, output limits, and anti-windup. Pressure and torque saturation must be visible to the controller.

### O-008 — Algebraic Loop and Discrete Sample Time

Unit Delay/Zero-Order Hold solves the direct-feedthrough loop, but the selected sample time has not been recorded or justified. Measure its effect on stability, phase lag, and solver speed.

### O-009 — Confirm Jacobian and Wrench Frames

Verify whether `completeArm_Jacobian` produces a body/tip-frame or spatial/base-frame Jacobian and ensure `W` is expressed in the matching frame. Add a finite-difference or virtual-work test for representative configurations.

### O-010 — Pressure Saturation and Hardware Mapping

The simulation can use signed abstract moment commands, but the physical arm requires `0...350000 Pa`. Validate the null-space/common-pressure allocator against actual regulators, minimum useful pressure, pressure dynamics, hysteresis, and actuator calibration before sim-to-real tests.

## Repository Maintenance

### O-011 — Consolidate Duplicate and Legacy Models

Several dated scripts and Simulink variants overlap. After the active pressure-driven model is validated, label or archive legacy files and add a concise root README pointing to the canonical entry points.

### O-012 — Automate Regression Tests

Create MATLAB tests for transformation order, Jacobian dimensions/indexing, pressure-to-moment inversion, pressure bounds, gravity-frame conversion, and 29-vs-58 torque comparison.

