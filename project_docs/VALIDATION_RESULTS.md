# Validation Results

Record repeatable tests, numerical results, and observed simulation behavior here. State the model, parameters, and whether the result is analytical, code-inspected, or simulation-observed.

## V-001 — Axial-Inclusive Torque Significance Sweep

**Source:** `check_tau_significance.m` and reported MATLAB output  
**Configuration:** `N = 29`, `L = 0.29 m`, sweep to `Phi = pi`  
**Status:** Diagnostic/legacy; axial force is no longer in the active wrench

- First-joint torque peak: `19.8279 N*m` at `Phi = 3.142 rad`.
- Axial-force share at the peak: `96.6%`.
- Final required spring torque in that test: `3.2499 N*m`.
- Final phi-joint torque range: `0.6740...19.8279 N*m`.
- Equivalent stiffness values reported at final phi:
  - Base joint: `366.061 N*m/rad`
  - Middle joint: `198.825 N*m/rad`
  - Tip joint: `12.444 N*m/rad`

**Interpretation:** The common axial load, not the bending moment, produced most of the calculated base torque. This supported removing the axial component from the revolute-joint actuation model while the backbone is treated as axially load-bearing.

## V-002 — Moment-Only Wrench Restored Return Motion

**Source:** Simulation observation  
**Status:** Passed qualitatively

With `Fz` removed from the wrench, the module bent and returned instead of remaining stuck at a bent configuration. The previously used `1e-2` torque gain was no longer needed; a gain of 1 was reported to simulate properly.

## V-003 — Delayed Feedback Removed Solver Slowdown

**Source:** Simulation observation  
**Status:** Passed qualitatively

Direct use of measured theta/phi or the Simscape base rotation matrix in a torque calculation created an algebraic loop and could make simulation extremely slow. A Unit Delay on measured signals and a Zero-Order Hold on the rotation-matrix path allowed the simulation to run. The exact sample times still need to be recorded.

## V-004 — Gravity Changes Curvature Distribution

**Source:** Simulation observation  
**Status:** Expected behavior confirmed qualitatively

- Gravity off: disks bent with approximately constant curvature.
- Gravity on: curvature became nonuniform and the distal disks bent less in one tested orientation.

**Interpretation:** A single common phi torque is insufficient when gravity is active; every joint supports a different amount of downstream mass.

## V-005 — Gravity-Frame Sanity Check

**Source:** Analytical frame check  
**World gravity:** `g_W = [0;0;-g]`, `g = 9.80665 m/s^2`  
**Fixed alignment:** `R_MK = Rx(-pi/2)`

For the specified base-orientation input `R_WB`, the expected kinematic-frame gravity vectors are:

| Module pose | `R_WB` | Expected `g_K` |
|---|---|---|
| Horizontal | `I` | `[0; g; 0]` |
| Downward | `[1 0 0; 0 0 1; 0 -1 0]` | `[0; 0; g]` |
| Upward | `[1 0 0; 0 0 -1; 0 1 0]` | `[0; 0; -g]` |

These values are a required unit test for the gravity conversion. Torque signs at the Simscape actuator still require end-to-end validation.

## V-006 — Repository Inspection at Documentation Initialization

**Date:** 2026-10-21  
**Status:** Text code inspected; binary model execution not performed

- Latest inspected `master` contained the newer configuration-to-torque, multi-torque, and pressure-driven `.slx` models.
- `m20260831_1_testing_config2torque.m` used `N = 29` and swept phi through `pi`.
- `pressure2force_moment_config.m` contained the corrected axial force `A*sum(p)`.
- `Stiffness_cal.m` computed continuum and per-joint stiffness from 3.5 bar, `Phi = pi`, and `N = 29`.
- Simulink signal connectivity, actuator signs, and solver settings were not executed or verified in this environment.

## Required Future Validation Dataset

For every orientation and pressure trajectory, log:

- `time`
- `p1`, `p2`, `p3`
- desired and measured `theta`
- desired/reference and measured `phi`
- pressure-generated torque vector
- gravity load and compensation torque vectors
- PD/PID correction torque vector
- final applied phi torque vector
- pressure saturation/feasibility flag

Test at minimum the horizontal, upward, and downward base poses with gravity both enabled and disabled.

