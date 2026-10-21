# Project Log

This file is an append-only chronological record. Do not rewrite older entries to match the current design; record superseding decisions in a new dated entry and update the authoritative state in `PROJECT_STATE.md`.

## 2026-08-31 — Configuration-to-Torque Baseline

- Established the rigid-link approximation of a 30-disk continuum module using 29 rigid-link intervals.
- Preserved the constant-curvature transformation order with four revolute joints per interval: `+theta`, two half-phi joints, and `-theta`.
- Used theta position control and phi torque input in Simscape.
- Created `s20260831_config2torque_sim_chatgpt.slx` as a reference model.
- Clarified that the model has `29*4 = 116` revolute joints, not `30*4`.

## 2026-09-01 — Torque Significance and Axial-Force Diagnosis

- Added/used `check_tau_significance.m` to decompose moment-generated and axial-force-generated joint torques.
- Corrected axial muscle force from an expression containing the radial offset to `Fz = A*(p1+p2+p3)`.
- The axial-inclusive diagnostic showed that the common axial load dominated the base-joint torque at large bending angles.
- Identified that a single common phi torque cannot reproduce the joint-to-joint load distribution, especially under gravity.

## 2026-09-02 — Pressure Allocation and Feedback Experiments

- Replaced iterative pressure updates with direct pressure-to-moment/configuration relationships for testing.
- Developed common-mode/null-space pressure allocation to keep all three pressures within `0...350000 Pa` while preserving the requested bending direction when feasible.
- Tested measured theta/phi feedback. Direct Simscape feedback created an algebraic loop or severe solver slowdown; delayed feedback ran normally.
- Observed that the arm could remain bent when the axial-force term was included in the wrench.
- Removed the axial-force component from the active torque calculation. The model then returned properly, and the external torque scale was restored to 1.

## 2026-09-03 to 2026-09-06 — Tracking and Gravity Observations

- Logged desired/measured theta and phi, pressures, and phi torque for sinusoidal commands.
- Observed overshoot and pressure saturation, motivating a phi PD/PID loop.
- Observed approximately constant curvature with gravity disabled.
- Observed nonuniform curvature with gravity enabled, confirming the need for distributed gravity torque rather than a single common joint torque.
- Rotated the module orientation and tuned a phi controller for sinusoidal tracking in a gravity-enabled case.

## 2026-09-10 — Distributed Torque and Base-Orientation Work

- Added multi-torque Simulink/Simscape models and a multi-torque module library.
- Formulated gravity torque as the sum of downstream disk-weight moments at every joint.
- Added base-orientation input and identified a fixed alignment between the Simscape model frame and the kinematic frame.
- Chose a temporary 29-output approximation: calculate the first phi-joint torque in every interval and send that value to both phi joints in the interval.
- Broke the Simscape rotation-matrix algebraic loop using a Zero-Order Hold; Unit Delay was also successful in other measured-signal paths.
- Began moving from configuration input to pressure input so pressure-derived bending direction can drive theta.

## 2026-10-21 — Repository State and Documentation Initialized

- The remote `master` branch contained the configuration-to-torque, multi-torque, and pressure-driven Simulink models plus logging/plotting scripts.
- The latest inspected code used `N = 29` in the main torque test and included the corrected axial-force equation in `pressure2force_moment_config.m`.
- Initialized `project_docs/` as the durable handoff and engineering log for future Codex sessions.
- Recorded current assumptions, open issues, equations, validation observations, and the intended pressure-driven/gravity-compensated architecture.

