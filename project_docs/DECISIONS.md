# Decisions

This file records design choices that materially affect the model. When a choice changes, retain the old entry and add a superseding entry with a date.

## D-001 — Use 29 Rigid-Link Intervals

**Status:** Accepted  
**Date:** 2026-08-31

The 30-disk module is represented by 29 distances between adjacent disks. Therefore `N = 29`, `l = L/N`, and the model has `29*4 = 116` revolute joints.

## D-002 — Preserve the Constant-Curvature Transformation Order

**Status:** Accepted  
**Date:** 2026-08-31

Each interval uses

`Rz(theta) * Ry(phi/(2N)) * Tz(l) * Ry(phi/(2N)) * Rz(-theta)`.

The paired z rotations select the bending plane and cancel net twist. The two y rotations split the interval's bending equally around the rigid translation.

## D-003 — Position-Control Theta and Torque-Control Phi

**Status:** Accepted for the current Simscape model  
**Date:** 2026-08-31

Theta joints receive position commands; phi joints receive torque commands. This matches the implemented joint interfaces. It does not reproduce fully passive bending-plane swing near zero curvature, so that behavior is outside the current milestone.

## D-004 — Omit Common Axial Muscle Force from the Active Joint Wrench

**Status:** Accepted for the current model; must be revisited if backbone axial compliance is modeled  
**Date:** 2026-09-02

Use `W = [Mx; My; 0; 0; 0; 0]` for active bending torque. The three muscles' common axial load is assumed to be carried by the continuum backbone rather than converted into large revolute-joint torques. Including `Fz = A*sum(p)` produced a torque dominated by the axial contribution and prevented realistic return motion.

## D-005 — Bound Pressure with Common-Mode/Null-Space Allocation

**Status:** Accepted for configuration-to-pressure operation  
**Date:** 2026-09-02

Add the same common pressure to all three muscles because this does not change `Mx` or `My`. Choose the common pressure so `0 <= p_i <= 350000 Pa`. If the requested differential pressure span is infeasible, scale the differential component while preserving bending direction.

## D-006 — Break Simscape Feedback with Discrete State

**Status:** Accepted  
**Date:** 2026-09-10

Use a Unit Delay or Zero-Order Hold between measured Simscape signals and torque calculations that feed the same physical network. Direct feedthrough creates an algebraic loop. The chosen sample time must be documented and small relative to the mechanical dynamics.

## D-007 — Include Fixed Simscape-to-Kinematic Frame Alignment

**Status:** Accepted pending sign validation  
**Date:** 2026-09-10

The nominal Simscape module is horizontal while the kinematic chain is defined along local `+z`. Convert gravity with

`R_WK = R_WB * R_MK`, `g_K = R_WK.' * g_W`,

where `R_MK = [1 0 0; 0 0 1; 0 -1 0]` and `g_W = [0;0;-9.80665]`.

## D-008 — Use 29 Phi-Torque Outputs as a Temporary Approximation

**Status:** Temporary  
**Date:** 2026-09-10

Calculate the first phi-joint torque in each interval (`tau(2 + 4*(i-1))`) and apply it to both phi joints in that interval. This reduces interface changes but is not exact for gravity because the two joint axes have different origins. A full 58-output implementation remains the reference for validation.

## D-009 — Make Pressure the Primary Command

**Status:** Current direction  
**Date:** 2026-09-10

Compute `Mx`, `My`, and theta from the three input pressures. Use the pressure-derived moment for feedforward phi torque, then add gravity compensation and an optional PD/PID moment correction. Near zero moment, hold the previous valid theta because bending direction is undefined.

## D-010 — Add Feedback as a Moment Correction

**Status:** Planned/current experiments  
**Date:** 2026-09-10

Express phi-controller output in N·m and add it along the commanded bending direction before Jacobian torque mapping. Include output limits and anti-windup when an integral term is used. Validate pressure feedforward and gravity compensation without feedback first so the controller does not hide model errors.

