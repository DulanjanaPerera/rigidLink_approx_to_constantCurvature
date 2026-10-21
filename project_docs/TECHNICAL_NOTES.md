# Technical Notes

## 1. Constant-Curvature and Rigid-Link Equivalence

The constant-curvature transform can be expressed as

`T(theta,phi) = Rz(theta) * Px(L/phi) * Ry(phi) * Px(-L/phi) * Rz(-theta)`.

The rigid-link approximation replaces the circular arc with `N` translations of length `l = L/N`, splitting each interval's bend symmetrically:

`T_i = Rz(theta) * Ry(phi/(2N)) * Tz(l) * Ry(phi/(2N)) * Rz(-theta)`.

For 30 disks, use `N = 29`. Each phi joint has the constant-curvature target coordinate

`q_phi = phi/(2N)`.

## 2. Pressure, Moment, and Configuration

Let `A` be muscle effective area and `r` the radial offset. For the current muscle numbering,

`Mx = (sqrt(3)/2)*A*r*(p2-p3)`

`My = (1/2)*A*r*(p2+p3-2*p1)`

`Fz = A*(p1+p2+p3)`

`M = sqrt(Mx^2+My^2)`

`theta = atan2(-Mx,My)`

`phi = M/K`.

The sign of theta depends on the established pressure and frame convention and must be checked against positive Simscape joint motion.

### Zero-Moment Handling

When `M < M_epsilon`, theta is physically undefined. Use the previous valid angle:

`theta_cmd(k) = theta_cmd(k-1)`.

For `M >= M_epsilon`, unwrap the new `atan2` result relative to the previous value before sending it to a position-controlled joint.

## 3. Pressure Null Space and Limits

A zero-sum differential pressure vector that generates desired moments is

`p_delta = [-2*My/(3*A*r); My/(3*A*r)+Mx/(sqrt(3)*A*r); My/(3*A*r)-Mx/(sqrt(3)*A*r)]`.

All vectors of the form

`p = p_delta + p_common*[1;1;1]`

produce the same bending moments. For limits `p_min` and `p_max`, the feasible common-pressure interval is

`p_common_min = p_min - min(p_delta)`

`p_common_max = p_max - max(p_delta)`.

If the interval is nonempty, choose its midpoint for equal pressure margin. If it is empty, scale `p_delta` by

`alpha = (p_max-p_min)/(max(p_delta)-min(p_delta))`

before recomputing the interval. This preserves the bending direction but reduces moment magnitude.

## 4. Wrench-to-Joint Torque

For a Jacobian and wrench expressed in compatible frames,

`tau = J.'*W`.

The active bending-only pressure wrench is

`W_bend = [Mx;My;0;0;0;0]`.

The axial-inclusive diagnostic wrench is

`W_full = [Mx;My;0;0;0;Fz]`,

but `Fz` is intentionally omitted from the current active joint actuation while the backbone is assumed to carry the common axial load.

For interval `i = 1...N`, the first and second phi-joint indices are

`idx_phi_1(i) = 2 + 4*(i-1)`

`idx_phi_2(i) = 3 + 4*(i-1)`.

## 5. Distributed Gravity Torque

Let phi joint `j` have origin `o_j` and unit rotation axis `a_j`, expressed in the kinematic base frame. Let downstream body `k` have center of mass `c_k`, mass `m_k`, and gravitational acceleration `g_K`. The gravity load at joint `j` is

`tau_g_load(j) = sum_{k downstream of j} a_j.'*cross(c_k-o_j, m_k*g_K)`.

The feedforward compensation is

`tau_g_comp(j) = -tau_g_load(j)`.

This must be calculated for every phi-joint origin for exact compensation. Reusing the first phi-joint result at the second joint in an interval is an approximation.

An equivalent Jacobian expression is

`tau_g_load = sum_k Jv_com_k.'*(m_k*g_K)`,

where `Jv_com_k` is the translational Jacobian of body `k`'s center of mass.

## 6. Gravity Frame Conversion

Simscape uses

`g_W = [0;0;-9.80665]`.

The rigid-link equations assume an undeformed backbone along local `+z`, while the nominal Simscape assembly is horizontal. With `R_WB` mapping nominal module/base vectors into world coordinates and

`R_MK = [1 0 0; 0 0 1; 0 -1 0]`,

use

`R_WK = R_WB*R_MK`

`g_K = R_WK.'*g_W`.

Always verify the three canonical orientations listed in `VALIDATION_RESULTS.md` before trusting torque signs.

## 7. Controller Moment

For total bending-angle error `e_phi = phi_ref-phi_meas`, a PD correction can be written

`M_corr = Kp*e_phi + Kd*d(e_phi)/dt`.

For a PID controller, add `Ki*integral(e_phi)`, with anti-windup when moment or pressure commands saturate. Apply the correction in the bending direction:

`Mx_corr = -M_corr*sin(theta_cmd)`

`My_corr =  M_corr*cos(theta_cmd)`.

Then map `[Mx_corr;My_corr;0;0;0;0]` through the same Jacobian. Log the pressure-feedforward and controller contributions separately.

## 8. Stiffness Mapping

If each phi joint is modeled as a torsional spring and the intended joint angle is `q_phi = phi/(2N)`, its local equivalent stiffness is

`k_j = |tau_j|/|q_phi|`.

If all `2N` phi joints have equal stiffness `k_joint` and carry an equal moment in an ideal no-gravity constant-curvature case, the continuum and joint stiffnesses satisfy

`k_joint = 2*N*K_continuum`.

This relation is not valid for selecting a single constant joint stiffness from gravity-loaded joint torques, because gravity torque varies along the arm and with configuration.

## 9. Numerical/Simulation Guidance

- Use fixed-size vectors in MATLAB Function blocks for code generation.
- Preallocate 29- or 58-element outputs explicitly.
- Delay or sample Simscape feedback to break direct algebraic loops.
- Record the discrete sample time alongside controller gains.
- Keep pressure torque, controller torque, gravity load, gravity compensation, and final torque as distinct signals.
- Use finite-difference/virtual-work checks to validate Jacobian columns and torque signs.
