function [theta_opt, phi_opt, info] = estimateThetaPhi_CC(R_sensor)
% Estimate constant-curvature parameters theta and phi from a measured
% rotation matrix at xi = 1.
%
% Inputs:
%   R_sensor : 3x3 measured rotation matrix
%
% Outputs:
%   theta_opt : bending direction [rad]
%   phi_opt   : bending amount [rad]
%   info      : struct with details

    xi = 1;

    % Ensure proper rotation matrix
    R_sensor = projectToSO3(R_sensor);

    % ---------- closed-form initial guess ----------
    % For xi = 1:
    % R13 = cos(theta)*sin(phi)
    % R23 = sin(theta)*sin(phi)
    % R33 = cos(phi)

    R13 = R_sensor(1,3);
    R23 = R_sensor(2,3);
    R33 = R_sensor(3,3);

    theta0 = atan2(R23, R13);
    phi0   = atan2(sqrt(R13^2 + R23^2), R33);

    x0 = [wrapToPiLocal(theta0); phi0];

    % ---------- bounds ----------
    % theta periodic, phi usually taken in [0, pi]
    lb = [-2*pi; 0];
    ub = [ 2*pi; pi];

    % ---------- optimization ----------
    opts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'off', ...
        'MaxFunctionEvaluations', 5000, ...
        'OptimalityTolerance', 1e-12, ...
        'StepTolerance', 1e-12');

    problem.objective = @(x) rotationCost_CC(x, R_sensor, xi);
    problem.x0 = x0;
    problem.lb = lb;
    problem.ub = ub;
    problem.nonlcon = [];
    problem.solver = 'fmincon';
    problem.options = opts;

    [x_opt, fval, exitflag, output] = fmincon(problem);

    theta_opt = wrapToPiLocal(x_opt(1));
    phi_opt   = x_opt(2);

    % ---------- reconstructed rotation ----------
    R_fit = RotMat_continuum_Ronly(theta_opt, phi_opt, xi);

    % ---------- error metrics ----------
    R_err = R_sensor' * R_fit;
    ang_err = rotationAngleFromMatrix(R_err);

    info.x0 = x0;
    info.theta0 = x0(1);
    info.phi0 = x0(2);
    info.fval = fval;
    info.exitflag = exitflag;
    info.output = output;
    info.R_fit = R_fit;
    info.R_err = R_err;
    info.angle_error_rad = ang_err;
    info.angle_error_deg = rad2deg(ang_err);
end