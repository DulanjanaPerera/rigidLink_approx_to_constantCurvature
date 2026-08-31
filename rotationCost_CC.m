function J = rotationCost_CC(x, R_sensor, xi)
% Cost function based on geodesic rotation error

    theta = x(1);
    phi   = x(2);

    R_cc = RotMat_continuum_Ronly(theta, phi, xi);

    % relative rotation error
    R_err = R_sensor' * R_cc;

    % geodesic angle error
    ang = rotationAngleFromMatrix(R_err);

    % squared angle cost
    J = ang^2;
end