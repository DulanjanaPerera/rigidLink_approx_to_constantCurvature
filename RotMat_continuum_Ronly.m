function R = RotMat_continuum_Ronly(theta, phi, xi)
% Rotation matrix of a constant-curvature continuum arm at point xi
%
% Inputs:
%   theta : bending direction [rad]
%   phi   : bending amount [rad]
%   xi    : normalized arc position, 0 <= xi <= 1
%
% Output:
%   R     : 3x3 rotation matrix

    cth = cos(theta);
    sth = sin(theta);
    ckp = cos(xi*phi);
    skp = sin(xi*phi);

    R = [ ...
        cth^2 * ckp + sth^2,         cth*sth*(ckp - 1),     cth*skp;
        cth*sth*(ckp - 1),           sth^2 * ckp + cth^2,   sth*skp;
       -cth*skp,                    -sth*skp,               ckp ];
end