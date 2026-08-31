function R = RotMat_continuum(theta, phi, xi)
% This function computes HTM at a point xi on a constant curvature
% continuum arm. The xi=0 is the base and xi=1 tip of the continuum arm.
%
% Inputs:
%   theta   : bending direction of the continuum arm [constant] (rad)
%   phi     : bending amount [constant] (rad)
%   xi      : selection factors [constant] {0,1}
% 
% Output:
%   R       : extracted rotation matrix from the HTM [3x3]


T = [cos(theta) ^ 2 * cos(xi * phi) + sin(theta) ^ 2 cos(theta) * cos(xi * phi) * sin(theta) - sin(theta) * cos(theta) cos(theta) * sin(xi * phi) -cos(theta) * cos(xi * phi) * L / phi + cos(theta) * L / phi; cos(theta) * cos(xi * phi) * sin(theta) - sin(theta) * cos(theta) sin(theta) ^ 2 * cos(xi * phi) + cos(theta) ^ 2 sin(theta) * sin(xi * phi) -sin(theta) * cos(xi * phi) * L / phi + sin(theta) * L / phi; -cos(theta) * sin(xi * phi) -sin(theta) * sin(xi * phi) cos(xi * phi) sin(xi * phi) * L / phi; 0 0 0 1;];
R = T(1:3, 1:3);
P = T(1:3,4);

end