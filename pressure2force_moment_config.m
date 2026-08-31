function [f, my, mx, theta, phi] = pressure2force_moment_config(p, A, r, K)
% THis function computes moments around the backbone in x-axis and y-axis.
% Also it computes the configuration parameters of the CC arc.
% 
% Inputs:
%   p   : Pressures [3x1] (Pa)
%   A   : effective area of the PMA [constant] (m^2)
%   r   : radial offset of PMA from the backbone [constant] (m)
%   K   : effective bending stiffness [constant] (N/rad)
% 
% Outputs:
%   f       : generated force [constant] (N)
%   my      : Moments around y-axis [constant] (Nm)
%   mx      : Moments around x-axis [constant] (Nm)
%   theta   : bending direction [constant] (rad)
%   phi     : bending amount [constant] (rad)

f = p(1)*A*r + p(2)*A*r + p(3)*A*r;
my = -(A * r * (-p(2) - p(3) + 2 * p(1))) / 0.2e1;
mx = A * r * sqrt(0.3e1) * (p(2) - p(3)) / 0.2e1;
theta = atan2(-A * r * sqrt(0.3e1) * (p(2) - p(3)) / 0.2e1, -A * r * (-p(2) - p(3) + 0.2e1 * p(1)) / 0.2e1);
phi = sqrt(A ^ 2 * r ^ 2 * (p(1) ^ 2 - p(2) * p(1) - p(3) * p(1) + p(2) ^ 2 - p(2) * p(3) + p(3) ^ 2)) / K;

end