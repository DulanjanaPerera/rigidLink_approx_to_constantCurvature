function c = f20260427_4_pressure2config(p, A, K, r)
% Compute the theta and phi from the pressures.
% Inputs:
%   p       : pressures (3x1) 
%   K       : stiffness (removed)
%   A       : area of the PMA tube
%   r       : radial distance from the backbone to the PMA center
% 
% Output:
%   c       : configuration vector [theta, phi] (1x2)


c = [atan2(-A * r * sqrt(0.3e1) * (p(2) - p(3)) / 0.2e1, -A * r * (-p(2) - p(3) + 0.2e1 * p(1)) / 0.2e1), sqrt(A ^ 2 * r ^ 2 * (p(1) ^ 2 - p(2) * p(1) - p(3) * p(1) + p(2) ^ 2 - p(2) * p(3) + p(3) ^ 2)) / K];

c(2) = wrapToPi(c(2));

end