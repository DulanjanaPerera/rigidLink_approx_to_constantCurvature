function T = linkTransformation(theta, Phi, l, n, N)
% This function computes n link transformation. Typical transforamtion
% should be Rz.Ry.Tz.Ry.Rz(-). However, previous transforamtion was Rz.Ry.Tz.Rz(-).Ry
% 
% Inputs:
%   theta   : curve parameter [constant] (rad)
%   Phi     : curve parameter [constant] (rad)
%   l       : link length [constant] (m)
%   n       : n-th link [contant]
%   N       : total links [constant] 
% 
% Output:
%   Jc      : Jacobian of j-th joint [6xj]

T = eye(4);
phi = Phi/N;

% Ti = [(cos(theta) ^ 2 + cos(theta)) * cos(phi / 0.2e1) ^ 2 + (0.1e1 - cos(theta) ^ 2) * cos(phi / 0.2e1) - cos(theta) sin(theta) * cos(theta) * (cos(phi / 0.2e1) - 0.1e1) sin(phi / 0.2e1) * (cos(theta) + 0.1e1) * (cos(theta) * cos(phi / 0.2e1) - cos(theta) + 0.1e1) cos(theta) * sin(phi / 0.2e1) * l; sin(theta) * (cos(phi / 0.2e1) - 0.1e1) * (cos(theta) * cos(phi / 0.2e1) + cos(phi / 0.2e1) + 0.1e1) -cos(theta) ^ 2 * cos(phi / 0.2e1) + cos(theta) ^ 2 + cos(phi / 0.2e1) sin(phi / 0.2e1) * sin(theta) * (cos(theta) * cos(phi / 0.2e1) - cos(theta) + cos(phi / 0.2e1)) sin(theta) * sin(phi / 0.2e1) * l; -sin(phi / 0.2e1) * cos(phi / 0.2e1) * (cos(theta) + 0.1e1) -sin(theta) * sin(phi / 0.2e1) (cos(theta) + 0.1e1) * cos(phi / 0.2e1) ^ 2 - cos(theta) cos(phi / 0.2e1) * l; 0 0 0 1;];
Ti = [0.2e1 * cos(theta) ^ 2 * cos(phi / 0.2e1) ^ 2 - 0.2e1 * cos(theta) ^ 2 + 0.1e1 -0.2e1 * cos(theta) * sin(theta) * sin(phi / 0.2e1) ^ 2 0.2e1 * cos(theta) * cos(phi / 0.2e1) * sin(phi / 0.2e1) cos(theta) * sin(phi / 0.2e1) * l; -0.2e1 * cos(theta) * sin(theta) * sin(phi / 0.2e1) ^ 2 (-0.2e1 * cos(theta) ^ 2 + 0.2e1) * cos(phi / 0.2e1) ^ 2 + 0.2e1 * cos(theta) ^ 2 - 0.1e1 0.2e1 * sin(theta) * cos(phi / 0.2e1) * sin(phi / 0.2e1) sin(theta) * sin(phi / 0.2e1) * l; -0.2e1 * cos(theta) * cos(phi / 0.2e1) * sin(phi / 0.2e1) -0.2e1 * sin(theta) * cos(phi / 0.2e1) * sin(phi / 0.2e1) 0.2e1 * cos(phi / 0.2e1) ^ 2 - 0.1e1 cos(phi / 0.2e1) * l; 0 0 0 1;];


for i=1:n
    T = T * Ti;
end


end