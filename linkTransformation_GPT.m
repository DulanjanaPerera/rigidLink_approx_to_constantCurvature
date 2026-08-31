function T = linkTransformation_GPT(theta, Phi, l, n, N)
% ^0T_{tip_of_link_nLinks} for a chain of nLinks identical links
% 
% Inputs:
%   theta   : curve parameter [constant] (rad)
%   Phi     : curve parameter [constant] (rad)
%   l       : link length [constant] (m)
%   n       : n-th link [contant]
%   N       : total links [constant] 
% 
% Output:
%   T      : HTM at n-th link tip [4x4]

T = eye(4);
a = Phi/(2*N);  % each Ry joint angle
Ti = Rz(theta) * Ry(a) * Tz(l) * Rz(-theta) * Ry(a);
for i = 1:n
    T = T * Ti;
end
end
