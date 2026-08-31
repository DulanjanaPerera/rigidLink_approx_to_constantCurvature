function Jc = Jth_Jacobian(theta, Phi, l, j, N)
% This function computes, the j-th Jacobian of the joint. The
% transformation is Rz.Ry.Tz.Ry.Rz(-). Therefore,
%   0-th joint = Rz
%   1-st joint = Ry
%   2-nd joint = Ry
%   3-rd joint = Rz(-)
% 
%
% Inputs:
%   theta   : curve parameter [constant] (rad)
%   Phi     : curve parameter [constant] (rad)
%   l       : link length [constant] (m)
%   j       : j-th joint [contant]
%   N       : total links [constant] 
% 
% Output:
%   Jc      : Jacobian of j-th joint [6xj]

joint = mod(j,4);
Jc = zeros(6,j);
Ttip = linkTransformation(theta, Phi, l, N, N);

switch joint
    case 0 % Rz rotation
        T = Jth_jointTransformation(theta, Phi, l, j, N);
        T_join2tip = (Ttip) \ T;
        S = zeros(6,1);
        S(3) = 1;
        Jc = adjointSE3(T_join2tip) * S;
    case 1 % Ry rotation at the tip
        T = Jth_jointTransformation(theta, Phi, l, j, N);
        T_join2tip = (Ttip) \ T;
        S = zeros(6,1);
        S(2) = 1;
        Jc = adjointSE3(T_join2tip) * S;
    case 2 % Ry rotation
        T = Jth_jointTransformation(theta, Phi, l, j, N);
        T_join2tip = (Ttip) \ T;
        S = zeros(6,1);
        S(2) = 1;
        Jc = adjointSE3(T_join2tip) * S;
    case 3 % -Rz rotation
        T = Jth_jointTransformation(theta, Phi, l, j, N);
        T_join2tip = (Ttip) \ T;
        S = zeros(6,1);
        S(3) = -1;
        Jc = adjointSE3(T_join2tip) * S;

end

end