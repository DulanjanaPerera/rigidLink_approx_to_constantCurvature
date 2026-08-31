function T = Jth_jointTransformation_old(theta, Phi, l, j, N)
% This function computes, j-th joint transformation. This includes both
% y-axis and z-axis rotations
% 
% Eg:-
%   j    =[0,1,2,3,4,5,6,7,8]
%   n    =[0,0,0,0,1,1,1,1,2]
%   joint=[0,1,2,3,0,1,2,3,0]
% 
% Therefore, if joint = 0 means, base (first Rz rotation)
% 
n = floor(j/4); % to detect the which link. if 0, that means first link
joint = mod(j,4); % to detect which joint in the link. if 0, that means base of the link (first Rz rotation)
T = eye(4);

phi = Phi / N;


if (joint == 0 && n~= 0) % if joint =0, and n~=0, that means we are at the base of the n-th joint (Rz transformation 
    T = linkTransformation(theta, Phi, l, n, N);

else % we are at some joint

    if n == 0 % if joint is at the first link
        T = eye(4);
    else % if joint is after the first link
        T = linkTransformation(theta, Phi, l, n, N);
    end

    switch joint
        case 1
            Rz = [cos(theta), -sin(theta), 0, 0; 
                        sin(theta), cos(theta), 0, 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            T = T * Rz;
        case 2
            Rz = [cos(theta), -sin(theta), 0, 0; 
                        sin(theta), cos(theta), 0, 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            Ry = [cos(phi/2), 0, sin(phi/2), 0; 
                        -sin(phi/2), 1, cos(phi/2), 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            T = T * Rz * Ry;
        case 3
            Rz = [cos(theta), -sin(theta), 0, 0; 
                        sin(theta), cos(theta), 0, 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            Rz_neg = [cos(-theta), -sin(-theta), 0, 0; 
                        sin(-theta), cos(-theta), 0, 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            Ry = [cos(phi/2), 0, sin(phi/2), 0; 
                        -sin(phi/2), 1, cos(phi/2), 0; 
                        0, 0, 1, 0; 
                        0, 0, 0, 1];
            Tz = eye(4);
            Tz(3,4) = l;
            T = T * Rz * Ry * Tz * Rz_neg;            

    end
end

end