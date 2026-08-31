function T = Jth_jointTransformation(theta, Phi, l, j, N)
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


if (n == 0) % if n == 0 means the base link (1st link)
    T = eye(4);
else % we are at some link other than first joint
    % transforamtion upto n-1 link
    T = linkTransformation(theta, Phi, l, n, N);
end

switch joint
    case 0 % 1st joint Rz
        Rz = [cos(theta), -sin(theta), 0, 0; 
              sin(theta),  cos(theta), 0, 0; 
                       0,           0, 1, 0; 
                       0,           0, 0, 1];
        T = T * Rz;
    case 1 % 2nd joint Ry
        Rz = [cos(theta), -sin(theta), 0, 0; 
              sin(theta),  cos(theta), 0, 0; 
                       0,           0, 1, 0; 
                       0,           0, 0, 1];
        Ry = [cos(phi/2), 0, sin(phi/2), 0; 
                       0, 1,          0, 0;
             -sin(phi/2), 0, cos(phi/2), 0;
                       0, 0,          0, 1];
        T = T * Rz * Ry;
    case 2 % 3rd joint Ry again
        Rz = [cos(theta), -sin(theta), 0, 0; 
              sin(theta),  cos(theta), 0, 0; 
                       0,           0, 1, 0; 
                       0,           0, 0, 1];
        Ry = [cos(phi/2), 0, sin(phi/2), 0; 
                       0, 1,          0, 0;
             -sin(phi/2), 0, cos(phi/2), 0;
                       0, 0,          0, 1];
        Tz = eye(4);
        Tz(3,4) = l;
        T = T * Rz * Ry * Tz * Ry;    
    case 3
        Rz = [cos(theta), -sin(theta), 0, 0; 
              sin(theta),  cos(theta), 0, 0; 
                       0,           0, 1, 0; 
                       0,           0, 0, 1];
        Rz_neg = [cos(-theta), -sin(-theta), 0, 0; 
                  sin(-theta),  cos(-theta), 0, 0; 
                            0,            0, 1, 0; 
                            0,            0, 0, 1];

        Ry = [cos(phi/2), 0, sin(phi/2), 0; 
                       0, 1,          0, 0;
             -sin(phi/2), 0, cos(phi/2), 0;
                       0, 0,          0, 1];
        Tz = eye(4);
        Tz(3,4) = l;
        T = T * Rz * Ry * Tz * Ry *Rz_neg;
        
end



end