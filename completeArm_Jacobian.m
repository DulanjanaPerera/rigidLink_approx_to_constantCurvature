function J = completeArm_Jacobian(theta, Phi, l, N)
%#codegen
% Inputs:
%   theta   : bending direction [constant] (rad)
%   phi     : bending amount [constant] (rad)
%   l       : linear offset between joints [constant] (m)
%   N       : number of discrete links in the module [constant]
% comput the jacobian  upto j-th joint. Joint index starts from 0 and ends
% at index 4*N - 1.
% 
% Eg:- N=2, last index of the joint is n=7.


n = 4*N - 1; % The last index of the joint of the N-link system
J = zeros(6, n+1); % total number of joints = n + 1;

for i=0:n
    J(:,i+1) = Jth_Jacobian(theta, Phi, l, i, N);
end

end