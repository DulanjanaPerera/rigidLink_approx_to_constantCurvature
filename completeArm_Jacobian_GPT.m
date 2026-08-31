function J = completeArm_Jacobian_GPT(theta, Phi, l, nLinks)
nJoints = 4*nLinks;
J = zeros(6, nJoints);
for j = 1:nJoints
    J(:,j) = Jth_Jacobian_GPT(theta, Phi, l, j, nLinks);
end
end
