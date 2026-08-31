function Jc = Jth_Jacobian_GPT(theta, Phi, l, j, nLinks)
T0tip = linkTransformation_GPT(theta, Phi, l, nLinks, nLinks);
T0j   = T_base_to_joint(theta, Phi, l, j, nLinks);

Ttip_j = T0tip \ T0j;  % ^tip T_j = inv(^0T_tip)*^0T_j

% screw axis in joint frame (after the joint)
S = zeros(6,1);
k = mod(j-1,4) + 1;

if k==1 || k==3
    % Rz joints
    S(3) = 1;
    if k==3
        % this joint angle is -theta in your construction:
        % either keep q = -theta, S(3)=+1, OR q=theta, S(3)=-1.
        % Your code uses S(3)=-1, so keep that convention:
        S(3) = -1;
    end
else
    % Ry joints
    S(2) = 1;
end

Jc = adjointSE3(Ttip_j) * S;
end
