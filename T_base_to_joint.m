function T0j = T_base_to_joint(theta, Phi, l, j, nLinks)
% j: 1..(4*nLinks)
linkIdx = ceil(j/4);     % 1..nLinks
k = mod(j-1,4) + 1;      % 1..4 within the link
a = Phi/(2*nLinks);

% transform up to end of previous link
T0j = eye(4);
if linkIdx > 1
    T0j = linkTransformation_GPT(theta, Phi, l, linkIdx-1);
end

% now multiply within the current link up to joint k (inclusive)
switch k
    case 1  % after Rz(theta)
        T0j = T0j * Rz(theta);
    case 2  % after Rz(theta)*Ry(a)
        T0j = T0j * Rz(theta) * Ry(a);
    case 3  % after Rz*Ry*Tz*Rz(-theta)
        T0j = T0j * Rz(theta) * Ry(a) * Tz(l) * Rz(-theta);
    case 4  % after full link (includes last Ry(a))
        T0j = T0j * Rz(theta) * Ry(a) * Tz(l) * Rz(-theta) * Ry(a);
end
end

