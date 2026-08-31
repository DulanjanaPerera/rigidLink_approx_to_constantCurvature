function J = f20260427_3_pressure2config_Jacobian(p, A, K, r)
% J: 2x3 finite double, robust to singular/near-singular inputs.
% if NaN is present then J = zeros

% % ---- constants ----
% A = pi*(0.013/2)^2;
% K = 0.22991; %0.24991
% r = 0.0130;

% ---- input hygiene ----
p = double(p(:));
if numel(p) ~= 3 || any(~isfinite(p))
    J = zeros(2,3);
    return;
end
p1 = p(1); p2 = p(2); p3 = p(3);

% ---- numerically safe helpers ----
EPS_DEN  = 1e-6;     % min magnitude for denominators (tune if needed)
EPS_SQRT = 1e-6;     % min value under sqrt (tune if needed)

den1 = 2*p2^2 + (-2*p1 - 2*p3)*p2 + 2*p1^2 - 2*p3*p1 + 2*p3^2;
den2 = 2*p1^2 + (-2*p2 - 2*p3)*p1 + 2*p2^2 - 2*p2*p3 + 2*p3^2;

% push denominators away from zero but preserve sign
den1 = sign(den1) * max(abs(den1), EPS_DEN);
den2 = sign(den2) * max(abs(den2), EPS_DEN);

quad = (p1^2 + (-p2 - p3)*p1 + p2^2 - p2*p3 + p3^2);
phiarg = (A^2 * r^2) * quad;

% clamp sqrt argument to be nonnegative and not too small
phiarg = max(phiarg, EPS_SQRT);

% ---- rows of Jacobian ----
row1 = [ ...
   -sqrt(3) * (p2 - p3) / den1, ...
    sqrt(3) * (-p3 + p1) / den2, ...
   -sqrt(3) * (-p2 + p1) / den2 ];

inv_sqrt = 1 / sqrt(phiarg);
c = (A^2 * r^2) * inv_sqrt / (2*K);

row2 = [ ...
    c * (-p2 - p3 + 2*p1), ...
   -c * ( p1 - 2*p2 + p3), ...
   -c * ( p1 + p2 - 2*p3) ];

J = [row1; row2];

% ---- final safety net ----
if any(~isfinite(J), 'all')
    J = zeros(2,3);  % fail-safe value rather than NaN/Inf
end

end
