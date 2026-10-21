A = pi*(0.013/2)^2;
r = 0.013;
delta_p = 3.5e5;       % Pa
Phi = pi;
N = 29;

M = A*r*delta_p;
K_continuum = M/Phi;
k_joint = 2*N*K_continuum;

fprintf('Bending moment       = %.8f N*m\n', M);
fprintf('Continuum stiffness  = %.8f N*m/rad\n', K_continuum);
fprintf('Joint stiffness      = %.8f N*m/rad\n', k_joint);