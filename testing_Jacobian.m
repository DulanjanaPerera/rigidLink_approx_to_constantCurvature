clear
A = pi * (0.013/2)^2;
r = 0.013;
K = 0.325/2;
l = 0.1;
N = 30;

stiff = 3.43;
d = 5;

p = [300000.0; 300000.0; 0.0];
[f, my, mx, theta, phi] = pressure2force_moment_config(p, A, r, K);
W = [mx; my; 0.0; 0.0; 0.0; f];
J = completeArm_Jacobian(theta, phi, l, N);

tau = J' * W;