clear
A = pi * (0.013/2)^2;
r = 0.013;
K = 0.002;
N = 1;
L = 0.27;
l = L/N;

p=[0.001;0.01;0.01];
phi = linspace(0.01, pi/2, 500);
theta = linspace(0, pi, 500);
% theta(1) = 0;
bend = zeros(size(phi));
dir = zeros(size(phi));
tau = zeros(4*N,size(phi,2));
p_mat = zeros(3,length(phi));
p_mat(:,1) = p;

frw = zeros(2, length(phi));

C = f20260427_4_pressure2config(p, A, K, r);

theta(1) = C(1);
phi(1) = C(2);


frw(:,1) = [theta(1); phi(1)];

for i=1:(length(phi))
    
    C = f20260427_4_pressure2config(p, A, K, r);

    dir(i) = C(1);
    bend(i) = C(2);

    if (i ~= length(phi))
        J_angle = f20260427_3_pressure2config_Jacobian(p, A, K, r);
        J_inv = pinv(J_angle);
        % p = p_mat(:, i) + J_inv*([theta(i+1); phi(i+1)]-[frw(1,i); frw(2,i)]);
        p = p_mat(:, i) + J_inv*([theta(i+1); phi(i+1)]-[dir(i); bend(i)]);
        % p = m20260427_5_PositivePressures(p);
        p_mat(:, i+1) = p; 
    
        % forward model
        frw(:,i+1) = wrapToPi(frw(:,i) + J_angle*(p_mat(:, i+1) - p_mat(:, i)));
    end

    [f, my, mx, ~, ~] = pressure2force_moment_config(p, A, r, K);
    W = [mx; my; 0.0; 0.0; 0.0; f];
    J = completeArm_Jacobian(theta(i), phi(i), l, N);

    tau(:,i) = J' * W;
end

figure(1);
plot((1:1:length(phi)),p_mat(1,:),'*');
hold on
plot((1:1:length(phi)),p_mat(2,:),'o', 'Color','k');
plot((1:1:length(phi)),p_mat(3,:),'--');
hold off
grid on
axis tight
legend p1 p2 p3

figure(2)
plot((rad2deg(dir)), '*')
hold on
% plot(rad2deg(frw(1,:)), '+')
plot((rad2deg(theta)),'--')
hold off
grid on
axis tight
legend theta 'theta desired'

figure(3)
plot((rad2deg(bend)), '*')
hold on
% plot(rad2deg(frw(2,:)), '+')
plot((rad2deg(phi)), '--')
hold off
grid on
axis tight
legend phi 'phi desired'

figure(4)
plot(wrapTo360(rad2deg(wrapTo2Pi(abs(dir - theta)))))
hold off
grid on
axis tight
legend 'theta difference'

figure(5)
plot(wrapTo360(rad2deg(wrapTo2Pi(abs(bend - phi)))))
hold off
grid on
axis tight
legend 'phi difference'

figure(6)
title("Torque")
plot(tau(2,:));
hold on;
plot(tau(3,:));
hold off
grid on
axis tight
legend phi1 phi2