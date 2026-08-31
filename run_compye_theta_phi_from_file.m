N = size(RRel, 3);

theta_all = zeros(N,1);
phi_all   = zeros(N,1);
err_all   = zeros(N,1);

for k = 1:N
    Rk = RRel(:,:,k);
    [theta_all(k), phi_all(k), info_k] = estimateThetaPhi_CC(Rk);
    err_all(k) = info_k.angle_error_deg;
end

tsec = 0:N-1;   % replace with your actual time vector if available

figure;
plot(tsec, rad2deg(theta_all), 'LineWidth', 1.2);
grid on;
xlabel('Sample');
ylabel('\theta [deg]');
title('Estimated bending direction');

figure;
plot(tsec, rad2deg(phi_all), 'LineWidth', 1.2);
grid on;
xlabel('Sample');
ylabel('\phi [deg]');
title('Estimated bending amount');

figure;
plot(tsec, err_all, 'LineWidth', 1.2);
grid on;
xlabel('Sample');
ylabel('Rotation fit error [deg]');
title('Optimization fit error');