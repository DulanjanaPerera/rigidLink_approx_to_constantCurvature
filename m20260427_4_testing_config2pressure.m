clear

A = pi*(0.013/2)^2;
r = 0.013;
K = 0.28;
L = 0.1778;

p=[0.0001;0.01;0.01];
phi = linspace(0.0001, pi/2, 500);
theta = linspace(0, 0.1, 500);
% theta(1) = 0;
bend = zeros(size(phi));
dir = zeros(size(phi));
p_mat = zeros(3,length(phi));
p_mat(:,1) = p;

frw = zeros(2, length(phi));

theta(1) = (atan2(-A * r * sqrt(0.3e1) * (p(2) - p(3)) / 0.2e1, -A * r * (-p(2) - p(3) + 0.2e1 * p(1)) / 0.2e1));
phi(1) = wrapToPi(sqrt(A ^ 2 * r ^ 2 * (p(1) ^ 2 - p(2) * p(1) - p(3) * p(1) + p(2) ^ 2 - p(2) * p(3) + p(3) ^ 2)) / K);


frw(:,1) = [theta(1); phi(1)];

for i=1:(length(phi))
    
    dir(i) = (atan2(-A * r * sqrt(0.3e1) * (p(2) - p(3)) / 0.2e1, -A * r * (-p(2) - p(3) + 0.2e1 * p(1)) / 0.2e1));
    bend(i) = wrapToPi(sqrt(A ^ 2 * r ^ 2 * (p(1) ^ 2 - p(2) * p(1) - p(3) * p(1) + p(2) ^ 2 - p(2) * p(3) + p(3) ^ 2)) / K);

    if (i ~= length(phi))
        % while(e < 0.001)
            J_angle = f20250421_1_Pressure_Jacobian(p, A, r, K);
            J_inv = pinv(J_angle);
            % p = p_mat(:, i) + J_inv*([theta(i+1); phi(i+1)]-[frw(1,i); frw(2,i)]);
            p = p_mat(:, i) + J_inv*([theta(i+1); phi(i+1)]-[dir(i); bend(i)]);
            p = m20250911_2_PositivePressures(p);
            p_mat(:, i+1) = p;  
        % end
        % forward model
        frw(:,i+1) = wrapToPi(frw(:,i) + J_angle*(p_mat(:, i+1) - p_mat(:, i)));
    end
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
legend theta forward 'theta desired'

figure(3)
plot((rad2deg(bend)), '*')
hold on
% plot(rad2deg(frw(2,:)), '+')
plot((rad2deg(phi)), '--')
hold off
grid on
axis tight
legend phi forward 'phi desired'

figure(4)
plot(wrapTo360(rad2deg(wrapTo2Pi(abs(dir - theta)))))
hold off
grid on
axis tight
legend 'theta difference'