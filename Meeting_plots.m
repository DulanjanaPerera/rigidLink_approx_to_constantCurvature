%% extract sim data
tout = out.simout.Time;
phi_tau = reshape(out.simout.Data(1,1,:),[1,length(tout)]);
pressure = reshape(out.simout.Data(2:4,1,:),[3,length(tout)]);
theta_meas = reshape(out.simout.Data(5,1,:),[1,length(tout)]);
theta_des = reshape(out.simout.Data(6,1,:),[1,length(tout)]);
phi_meas = reshape(out.simout.Data(7,1,:),[1,length(tout)]);
phi_des = reshape(out.simout.Data(8,1,:),[1,length(tout)]);
phi_tau_unscale = reshape(out.simout.Data(9,1,:),[1,length(tout)]);


%% plot sim data

% plotting theta and phi
figure(1)
H = orderedcolors("gem");
plot(tout, phi_meas,'Color', H(1,:));
hold on
plot(tout, phi_des, 'Color', H(1,:), 'LineStyle','--');
plot(tout, theta_meas,'Color', H(3,:));
plot(tout, theta_des, 'Color', H(3,:), 'LineStyle','--');
hold off
legend('Measured \phi', 'Desired \phi', 'Measured \theta', 'Desired \theta');
xlabel('Time (s)');
ylabel('Bending (\phi) (rad)');
title('\theta and \phi Trajectory');
grid on;
axis tight;

% plotting pressures
figure(2)
H = orderedcolors("gem");
plot(tout, pressure(1,:)./1e5,'Color', H(1,:));
hold on
plot(tout, pressure(2,:)./1e5, 'Color', H(5,:));
plot(tout, pressure(3,:)./1e5,'Color', H(3,:));
hold off
legend('P1', 'P2', 'P3');
xlabel('Time (s)');
ylabel('Pressure (Bar)');
title('Pressure Trajectory');
grid on;
axis tight;