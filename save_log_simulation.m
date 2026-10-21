tout = out.simout.Time;
phi_tau = reshape(out.simout.Data(1,1,:),[1,length(tout)]);
pressure = reshape(out.simout.Data(2:4,1,:),[3,length(tout)]);
theta_meas = reshape(out.simout.Data(5,1,:),[1,length(tout)]);
theta_des = reshape(out.simout.Data(6,1,:),[1,length(tout)]);
phi_meas = reshape(out.simout.Data(7,1,:),[1,length(tout)]);
phi_des = reshape(out.simout.Data(8,1,:),[1,length(tout)]);
phi_tau_unscale = reshape(out.simout.Data(9,1,:),[1,length(tout)]);

% save('sine_phi_log.mat', ...
% 'pressure', 'theta_meas', 'phi_meas', 'phi_tau', 'tout', 'theta_des', 'phi_des', 'phi_tau_unscale');