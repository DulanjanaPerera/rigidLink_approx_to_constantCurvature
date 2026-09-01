%% Check the significance of the tip-wrench torque components
% Run the configuration-to-pressure script first so that the workspace
% contains theta, phi, and p_mat. This script then recalculates the joint
% torques using the corrected axial force and separates the moment and
% axial-force contributions.
%
% Required workspace variables
%   theta : bending-plane angle at each sample [rad]
%   phi   : total bending angle at each sample [rad]
%   p_mat : three muscle pressures, size 3-by-number_of_samples

% Wrench convention used by completeArm_Jacobian:
%   W = [Mx; My; Mz; Fx; Fy; Fz], expressed in the tip frame.

% The model has 30 disks and 29 rigid-link intervals. Each interval has
% four revolute joints, including two phi joints.


%% Validate and format the input data

required_variables = {'theta', 'phi', 'p_mat'};
for variable_number = 1:numel(required_variables)
    variable_name = required_variables{variable_number};
    if ~exist(variable_name, 'var')
        error(['Missing workspace variable "%s". Run the configuration-', ...
               'to-pressure script before check_tau_significance.'], ...
              variable_name);
    end
end

theta_check = reshape(theta, 1, []);
phi_check   = reshape(phi,   1, []);

number_of_samples = numel(phi_check);

if isscalar(theta_check)
    theta_check = repmat(theta_check, 1, number_of_samples);
end

if numel(theta_check) ~= number_of_samples
    error('theta and phi must contain the same number of samples.');
end

if size(p_mat, 1) ~= 3 && size(p_mat, 2) == 3
    p_mat = p_mat.';
end

if size(p_mat, 1) ~= 3 || size(p_mat, 2) ~= number_of_samples
    error('p_mat must have size 3-by-numel(phi).');
end


%% Model parameters

number_of_links = 29;
arm_length = 0.29;                         % [m]
link_length = arm_length/number_of_links;  % [m]

muscle_area = pi*(0.013/2)^2;              % [m^2]
muscle_radius = 0.013;                     % [m]

current_joint_stiffness = 60;              % [N m/rad]

% Use +1 when positive Fz agrees with the force convention in the model.
% For a contractile muscle force acting toward the base, the appropriate
% sign may be -1. The magnitude comparison is unaffected by this choice.
axial_force_sign = 1;


%% Identify the two phi joints in every rigid-link interval

% MATLAB column numbers are:
%   2, 3, 6, 7, ..., 114, 115
phi_joint_indices = reshape( ...
    [2:4:(4*number_of_links); 3:4:(4*number_of_links)], 1, []);

number_of_phi_joints = numel(phi_joint_indices);  % 2*N = 58

tau_moment = zeros(number_of_phi_joints, number_of_samples);
tau_axial  = zeros(number_of_phi_joints, number_of_samples);
tau_total  = zeros(number_of_phi_joints, number_of_samples);


%% Decompose tau = J'W into moment and axial-force contributions

for sample_number = 1:number_of_samples
    pressure = p_mat(:, sample_number);

    % Moments produced by the three muscles.
    moment_y = -(muscle_area*muscle_radius* ...
        (-pressure(2) - pressure(3) + 2*pressure(1)))/2;
    moment_x = muscle_area*muscle_radius*sqrt(3)* ...
        (pressure(2) - pressure(3))/2;

    % Corrected axial force. The previous expression contained an
    % erroneous multiplication by muscle_radius.
    force_z = axial_force_sign*muscle_area*sum(pressure);

    wrench_moment = [moment_x; moment_y; 0; 0; 0; 0];
    wrench_axial  = [0; 0; 0; 0; 0; force_z];

    jacobian = completeArm_Jacobian( ...
        theta_check(sample_number), phi_check(sample_number), ...
        link_length, number_of_links);

    % J'*W returns all 4*N joint torques. Retain only the 2*N phi joints.
    tau_moment_all = jacobian.'*wrench_moment;
    tau_axial_all  = jacobian.'*wrench_axial;

    tau_moment(:, sample_number) = tau_moment_all(phi_joint_indices);
    tau_axial(:,  sample_number) = tau_axial_all(phi_joint_indices);
    tau_total(:,  sample_number) = ...
        tau_moment(:, sample_number) + tau_axial(:, sample_number);
end


%% Representative joints and elastic holding torque

base_joint = 1;
middle_joint = round(number_of_phi_joints/2);
tip_joint = number_of_phi_joints;

% Under constant curvature, every phi joint has the desired angle
% q_phi = Phi/(2*N).
desired_joint_angle = phi_check/(2*number_of_links);
required_spring_torque = current_joint_stiffness*desired_joint_angle;


%% Equivalent stiffness required at each phi joint

% Calculate each sample separately. This prevents accidental division by
% only the final desired angle, which produces a misleading rising curve.
k_equivalent = nan(size(tau_total));
k_moment_only = nan(size(tau_moment));

for sample_number = 1:number_of_samples
    local_joint_angle = desired_joint_angle(sample_number);

    if abs(local_joint_angle) > 1e-8
        % Stiffness magnitudes required to balance the applied torques.
        k_equivalent(:, sample_number) = ...
            abs(tau_total(:, sample_number))/abs(local_joint_angle);
        k_moment_only(:, sample_number) = ...
            abs(tau_moment(:, sample_number))/abs(local_joint_angle);
    end
end


%% Torque plots

figure('Name', 'Torque significance');
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(phi_check, tau_moment(base_joint, :), 'LineWidth', 1.5);
hold on;
plot(phi_check, tau_axial(base_joint, :), 'LineWidth', 1.5);
plot(phi_check, tau_total(base_joint, :), 'k', 'LineWidth', 1.8);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('\tau [N m]');
title('Torque decomposition at first \phi joint');
legend('Moment contribution', 'Axial-force contribution', 'Total', ...
       'Location', 'best');

nexttile;
plot(phi_check, tau_total(base_joint, :), 'LineWidth', 1.5);
hold on;
plot(phi_check, tau_total(middle_joint, :), 'LineWidth', 1.5);
plot(phi_check, tau_total(tip_joint, :), 'LineWidth', 1.5);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('\tau [N m]');
title('Variation of torque along the arm');
legend('Base \phi joint', 'Middle \phi joint', 'Tip \phi joint', ...
       'Location', 'best');

nexttile;
plot(phi_check, abs(tau_total(base_joint, :)), 'LineWidth', 1.5);
hold on;
plot(phi_check, abs(required_spring_torque), '--', 'LineWidth', 1.8);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('Torque magnitude [N m]');
title('Available torque versus elastic holding torque');
legend('Computed \tau at first joint', 'Required spring torque', ...
       'Location', 'best');


%% Equivalent-stiffness plot

figure('Name', 'Equivalent joint stiffness');
plot(phi_check, k_equivalent(base_joint, :), 'LineWidth', 1.5);
hold on;
plot(phi_check, k_equivalent(middle_joint, :), 'LineWidth', 1.5);
plot(phi_check, k_equivalent(tip_joint, :), 'LineWidth', 1.5);
yline(current_joint_stiffness, '--k', 'Current stiffness', ...
      'LabelHorizontalAlignment', 'right');
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('Equivalent joint stiffness [N m/rad]');
title('Equivalent stiffness required for constant curvature');
legend('Base joint', 'Middle joint', 'Tip joint', ...
       'Current stiffness', 'Location', 'best');


%% Numerical summary

[peak_first_torque, peak_sample] = max(abs(tau_total(base_joint, :)));
axial_share = 100*abs(tau_axial(base_joint, peak_sample))/ ...
    max(peak_first_torque, eps);

valid_sample = find(abs(desired_joint_angle) > 1e-8, 1, 'last');
if isempty(valid_sample)
    error('All phi samples are zero; equivalent stiffness is undefined.');
end

fprintf('\nFirst-joint torque peaks at Phi = %.3f rad\n', ...
    phi_check(peak_sample));
fprintf('Peak first-joint torque        = %.4f N*m\n', peak_first_torque);
fprintf('Axial-force share at peak      = %.1f %%\n', axial_share);
fprintf('Final required spring torque   = %.4f N*m\n', ...
    abs(required_spring_torque(valid_sample)));
fprintf('Final computed first torque    = %.4f N*m\n', ...
    abs(tau_total(base_joint, valid_sample)));
fprintf('Final joint-torque range       = %.4f to %.4f N*m\n', ...
    min(abs(tau_total(:, valid_sample))), ...
    max(abs(tau_total(:, valid_sample))));

fprintf('\nEquivalent stiffness at final nonzero Phi:\n');
fprintf('Base joint   : %.3f N*m/rad\n', ...
    k_equivalent(base_joint, valid_sample));
fprintf('Middle joint : %.3f N*m/rad\n', ...
    k_equivalent(middle_joint, valid_sample));
fprintf('Tip joint    : %.3f N*m/rad\n', ...
    k_equivalent(tip_joint, valid_sample));

fprintf('\nMoment-only equivalent stiffness at the same Phi:\n');
fprintf('Base joint   : %.3f N*m/rad\n', ...
    k_moment_only(base_joint, valid_sample));
fprintf('Middle joint : %.3f N*m/rad\n', ...
    k_moment_only(middle_joint, valid_sample));
fprintf('Tip joint    : %.3f N*m/rad\n', ...
    k_moment_only(tip_joint, valid_sample));

% Direct check of the stiffness calculation at the final sample.
k_manual = abs(tau_total(base_joint, valid_sample))* ...
    (2*number_of_links)/abs(phi_check(valid_sample));
fprintf('\nStiffness calculation check (base joint):\n');
fprintf('Direct formula : %.6f N*m/rad\n', k_manual);
fprintf('Stored value   : %.6f N*m/rad\n', ...
    k_equivalent(base_joint, valid_sample));

