%% check_tau_significance.m
% Separates moment-induced and axial-force-induced joint torques.
% Also compares the applied common torque with the torque required
% to deform the Simscape joint springs.

N = 29;
L = 0.29;
l = L/N;

A = pi*(0.013/2)^2;
r = 0.013;
k_joint = 60;                 % Simscape phi-joint stiffness [N*m/rad]

phi   = phi(:).';
theta = theta(:).';

if isscalar(theta)
    theta = theta*ones(size(phi));
end

nSamples = numel(phi);
assert(size(p_mat,2) == nSamples, ...
    'p_mat must contain one pressure column per phi sample.');

% MATLAB columns corresponding to the two phi joints of every link:
% [2,3,6,7,...]
phi_idx = find(ismember(mod(0:4*N-1,4), [1,2]));

tau_moment = zeros(2*N,nSamples);
tau_force  = zeros(2*N,nSamples);
tau_total  = zeros(2*N,nSamples);

for k = 1:nSamples
    p = p_mat(:,k);

    % Tip bending moments
    mx = A*r*sqrt(3)*(p(2) - p(3))/2;
    my = -A*r*(-p(2) - p(3) + 2*p(1))/2;

    % Corrected resultant axial force
    Fz = A*sum(p);

    % If the PMAs pull toward the base, test this instead:
    % Fz = -A*sum(p);

    W_moment = [mx; my; 0; 0; 0; 0];
    W_force  = [0; 0; 0; 0; 0; Fz];

    J = completeArm_Jacobian(theta(k),phi(k),l,N);

    % Compute all 116 joint torques
    tau_moment_all = J.'*W_moment;
    tau_force_all  = J.'*W_force;
    
    % Retain only the 58 phi-joint torques
    tau_moment(:,k) = tau_moment_all(phi_idx);
    tau_force(:,k)  = tau_force_all(phi_idx);
    tau_total(:,k)  = tau_moment(:,k) + tau_force(:,k);
end

% Common torque needed only to balance the 58 joint springs
tau_spring = k_joint*phi/(2*N);

% Select representative phi joints
iBase = 1;
iMiddle = round(size(tau_total,1)/2);
iTip = size(tau_total,1);

figure;
tiledlayout(3,1);

% Contribution of moment and axial force
nexttile;
plot(phi,tau_moment(iBase,:),'LineWidth',1.5);
hold on;
plot(phi,tau_force(iBase,:),'LineWidth',1.5);
plot(phi,tau_total(iBase,:),'k','LineWidth',2);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('\tau [N m]');
legend('Moment contribution','Axial-force contribution','Total');
title('Torque decomposition at first \phi joint');

% Differences among joints
nexttile;
plot(phi,tau_total(iBase,:),'LineWidth',1.5);
hold on;
plot(phi,tau_total(iMiddle,:),'LineWidth',1.5);
plot(phi,tau_total(iTip,:),'LineWidth',1.5);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('\tau [N m]');
legend('Base \phi joint','Middle \phi joint','Tip \phi joint');
title('Variation of torque along the arm');

% Compare computed torque with spring requirement
nexttile;
plot(phi,tau_total(iBase,:),'LineWidth',1.5);
hold on;
plot(phi,tau_spring,'--','LineWidth',2);
hold off;
grid on;
xlabel('\Phi [rad]');
ylabel('\tau [N m]');
legend('Computed \tau at first joint','Required spring torque');
title('Available torque versus elastic holding torque');

% Numerical summary
[peakTau,kPeak] = max(tau_total(iBase,:));

force_share = abs(tau_force(iBase,:)) ./ ...
    (abs(tau_moment(iBase,:)) + abs(tau_force(iBase,:)) + eps);

fprintf('First-joint torque peaks at Phi = %.3f rad\n',phi(kPeak));
fprintf('Peak first-joint torque        = %.4f N*m\n',peakTau);
fprintf('Axial-force share at peak      = %.1f %%\n',100*force_share(kPeak));
fprintf('Final required spring torque   = %.4f N*m\n',tau_spring(end));
fprintf('Final computed first torque    = %.4f N*m\n',tau_total(iBase,end));
fprintf('Final joint-torque range       = %.4f to %.4f N*m\n', ...
    min(tau_total(:,end)),max(tau_total(:,end)));


% Equivalent stiffness required at every phi joint

valid = phi > 1e-4;

phi_valid = reshape(phi(valid),1,[]);
q_phi_des = phi_valid/(2*N);

% Explicitly repeat q_phi_des for all 58 phi joints
q_matrix = repmat(q_phi_des,2*N,1);

k_equiv = tau_total(:,valid)./q_matrix;

figure;
plot(phi_valid,k_equiv(1,:),'LineWidth',1.5);
hold on;
plot(phi_valid,k_equiv(round(end/2),:),'LineWidth',1.5);
plot(phi_valid,k_equiv(end,:),'LineWidth',1.5);
yline(k_joint,'k--','Current stiffness');
hold off;

grid on;
xlabel('\Phi [rad]');
ylabel('Equivalent joint stiffness [N m/rad]');
legend('Base joint','Middle joint','Tip joint','Current stiffness');
title('Equivalent stiffness required for constant curvature');