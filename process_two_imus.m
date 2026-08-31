%% process_two_imus.m
% Read two Yost IMU text files, align start time, and compute
% relative rotation of sensor 2 with respect to sensor 1.
%
% Assumption:
%   Raw quaternion columns in file are [QX QY QZ QW]
%
% Convention used here:
%   R1 = orientation of sensor 1 w.r.t. world
%   R2 = orientation of sensor 2 w.r.t. world
%   Relative rotation of sensor 2 w.r.t. sensor 1:
%       R_21 = R1' * R2
%
% If your sensor quaternion convention is the opposite direction
% (world w.r.t. sensor), then use:
%       R_21 = R1 * R2'
% instead.

clear; clc;

%% -------- user inputs --------
file1 = 'sensor1.txt';
file2 = 'sensor2.txt';

% choose reference timeline:
% 'sensor1' -> use sensor 1 timestamps in overlap window
% 'sensor2' -> use sensor 2 timestamps in overlap window
referenceTimeline = 'sensor1';

% maximum allowable time mismatch for nearest-neighbor matching [seconds]
maxTimeMismatch = 0.02;   % 20 ms, adjust if needed

%% -------- read files --------
data1 = readYostIMUFile(file1);
data2 = readYostIMUFile(file2);

fprintf('Loaded sensor 1: %d samples\n', numel(data1.time));
fprintf('Loaded sensor 2: %d samples\n', numel(data2.time));

%% -------- align start time as closely as possible --------
% pick the later of the two start times, then find the nearest sample in each file
t0 = max(data1.time(1), data2.time(1));

idx1_start = nearestTimeIndex(data1.time, t0);
idx2_start = nearestTimeIndex(data2.time, t0);

t1_start = data1.time(idx1_start);
t2_start = data2.time(idx2_start);

fprintf('\nRequested common start time  : %s\n', string(t0));
fprintf('Chosen sensor 1 start sample : %s (idx=%d)\n', string(t1_start), idx1_start);
fprintf('Chosen sensor 2 start sample : %s (idx=%d)\n', string(t2_start), idx2_start);
fprintf('Initial start mismatch       : %.6f s\n', seconds(t2_start - t1_start));

% trim everything before aligned start
data1 = trimDataFromIndex(data1, idx1_start);
data2 = trimDataFromIndex(data2, idx2_start);

%% -------- overlapping interval --------
tStart = max(data1.time(1), data2.time(1));
tEnd   = min(data1.time(end), data2.time(end));

if tEnd <= tStart
    error('No overlapping time interval between the two files.');
end

mask1 = (data1.time >= tStart) & (data1.time <= tEnd);
mask2 = (data2.time >= tStart) & (data2.time <= tEnd);

data1 = applyMask(data1, mask1);
data2 = applyMask(data2, mask2);

fprintf('\nOverlap start : %s\n', string(tStart));
fprintf('Overlap end   : %s\n', string(tEnd));
fprintf('Overlap dur   : %.3f s\n', seconds(tEnd - tStart));

%% -------- choose common timeline and match nearest samples --------
switch lower(referenceTimeline)
    case 'sensor1'
        tCommon = data1.time;
    case 'sensor2'
        tCommon = data2.time;
    otherwise
        error('referenceTimeline must be ''sensor1'' or ''sensor2''.');
end

n = numel(tCommon);

idx1 = zeros(n,1);
idx2 = zeros(n,1);
dtErr = zeros(n,1);

for k = 1:n
    idx1(k) = nearestTimeIndex(data1.time, tCommon(k));
    idx2(k) = nearestTimeIndex(data2.time, tCommon(k));

    t1k = data1.time(idx1(k));
    t2k = data2.time(idx2(k));
    dtErr(k) = abs(seconds(t2k - t1k));
end

good = dtErr <= maxTimeMismatch;

fprintf('\nMatched samples kept         : %d / %d\n', nnz(good), n);
fprintf('Mean time mismatch           : %.6f s\n', mean(dtErr(good)));
fprintf('Max time mismatch (kept)     : %.6f s\n', max(dtErr(good)));

tAligned = tCommon(good);
idx1 = idx1(good);
idx2 = idx2(good);
dtErr = dtErr(good);

q1_xyzw = data1.q(idx1,:);   % [x y z w]
q2_xyzw = data2.q(idx2,:);   % [x y z w]

%% -------- compute relative rotation --------
% Relative quaternion:
%   q_21 = conj(q1) * q2
% where q1, q2 are [w x y z] for quaternion multiplication here.
%
% Output storage:
%   qRel_xyzw = [x y z w]
%   RRel(:,:,k) = relative rotation matrix of sensor 2 w.r.t. sensor 1

N = size(q1_xyzw,1);
qRel_xyzw = zeros(N,4);
RRel = zeros(3,3,N);

for k = 1:N
    q1_wxyz = xyzw2wxyz(q1_xyzw(k,:));
    q2_wxyz = xyzw2wxyz(q2_xyzw(k,:));

    q1_wxyz = q1_wxyz / norm(q1_wxyz);
    q2_wxyz = q2_wxyz / norm(q2_wxyz);

    % relative quaternion: sensor2 w.r.t sensor1
    qRel_wxyz = quatMultiply(quatConj(q1_wxyz), q2_wxyz);
    qRel_wxyz = qRel_wxyz / norm(qRel_wxyz);

    qRel_xyzw(k,:) = wxyz2xyzw(qRel_wxyz);
    RRel(:,:,k) = quatWXYZ2rotm(qRel_wxyz);
end

%% -------- optional Euler angles of relative motion --------
% ZYX = [yaw pitch roll] from rotm2eul in MATLAB
eulRel_ZYX = zeros(N,3);
for k = 1:N
    eulRel_ZYX(k,:) = rotm2eul(RRel(:,:,k), 'ZYX');
end
% columns: [yaw pitch roll]

%% -------- package results --------
result.time = tAligned;
result.timeMismatch_s = dtErr;
result.q1_xyzw = q1_xyzw;
result.q2_xyzw = q2_xyzw;
result.qRel_xyzw = qRel_xyzw;
result.RRel = RRel;
result.eulRel_ZYX = eulRel_ZYX;   % [yaw pitch roll], radians

%% -------- display first result --------
fprintf('\nFirst relative rotation matrix R_21:\n');
disp(RRel(:,:,1));

fprintf('First relative quaternion [x y z w]:\n');
disp(qRel_xyzw(1,:));

fprintf('First relative Euler ZYX [yaw pitch roll] deg:\n');
disp(rad2deg(eulRel_ZYX(1,:)));

%% -------- save result --------
save('relative_rotation_result.mat', 'result');
fprintf('\nSaved result to relative_rotation_result.mat\n');

%% -------- example plots --------
tsec = seconds(result.time - result.time(1));

figure;
plot(tsec, rad2deg(result.eulRel_ZYX(:,1)), 'LineWidth', 1.2); hold on;
plot(tsec, rad2deg(result.eulRel_ZYX(:,2)), 'LineWidth', 1.2);
plot(tsec, rad2deg(result.eulRel_ZYX(:,3)), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Relative Euler angle [deg]');
legend('Yaw','Pitch','Roll');
title('Relative orientation of sensor 2 with respect to sensor 1');

figure;
plot(tsec, result.timeMismatch_s, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Match error [s]');
title('Timestamp mismatch after nearest-neighbor alignment');

%% ================= local functions =================

function data = readYostIMUFile(filename)
    % Reads Yost Labs text file with first line as comment/header.
    %
    % Expected columns:
    % datetime, QX, QY, QZ, QW, OrientPitch, OrientYaw, OrientRoll, VX, VY, VZ, A

    opts = detectImportOptions(filename, ...
        'Delimiter', ',', ...
        'CommentStyle', '#', ...
        'ReadVariableNames', false);

    T = readtable(filename, opts);

    % first column is datetime string
    timeStr = string(T{:,1});
    qx = T{:,2};
    qy = T{:,3};
    qz = T{:,4};
    qw = T{:,5};

    % parse datetime with milliseconds
    t = datetime(timeStr, ...
        'InputFormat', 'M/d/yyyy H:m:s.SSS', ...
        'Format', 'M/d/yyyy HH:mm:ss.SSS');

    q = [qx qy qz qw];

    % normalize all quaternions
    q = normalizeRows(q);

    data.time = t;
    data.q = q;   % [x y z w]
end

function idx = nearestTimeIndex(tvec, tQuery)
    [~, idx] = min(abs(seconds(tvec - tQuery)));
end

function dataOut = trimDataFromIndex(dataIn, idx0)
    dataOut.time = dataIn.time(idx0:end);
    dataOut.q    = dataIn.q(idx0:end,:);
end

function dataOut = applyMask(dataIn, mask)
    dataOut.time = dataIn.time(mask);
    dataOut.q    = dataIn.q(mask,:);
end

function qn = normalizeRows(q)
    nrm = sqrt(sum(q.^2, 2));
    qn = q ./ nrm;
end

function q_wxyz = xyzw2wxyz(q_xyzw)
    q_wxyz = [q_xyzw(4) q_xyzw(1) q_xyzw(2) q_xyzw(3)];
end

function q_xyzw = wxyz2xyzw(q_wxyz)
    q_xyzw = [q_wxyz(2) q_wxyz(3) q_wxyz(4) q_wxyz(1)];
end

function qc = quatConj(q)
    % q = [w x y z]
    qc = [q(1) -q(2) -q(3) -q(4)];
end

function q = quatMultiply(q1, q2)
    % Hamilton product, q = q1 * q2
    % q1, q2, q are [w x y z]

    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

    q = [ ...
        w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
        w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
        w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
        w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end

function R = quatWXYZ2rotm(q)
    % q = [w x y z], unit quaternion
    q = q / norm(q);

    w = q(1); x = q(2); y = q(3); z = q(4);

    R = [ ...
        1 - 2*(y^2 + z^2),   2*(x*y - z*w),       2*(x*z + y*w);
        2*(x*y + z*w),       1 - 2*(x^2 + z^2),   2*(y*z - x*w);
        2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x^2 + y^2) ];
end