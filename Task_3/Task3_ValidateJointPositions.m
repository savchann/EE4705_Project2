
%% Task 3 - Validate PoE Joint Positions

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end

addpath(thisDir);

robot = loadrobot('universalUR5e', ...
    'DataFormat','column');

theta = [0.3; -0.5; 0.8; -0.4; 0.6; -0.2];

% PoE-based joint positions
[X, T, Tee] = updateJointsWorldPosition( ...
    robot,theta,'tool0');

fprintf('\n==== PoE Joint Position Validation ====\n');

movingNames = { ...
    'shoulder_link', ...
    'upper_arm_link', ...
    'forearm_link', ...
    'wrist_1_link', ...
    'wrist_2_link', ...
    'wrist_3_link'};

for k = 1:6

    Tref = getTransform(robot,theta,movingNames{k});

    posErr = norm( ...
        T{k}(1:3,4) - Tref(1:3,4));

    rotErr = norm( ...
        T{k}(1:3,1:3) - Tref(1:3,1:3),'fro');

    fprintf('%-18s | pos %.3e m | rot %.3e\n', ...
        movingNames{k},posErr,rotErr);

end

% End-effector validation
TrefEE = getTransform(robot,theta,'tool0');

fprintf('tool0              | pos %.3e m | rot %.3e\n', ...
    norm(Tee(1:3,4)-TrefEE(1:3,4)), ...
    norm(Tee(1:3,1:3)-TrefEE(1:3,1:3),'fro'));

fprintf('\nJoint position matrix size: %d x %d\n', ...
    size(X,1),size(X,2));
