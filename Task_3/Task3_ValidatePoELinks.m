
%% Task 3 - Validate PoE for all UR5e moving links

clear; clc;

robot = loadrobot('universalUR5e', ...
    'DataFormat','column');

% Initialize PoE model
poe = ur5ePoEModel(robot);

% Test joint configuration
theta = [0.3; -0.5; 0.8; -0.4; 0.6; -0.2];

fprintf('\n==== PoE Link Validation ====\n');

for k = 1:poe.nJoints

    bodyName = poe.movingNames{k};

    % MATLAB forward kinematics
    T_matlab = getTransform(robot,theta,bodyName);

    % PoE forward kinematics
    T_poe = ur5eFKinSpace( ...
        poe.Mlinks{k}, ...
        poe.Slist(:,1:k), ...
        theta(1:k));

    % Position error
    posErr = norm( ...
        T_matlab(1:3,4) - T_poe(1:3,4));

    % Rotation matrix error
    rotErr = norm( ...
        T_matlab(1:3,1:3) - T_poe(1:3,1:3), ...
        'fro');

    fprintf('%-18s | pos %.3e m | rot %.3e\n', ...
        bodyName,posErr,rotErr);

end

%% Validate end-effector

T_matlab = getTransform(robot,theta,'tool0');

T_poe = ur5eFKinSpace( ...
    poe.Mee,poe.Slist,theta);

posErr = norm( ...
    T_matlab(1:3,4) - T_poe(1:3,4));

rotErr = norm( ...
    T_matlab(1:3,1:3) - T_poe(1:3,1:3), ...
    'fro');

fprintf('%-18s | pos %.3e m | rot %.3e\n', ...
    'tool0',posErr,rotErr);
