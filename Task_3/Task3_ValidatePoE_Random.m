
%% Task 3 - PoE Forward Kinematics Validation
% Compare PoE against MATLAB getTransform()
% across 100 random UR5e joint configurations.

clear; clc; close all;

%% Setup
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
addpath(thisDir);

resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

robot = loadrobot('universalUR5e', ...
    'DataFormat', 'column');

qHome = zeros(6,1);
M = getTransform(robot, qHome, 'tool0');

%% Calculate screw axes at home configuration
Slist = zeros(6,6);
jointIndex = 0;

for i = 1:robot.NumBodies

    body = robot.Bodies{i};
    joint = body.Joint;

    if strcmp(joint.Type, 'fixed')
        continue;
    end

    jointIndex = jointIndex + 1;

    Tbody = getTransform(robot, qHome, body.Name);
    Tjoint = Tbody / joint.ChildToJointTransform;

    omega = Tjoint(1:3,1:3) * joint.JointAxis(:);
    qAxis = Tjoint(1:3,4);

    Slist(:,jointIndex) = [omega; -cross(omega,qAxis)];
end

%% Random configuration validation
rng(42,'twister');

nTests = 100;
positionErrors = zeros(nTests,1);
rotationErrors = zeros(nTests,1);

for k = 1:nTests

    % Random joint angles within [-pi, pi]
    q = -pi + 2*pi*rand(6,1);

    % MATLAB built-in FK
    T_matlab = getTransform(robot, q, 'tool0');

    % PoE FK
    T_poe = ur5eFKinSpace(M, Slist, q);

    % Position error (metres)
    positionErrors(k) = norm( ...
        T_matlab(1:3,4) - T_poe(1:3,4));

    % Rotation matrix difference (Frobenius norm)
    rotationErrors(k) = norm( ...
        T_matlab(1:3,1:3) - T_poe(1:3,1:3), 'fro');
end

%% Display results
fprintf('\n==== PoE Validation: %d Configurations ====\n',nTests);
fprintf('Mean position error: %.3e m\n',mean(positionErrors));
fprintf('Max position error : %.3e m\n',max(positionErrors));
fprintf('Mean rotation error: %.3e\n',mean(rotationErrors));
fprintf('Max rotation error : %.3e\n',max(rotationErrors));

%% Save results
summary = table((1:nTests)',positionErrors,rotationErrors, ...
    'VariableNames',{'Test','PositionError_m','RotationError_Fro'});

stamp = datestr(now,'yyyymmdd_HHMMSS');
base = fullfile(resultsDir,['Task3_PoE_Validation_' stamp]);

writetable(summary,[base '.csv']);
save([base '.mat'],'summary','Slist','M','positionErrors','rotationErrors');

%% Plot validation errors
fig = figure('Color','w', ...
    'Name','UR5e PoE Validation', ...
    'Position',[100 100 1000 420]);

tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

nexttile;
semilogy(1:nTests, max(positionErrors,eps),'-o', ...
    'LineWidth',1,'MarkerSize',3);
grid on;
xlabel('Test Configuration');
ylabel('Position Error (m)');
title('PoE Position Error');

nexttile;
semilogy(1:nTests, max(rotationErrors,eps),'-o', ...
    'LineWidth',1,'MarkerSize',3);
grid on;
xlabel('Test Configuration');
ylabel('Rotation Matrix Error (Frobenius Norm)');
title('PoE Orientation Error');

% White background, black text
ax = findall(fig,'Type','axes');

for i = 1:numel(ax)
    ax(i).Color = 'w';
    ax(i).XColor = 'k';
    ax(i).YColor = 'k';
    ax(i).Title.Color = 'k';
    ax(i).XLabel.Color = 'k';
    ax(i).YLabel.Color = 'k';
    ax(i).GridAlpha = 0.2;
end

exportgraphics(fig,[base '_graphs.png'],'Resolution',300);
savefig(fig,[base '_graphs.fig']);

fprintf('Validation results saved to: %s\n',resultsDir);
