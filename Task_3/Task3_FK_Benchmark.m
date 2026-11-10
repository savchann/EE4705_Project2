
%% Task 3 - Forward Kinematics Performance Benchmark
% Compare MATLAB getTransform() against PoE forward kinematics.
% Tests all six moving links and the tool0 end-effector.

clear; clc; close all;

%% Paths and results
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
addpath(thisDir, '-begin');

resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

%% Load UR5e
robot = loadrobot('universalUR5e', 'DataFormat', 'column');

% Initialize PoE model once
poe = ur5ePoEModel(robot);

%% Benchmark settings
rng(42, 'twister');

nTests = 1000;
nRepeats = 5;
nJoints = poe.nJoints;

% Generate identical configurations for both methods
Q = -pi + 2*pi*rand(nJoints, nTests);

% Warm up both methods
q = Q(:,1);

for k = 1:nJoints
    getTransform(robot, q, poe.movingNames{k});
    ur5eFKinSpace(poe.Mlinks{k}, ...
        poe.Slist(:,1:k), q(1:k));
end

getTransform(robot, q, 'tool0');
ur5eFKinSpace(poe.Mee, poe.Slist, q);

%% Timing benchmark
timeMATLAB = zeros(nRepeats,1);
timePoE = zeros(nRepeats,1);

for r = 1:nRepeats

    % Built-in MATLAB forward kinematics
    tStart = tic;

    for i = 1:nTests
        q = Q(:,i);

        for k = 1:nJoints
            T = getTransform(robot, q, poe.movingNames{k});
        end

        T = getTransform(robot, q, 'tool0');
    end

    timeMATLAB(r) = toc(tStart);

    % PoE forward kinematics
    tStart = tic;

    for i = 1:nTests
        q = Q(:,i);

        for k = 1:nJoints
            T = ur5eFKinSpace( ...
                poe.Mlinks{k}, ...
                poe.Slist(:,1:k), q(1:k));
        end

        T = ur5eFKinSpace( ...
            poe.Mee, poe.Slist, q);
    end

    timePoE(r) = toc(tStart);

end

%% Accuracy validation
positionErrors = zeros(nTests,1);
rotationErrors = zeros(nTests,1);

for i = 1:nTests

    q = Q(:,i);

    Tref = getTransform(robot, q, 'tool0');
    Tpoe = ur5eFKinSpace(poe.Mee, poe.Slist, q);

    positionErrors(i) = norm( ...
        Tref(1:3,4) - Tpoe(1:3,4));

    rotationErrors(i) = norm( ...
        Tref(1:3,1:3) - Tpoe(1:3,1:3), 'fro');

end

%% Summary
meanMATLAB = mean(timeMATLAB);
meanPoE = mean(timePoE);

speedup = meanMATLAB / meanPoE;

fprintf('\n==== UR5e FK Benchmark ====\n');
fprintf('Configurations: %d\n', nTests);
fprintf('Repeats: %d\n', nRepeats);
fprintf('MATLAB getTransform: %.4f s\n', meanMATLAB);
fprintf('PoE implementation : %.4f s\n', meanPoE);
fprintf('Speedup (MATLAB/PoE): %.2fx\n', speedup);

fprintf('\nMax position error: %.3e m\n', ...
    max(positionErrors));

fprintf('Max rotation error: %.3e\n', ...
    max(rotationErrors));

%% Save numerical results
summary = table( ...
    (1:nRepeats)', timeMATLAB, timePoE, ...
    'VariableNames', ...
    {'Repeat','MATLAB_Time_s','PoE_Time_s'});

stamp = datestr(now, 'yyyymmdd_HHMMSS');
base = fullfile(resultsDir, ['Task3_FK_Benchmark_' stamp]);

writetable(summary, [base '.csv']);

save([base '.mat'], ...
    'summary','Q','timeMATLAB','timePoE', ...
    'positionErrors','rotationErrors','speedup');

%% Plot results
fig = figure( ...
    'Color','w', ...
    'Name','UR5e FK Benchmark', ...
    'Position',[100 100 850 480]);

bar([meanMATLAB meanPoE]);

set(gca, ...
    'XTick',1:2, ...
    'XTickLabel',{'MATLAB getTransform','PoE'});

ylabel('Mean Runtime (s)');
title('UR5e Forward Kinematics Benchmark');
grid on;

ax = gca;
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';
ax.Title.Color = 'k';
ax.YLabel.Color = 'k';
ax.GridColor = [0.7 0.7 0.7];
ax.GridAlpha = 0.2;
ax.FontSize = 12;

exportgraphics(fig, ...
    [base '_graph.png'], ...
    'Resolution',300);

savefig(fig, [base '_graph.fig']);

fprintf('\nResults saved to: %s\n', resultsDir);
