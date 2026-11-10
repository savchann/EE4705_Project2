%% Task 2: waypoint-resolution validation (10, 20, 30 waypoints)
% Run from Task_2. Does not modify the original Task 2 simulation or files.
clear; clc;
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
assert(isfolder(task1Dir), 'Task 1 folder not found beside Task_2');
addpath(task1Dir); addpath(thisDir);
resultsDir = fullfile(thisDir, 'Results', 'Waypoint_Validation');
if ~isfolder(resultsDir), mkdir(resultsDir); end

robot = loadrobot('universalUR5e', 'DataFormat', 'column');
endEffector = 'tool0';
R_down = diag([1 -1 -1]);
taskInit = trvec2tform([0.45 -0.10 0.20]) * rotm2tform(R_down);
taskFinal = trvec2tform([0.40 0.55 0.30]) * rotm2tform(R_down);
seedInit = [-0.512; -1.407; 2.210; -2.374; -1.571; -2.083];
seedFinal = [0.745; -1.081; 1.440; -1.930; -1.571; -0.826];
% Use a fresh IK solver in ur5eSolveIK or clear its persistent state first.
clear ur5eSolveIK
[q0, ~] = ur5eSolveIK(robot, endEffector, taskInit, seedInit);
[qT, ~] = ur5eSolveIK(robot, endEffector, taskFinal, seedFinal);
helperCreateObstaclesUR5e;
model.spheres = ur5eSphereModel(robot);

waypointCounts = [10 20 30];
seeds = [1 2 3 4 5 42 10 20 30 100];
opts = struct('nDiscretize',20, 'nPaths',20, ...
    'convergenceThreshold',0.1, 'maxIterations',50, 'verbose',false);
% Consistent *maximum* angular interpolation step across resolutions.
% This is a discrete check, not continuous collision certification.
maxJointStepRad = deg2rad(1);

n = numel(waypointCounts)*numel(seeds);
WaypointCount = zeros(n,1); Seed = zeros(n,1);
Success = false(n,1); Iterations = zeros(n,1); Runtime_s = zeros(n,1);
InitialCost = zeros(n,1); FinalCost = zeros(n,1);
CollidingWaypoints = zeros(n,1); CollidingDense = zeros(n,1);
DenseConfigurations = zeros(n,1); MinClearance_m = zeros(n,1);
StopReason = strings(n,1);
runResults = cell(n,1);
r = 0;
for w = waypointCounts
    opts.nDiscretize = w;
    for seed = seeds
        r = r + 1;
        rng(seed, 'twister');
        tStart = tic;
        res = stompPlan(robot, q0, qT, voxel_world, model, opts);
        Runtime_s(r) = toc(tStart);
        % Use a common angular sampling resolution for collision checking.
        maxDelta = max(abs(diff(res.theta,1,2)), [], 'all');
        nInterp = max(10, ceil(maxDelta/maxJointStepRad));
        c = stompCheckCollision(robot, res.theta, world, nInterp);
        WaypointCount(r) = w;
        Seed(r) = seed;
        Success(r) = c.free && c.nWp == 0;
        Iterations(r) = res.iter;
        InitialCost(r) = res.Qtheta_initial;
        FinalCost(r) = res.Qtheta;
        CollidingWaypoints(r) = c.nWp;
        CollidingDense(r) = c.nDense;
        DenseConfigurations(r) = numel(c.denseCollision);
        MinClearance_m(r) = c.minClearance;
        StopReason(r) = string(res.stopReason);
        runResults{r} = struct('waypoints',w,'seed',seed, ...
            'planner',res,'collision',c,'nInterp',nInterp);
        fprintf('%2d wp | seed %3d | free %d | iter %2d | cost %7.2f | clr %7.1f mm | %5.2f s\n', ...
            w,seed,Success(r),Iterations(r),FinalCost(r), ...
            1000*MinClearance_m(r),Runtime_s(r));
    end
end
summary = table(WaypointCount,Seed,Success,Iterations,Runtime_s, ...
    InitialCost,FinalCost,CollidingWaypoints,CollidingDense, ...
    DenseConfigurations,MinClearance_m,StopReason);

nW = numel(waypointCounts);
SuccessRate_pct = zeros(nW,1); MeanRuntime_s = zeros(nW,1);
MeanIterations = zeros(nW,1); MeanFinalCost = zeros(nW,1);
MinClearance_mm = zeros(nW,1); MeanClearance_mm = zeros(nW,1);
for j = 1:nW
    idx = WaypointCount == waypointCounts(j);
    SuccessRate_pct(j) = 100*mean(Success(idx));
    MeanRuntime_s(j) = mean(Runtime_s(idx));
    MeanIterations(j) = mean(Iterations(idx));
    MeanFinalCost(j) = mean(FinalCost(idx));
    MinClearance_mm(j) = 1000*min(MinClearance_m(idx));
    MeanClearance_mm(j) = 1000*mean(MinClearance_m(idx));
end
comparison = table(waypointCounts(:),SuccessRate_pct,MeanRuntime_s, ...
    MeanIterations,MeanFinalCost,MinClearance_mm,MeanClearance_mm, ...
    'VariableNames',{'Waypoints','SuccessRate_pct','MeanRuntime_s', ...
    'MeanIterations','MeanFinalCost','MinClearance_mm','MeanClearance_mm'});

stamp = datestr(now,'yyyymmdd_HHMMSS');
base = fullfile(resultsDir,['Task2_Waypoint_Validation_' stamp]);
writetable(summary,[base '_trials.csv']);
writetable(comparison,[base '_summary.csv']);
save([base '.mat'],'summary','comparison','runResults','seeds', ...
    'waypointCounts','maxJointStepRad','q0','qT');

fig = figure('Color','w','Name','Task 2 Waypoint Validation', ...
    'Position',[100 100 1100 800]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; bar(waypointCounts,SuccessRate_pct); ylim([0 105]); grid on;
xlabel('Waypoints'); ylabel('Collision-free trials (%)'); title('Success rate');
nexttile; bar(waypointCounts,MeanRuntime_s); grid on;
xlabel('Waypoints'); ylabel('Mean runtime (s)'); title('Optimization runtime');
nexttile; bar(waypointCounts,MinClearance_mm); grid on;
xlabel('Waypoints'); ylabel('Minimum clearance (mm)'); title('Worst-case mesh clearance');
nexttile; bar(waypointCounts,MeanIterations); grid on;
xlabel('Waypoints'); ylabel('Mean iterations'); title('Convergence iterations');
sg = sgtitle('UR5e STOMP | waypoint-resolution validation (10 seeds per setting)');
sg.Color = 'k';
sg.FontWeight = 'bold';
%% Figure formatting - White background, black text

% Set figure background to white
fig.Color = 'w';

% Set all subplot axes
ax = findall(fig, 'Type', 'axes');

for k = 1:numel(ax)
    ax(k).Color = 'w';          % White plot background
    ax(k).XColor = 'k';         % Black X-axis
    ax(k).YColor = 'k';         % Black Y-axis
    ax(k).Title.Color = 'k';    % Black subplot title
    ax(k).XLabel.Color = 'k';   % Black X-axis label
    ax(k).YLabel.Color = 'k';   % Black Y-axis label
    ax(k).LineWidth = 1;
    ax(k).GridColor = [0.7 0.7 0.7];
    ax(k).GridAlpha = 0.2;
    ax(k).FontSize = 11;
end

% Set all text, including main title, to black
allText = findall(fig, 'Type', 'text');
set(allText, 'Color', 'k');

% Explicitly set the main title to black
mainTitle = findall(fig, 'Type', 'Text', ...
    'String', 'UR5e STOMP validation');

exportgraphics(fig,[base '_graphs.png'],'Resolution',300);
savefig(fig,[base '_graphs.fig']);
fprintf('\n--- Waypoint comparison ---\n'); disp(comparison);
fprintf('Saved validation files under: %s\n',resultsDir);
fprintf(['Note: raw STOMP costs for different waypoint counts are not directly ' ...
    'comparable without accounting for discretization.\n']);
