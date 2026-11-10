%% *Task 5 extra experiments: choosing the weight, screening, failure cause*
% All on the Task 4 scenarios with cup-upright endpoints (helperScenarioCup.m).
%
% A. Weight selection. Sweep w_c on the two scenarios with the largest
%    unconstrained tilt (pillar, clutter) and apply a stated rule instead of
%    hand-tuning: pick the SMALLEST w_c > 0 for which, in BOTH scenarios,
%      (1) the median of the max cup tilt is <= tiltLimit (10 deg), and
%      (2) the number of collision-free seeds drops by at most one
%          compared with no constraint (w_c = 0).
% B. Pillar-distance screening with the constraint on (0.65 / 0.55 / 0.42 m),
%    plus the 0.42 m pillar WITHOUT the constraint: does the constraint
%    create new failures, or does the planner fail there anyway?
% C. Failure-cause test: 0.42 m pillar, constraint on, STOMP started from a
%    path through one via-point between the base and the pillar instead of
%    the straight line. If success rises, the failure is a local minimum of
%    the straight-line start, not an impossible task.
%
% Run time: (8 + 3 + 1 + 1) x nTrials planning runs (39 with nTrials = 3),
% about 30 min.
% Results: Task_5/Results/Task5_extras_*.png, Task5_extras.csv, .mat

clear; clc; close all;

%% Settings
nTrials      = 3;
weights      = [0 3 10 30];
weightScen   = {'pillar', 'clutter'};
tiltLimit    = 10;                 % deg
qcWeightMain = 10;                 % weight used in Task5_CupUpright (for B and C)
pillarXs     = [0.65 0.55 0.42];
saveFigures  = true;

%% Paths
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
addpath(task1Dir); addpath(fullfile(rootDir, 'Task_2')); addpath(fullfile(rootDir, 'Task_4')); addpath(thisDir);
assert(startsWith(which('stompTrajCost'), thisDir), 'Task_5 stompTrajCost.m must be first on the path.');
resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

robot = loadrobot('universalUR5e', 'DataFormat', 'column');
ee = 'tool0';
base.spheres = ur5eSphereModel(robot);
base.eeName = ee; base.upAxis = 2; base.worldUp = [0; 0; 1];
opts = struct('nDiscretize', 20, 'nPaths', 20, 'stopRule', 'improved', 'maxIterations', 150);
seeds = 1:nTrials;
rows = struct([]);

%% A. Weight selection
fprintf('\n=== A. Weight selection (%d seeds each) ===\n', nTrials);
nS = zeros(numel(weightScen), numel(weights)); medT = nS; meanIt = nS;
for s = 1:numel(weightScen)
    sc = helperScenarioCup(weightScen{s});
    [world, vw, clrIdx, q0, qT] = setupScenario(robot, ee, sc);
    for k = 1:numel(weights)
        m = base; m.qcWeight = weights(k);
        out = runTrials(robot, q0, qT, vw, world, m, opts, seeds, ee, clrIdx);
        nS(s, k) = sum(out.success); medT(s, k) = median(out.maxTilt); meanIt(s, k) = mean(out.iter);
        fprintf('%-8s w_c = %4g: success %d/%d, median max tilt %5.1f deg, mean %.0f iterations\n', ...
            weightScen{s}, weights(k), nS(s, k), nTrials, medT(s, k), meanIt(s, k));
        rows = addRow(rows, 'A weight selection', sprintf('%s, w_c = %g', weightScen{s}, weights(k)), out);
    end
end
okTilt = all(medT <= tiltLimit, 1);
okSucc = all(nS >= nS(:, weights == 0) - 1, 1);
cand = find(okTilt & okSucc & weights > 0, 1, 'first');
if isempty(cand)
    chosenW = NaN;
    fprintf('\nRule: no weight satisfies both conditions with %d seeds.\n', nTrials);
else
    chosenW = weights(cand);
    fprintf('\nRule picks w_c = %g (smallest weight meeting tilt <= %g deg and success >= w_c=0 minus one seed).\n', chosenW, tiltLimit);
end
fprintf('Weights meeting the rule: %s\n', mat2str(weights(okTilt & okSucc & weights > 0)));

figA = figure('Name', 'Task5 weight selection', 'Color', 'w', 'Position', [100 100 900 380]);
subplot(1, 2, 1); bar(categorical(string(weights), string(weights)), 100 * nS' / nTrials); grid on; ylim([0 105]);
xlabel('w_c'); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials));
legend(weightScen, 'Interpreter', 'none', 'Location', 'south'); title('Obstacle avoidance');
subplot(1, 2, 2); plot(1:numel(weights), medT', '-o', 'LineWidth', 2); hold on;
yline(tiltLimit, 'k--', sprintf('%g deg limit', tiltLimit));
set(gca, 'XTick', 1:numel(weights), 'XTickLabel', string(weights)); grid on;
xlabel('w_c'); ylabel('Median of max cup tilt (deg)'); legend(weightScen, 'Interpreter', 'none'); title('Cup tilt');
sgtitle(sprintf('Choosing the constraint weight (rule picks w_c = %g)', chosenW));

%% B. Pillar-distance screening with the cup (constraint on), and 0.42 m without
fprintf('\n=== B. Pillar distance with the cup, w_c = %g ===\n', qcWeightMain);
rateB = zeros(size(pillarXs)); tiltB = rateB;
for k = 1:numel(pillarXs)
    sc = helperScenarioCup('pillar', struct('pillarX', pillarXs(k)));
    [world, vw, clrIdx, q0, qT] = setupScenario(robot, ee, sc);
    m = base; m.qcWeight = qcWeightMain;
    out = runTrials(robot, q0, qT, vw, world, m, opts, seeds, ee, clrIdx);
    rateB(k) = mean(out.success); tiltB(k) = median(out.maxTilt);
    fprintf('pillar %.2f m, w_c = %g: success %d/%d, median max tilt %.1f deg, mean %.0f iterations\n', ...
        pillarXs(k), qcWeightMain, sum(out.success), nTrials, tiltB(k), mean(out.iter));
    rows = addRow(rows, 'B pillar screening (cup)', sprintf('pillar %.2f m, w_c = %g', pillarXs(k), qcWeightMain), out);
end
% the closest pillar again WITHOUT the constraint
sc = helperScenarioCup('pillar', struct('pillarX', pillarXs(end)));
[world42, vw42, clrIdx42, q042, qT42] = setupScenario(robot, ee, sc);
m0 = base; m0.qcWeight = 0;
out0 = runTrials(robot, q042, qT42, vw42, world42, m0, opts, seeds, ee, clrIdx42);
fprintf('pillar %.2f m, w_c = 0:  success %d/%d, median max tilt %.1f deg\n', ...
    pillarXs(end), sum(out0.success), nTrials, median(out0.maxTilt));
rows = addRow(rows, 'B pillar screening (cup)', sprintf('pillar %.2f m, w_c = 0', pillarXs(end)), out0);

figB = figure('Name', 'Task5 pillar screening', 'Color', 'w');
bar(pillarXs, 100 * rateB, 0.5); grid on; ylim([0 105]);
xlabel('Pillar distance from base (m)'); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials));
title(sprintf('Pillar with the cup upright (w_c = %g)', qcWeightMain));

%% C. 0.42 m pillar: via-point initial path, constraint on
fprintf('\n=== C. Pillar %.2f m: via-point initial path, w_c = %g ===\n', pillarXs(end), qcWeightMain);
qV = ur5eSolveIK(robot, ee, sc.taskVia, sc.seedVia);
thVia = viaPointInit(q042, qV, qT42, opts.nDiscretize);
cVia = stompCheckCollision(robot, thVia, world42);
fprintf('Via-point initial path: %d/20 waypoints colliding (non-trivial if > 0)\n', cVia.nWp);
optsVia = opts; optsVia.thetaInit = thVia;
m = base; m.qcWeight = qcWeightMain;
outVia = runTrials(robot, q042, qT42, vw42, world42, m, optsVia, seeds, ee, clrIdx42);
fprintf('pillar %.2f m, via-point init: success %d/%d, median max tilt %.1f deg, mean %.0f iterations\n', ...
    pillarXs(end), sum(outVia.success), nTrials, median(outVia.maxTilt), mean(outVia.iter));
rows = addRow(rows, 'C via-point init', sprintf('pillar %.2f m, via-point, w_c = %g', pillarXs(end), qcWeightMain), outVia);

% overlays: straight-line start (seed 1) vs via-point start (best seed)
m = base; m.qcWeight = qcWeightMain;
outLine = runTrials(robot, q042, qT42, vw42, world42, m, opts, 1, ee, clrIdx42);   % seed 1, for the figure
thLine = straightLine(q042, qT42, opts.nDiscretize);
figC1 = plotTrajectoryOverlay(robot, outLine.theta{1}, thLine, world42, ee, ...
    sprintf('Pillar %.2f m, straight-line start | %s', pillarXs(end), ternary(outLine.success(1), 'collision-free', 'IN COLLISION')), ...
    struct('view', sc.view, 'axisLimits', sc.axisLimits, 'showEEAxis', true));
[~, best] = max(outVia.success - 1e-3 * outVia.maxTilt);
figC2 = plotTrajectoryOverlay(robot, outVia.theta{best}, thVia, world42, ee, ...
    sprintf('Pillar %.2f m, via-point start (red dashed) | %s, max tilt %.0f deg', pillarXs(end), ...
    ternary(outVia.success(best), 'collision-free', 'IN COLLISION'), outVia.maxTilt(best)), ...
    struct('view', sc.view, 'axisLimits', sc.axisLimits, 'showEEAxis', true));

%% Save
T = struct2table(rows); disp(T);
writetable(T, fullfile(resultsDir, 'Task5_extras.csv'));
save(fullfile(resultsDir, 'Task5_extras.mat'), 'rows', 'weights', 'weightScen', 'nS', 'medT', 'meanIt', ...
    'chosenW', 'pillarXs', 'rateB', 'tiltB', 'out0', 'outVia', 'thVia');
if saveFigures
    saveFigureSafe(figA, fullfile(resultsDir, 'Task5_extras_weight_selection.png'));
    saveFigureSafe(figB, fullfile(resultsDir, 'Task5_extras_pillar_screening.png'));
    saveFigureSafe(figC1, fullfile(resultsDir, 'Task5_extras_pillar042_line.png'));
    saveFigureSafe(figC2, fullfile(resultsDir, 'Task5_extras_pillar042_via.png'));
end
fprintf('\nDone. Results in %s\n', resultsDir);

%% ---- local functions ----
function [world, vw, clrIdx, q0, qT] = setupScenario(robot, ee, sc)
[world, vw] = buildVoxelWorld(sc.shapes);
clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
fprintf('[%s] endpoint clearance: start %.3f m, goal %.3f m\n', sc.name, ...
    endpointClearance(robot, q0, world, clrIdx), endpointClearance(robot, qT, world, clrIdx));
end

function rows = addRow(rows, experiment, setting, out)
r.experiment    = string(experiment);
r.setting       = string(setting);
r.nSeeds        = numel(out.success);
r.nSuccess      = sum(out.success);
r.medianMaxTilt = median(out.maxTilt);
r.meanIter      = mean(out.iter);
r.meanTimeSec   = mean(out.time);
rows = [rows; r];
end

function th = straightLine(q0, qT, N)
th = zeros(numel(q0), N);
for k = 1:numel(q0), th(k, :) = linspace(q0(k), qT(k), N); end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
