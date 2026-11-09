%% *Task 5 extra experiments: choosing the weight, screening, failure cause*
% A. Weight selection. Sweep w_c on two scenarios (window, board_low) and
%    apply a stated rule instead of hand-tuning:
%      pick the SMALLEST w_c > 0 for which, in BOTH scenarios,
%        (1) the median of the max cup tilt is <= tiltLimit (10 deg), and
%        (2) the number of collision-free seeds drops by at most one
%            compared with no constraint (w_c = 0).
%    10 deg is a practical spill limit for a cup that is not full to the
%    brim. A small w_c is preferred because it disturbs obstacle avoidance least.
% B. Board-height screening (how the Task 5 failure case was chosen): with
%    the constraint on, raise the board from 0.22 to 0.30 m and see where
%    the constrained planner stops finding the lift-over route.
% C. Failure cause test for board_high: start STOMP from a path through a
%    via-point ABOVE the board. If success rises, the failure is a local
%    minimum of the straight-line start, not an impossible task.
%
% Run time: (8 + 3 + 1) x nTrials planning runs (36 with nTrials = 3),
% ~45 s each with the improved stopping rule: about 25-30 min.
% Results: Task_5/Results/Task5_extras_*.png, Task5_extras.csv, .mat

clear; clc; close all;

%% Settings
nTrials      = 3;
weights      = [0 3 10 30];
weightScen   = {'window', 'board_low'};
tiltLimit    = 10;                 % deg
qcWeightMain = 10;                 % weight used in Task5_CupUpright (for B and C)
boardZs      = [0.22 0.26 0.30];
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
nS = zeros(numel(weightScen), numel(weights)); medT = nS;
for s = 1:numel(weightScen)
    sc = helperScenarioCup(weightScen{s});
    [world, vw] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
    for k = 1:numel(weights)
        m = base; m.qcWeight = weights(k);
        out = runTrials(robot, q0, qT, vw, world, m, opts, seeds, ee, clrIdx);
        nS(s, k) = sum(out.success); medT(s, k) = median(out.maxTilt);
        fprintf('%-10s w_c = %4g: success %d/%d, median max tilt %5.1f deg, mean %.0f iterations\n', ...
            weightScen{s}, weights(k), nS(s, k), nTrials, medT(s, k), mean(out.iter));
        rows = addRow(rows, 'A weight selection', sprintf('%s, w_c = %g', weightScen{s}, weights(k)), out);
    end
end
okTilt = all(medT <= tiltLimit, 1);
okSucc = all(nS >= nS(:, weights == 0) - 1, 1);
cand = find(okTilt & okSucc & weights > 0, 1, 'first');
if isempty(cand)
    fprintf('\nRule: no weight satisfies both conditions with %d seeds.\n', nTrials);
    chosenW = NaN;
else
    chosenW = weights(cand);
    fprintf('\nRule picks w_c = %g (smallest weight meeting tilt <= %g deg and success >= w_c=0 minus one seed).\n', ...
        chosenW, tiltLimit);
end
fprintf('Weights meeting the rule: %s\n', mat2str(weights(okTilt & okSucc & weights > 0)));

figA = figure('Name', 'Task5 weight selection', 'Color', 'w', 'Position', [100 100 900 380]);
subplot(1, 2, 1); bar(categorical(string(weights), string(weights)), 100 * nS' / nTrials); grid on; ylim([0 105]);
xlabel('w_c'); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials)); legend(weightScen, 'Interpreter', 'none', 'Location', 'south');
title('Obstacle avoidance');
subplot(1, 2, 2); plot(1:numel(weights), medT', '-o', 'LineWidth', 2); hold on;
yline(tiltLimit, 'k--', sprintf('%g deg limit', tiltLimit));
set(gca, 'XTick', 1:numel(weights), 'XTickLabel', string(weights)); grid on;
xlabel('w_c'); ylabel('Median of max cup tilt (deg)'); legend(weightScen, 'Interpreter', 'none');
title('Cup tilt');
sgtitle(sprintf('Choosing the constraint weight (rule picks w_c = %g)', chosenW));

%% B. Board height screening (constraint on)
fprintf('\n=== B. Board height screening, w_c = %g ===\n', qcWeightMain);
rateB = zeros(size(boardZs)); tiltB = rateB;
m = base; m.qcWeight = qcWeightMain;
for k = 1:numel(boardZs)
    sc = helperScenarioCup('board_low', struct('boardZ', boardZs(k)));
    [world, vw] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
    out = runTrials(robot, q0, qT, vw, world, m, opts, seeds, ee, clrIdx);
    rateB(k) = mean(out.success); tiltB(k) = median(out.maxTilt);
    fprintf('board underside %.2f m: success %d/%d, median max tilt %.1f deg\n', ...
        boardZs(k), sum(out.success), nTrials, tiltB(k));
    rows = addRow(rows, 'B board height', sprintf('underside %.2f m, w_c = %g', boardZs(k), qcWeightMain), out);
end
figB = figure('Name', 'Task5 board screening', 'Color', 'w');
bar(100 * boardZs, 100 * rateB, 0.5); grid on; ylim([0 105]);
xlabel('Board underside height (cm)'); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials));
title(sprintf('Screening: where the constrained planner breaks (w_c = %g)', qcWeightMain));

%% C. board_high with a via-point initial path above the board
fprintf('\n=== C. board_high: via-point initial path, w_c = %g ===\n', qcWeightMain);
sc = helperScenarioCup('board_high');
[world, vw] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
qV = ur5eSolveIK(robot, ee, sc.taskVia, sc.seedVia);
thVia = viaPointInit(q0, qV, qT, opts.nDiscretize);
cVia = stompCheckCollision(robot, thVia, world);
fprintf('Via-point initial path: %d/20 waypoints colliding (still non-trivial if > 0)\n', cVia.nWp);
optsVia = opts; optsVia.thetaInit = thVia;
outVia = runTrials(robot, q0, qT, vw, world, m, optsVia, seeds, ee, clrIdx);
fprintf('board_high, via-point init: success %d/%d, median max tilt %.1f deg (straight-line init: see B, %.2f m)\n', ...
    sum(outVia.success), nTrials, median(outVia.maxTilt), boardZs(end));
rows = addRow(rows, 'C board_high init', sprintf('via-point init, w_c = %g', qcWeightMain), outVia);

[~, best] = max(outVia.success - 1e-3 * outVia.maxTilt);
figC = plotTrajectoryOverlay(robot, outVia.theta{best}, thVia, world, ee, ...
    sprintf('board_high, via-point start (red dashed) | %s, max tilt %.0f deg', ...
    ternary(outVia.success(best), 'collision-free', 'IN COLLISION'), outVia.maxTilt(best)), ...
    struct('view', sc.view, 'axisLimits', sc.axisLimits, 'showEEAxis', true));

%% Save
T = struct2table(rows); disp(T);
writetable(T, fullfile(resultsDir, 'Task5_extras.csv'));
save(fullfile(resultsDir, 'Task5_extras.mat'), 'rows', 'weights', 'weightScen', 'nS', 'medT', ...
    'chosenW', 'boardZs', 'rateB', 'tiltB', 'outVia', 'thVia');
if saveFigures
    saveFigureSafe(figA, fullfile(resultsDir, 'Task5_extras_weight_selection.png'));
    saveFigureSafe(figB, fullfile(resultsDir, 'Task5_extras_board_screening.png'));
    saveFigureSafe(figC, fullfile(resultsDir, 'Task5_extras_board_high_via.png'));
end
fprintf('\nDone. Results in %s\n', resultsDir);

%% ---- local functions ----
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

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
