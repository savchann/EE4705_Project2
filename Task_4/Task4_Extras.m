%% *Task 4 extra experiments: stopping rule and scenario screening*
% A. Stopping rule, before vs after (extension to Task 1):
%    the same four scenarios and the same seeds, planned once with the
%    original Task 1 rule (stop when |dQ| < 0.1, max 50 iterations) and once
%    with the improved rule (stop when STOMP's sphere model is collision-free
%    and the cost has settled; give up when stalled; max 150 iterations).
% B. Scenario screening (how the final pillar scenario was chosen): the
%    pillar distance is a difficulty knob. Closer = less room between base
%    and pillar. Run with the improved rule.
%
% Run time: 4 x nTrials (original rule, ~25 s each) + 4 x nTrials and
% 3 x nTrials (improved rule, ~45 s each): about 15 min with nTrials = 3.
% Results: Task_4/Results/Task4_extras_*.png, Task4_extras.csv, .mat

clear; clc; close all;

%% Settings
nTrials     = 3;                     % seeds per setting
scenarios   = {'pillar', 'clutter', 'window', 'shelf'};
pillarXs    = [0.42 0.55 0.65];      % candidate pillar distances (m)
saveFigures = true;

%% Paths
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
addpath(task1Dir); addpath(fullfile(rootDir, 'Task_2')); addpath(thisDir);
resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

robot = loadrobot('universalUR5e', 'DataFormat', 'column');
ee = 'tool0';
model.spheres = ur5eSphereModel(robot);
optsOld = struct('nDiscretize', 20, 'nPaths', 20, 'stopRule', 'original', ...
                 'convergenceThreshold', 0.1, 'maxIterations', 50);
optsNew = struct('nDiscretize', 20, 'nPaths', 20, 'stopRule', 'improved', 'maxIterations', 150);
seeds = 1:nTrials;
rows = struct([]);

%% A. Original vs improved stopping rule
fprintf('\n=== A. Stopping rule: original vs improved (%d seeds each) ===\n', nTrials);
rateOld = zeros(1, numel(scenarios)); rateNew = rateOld;
for s = 1:numel(scenarios)
    sc = helperScenarioUR5e(scenarios{s});
    [world, vw] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
    cInit = stompCheckCollision(robot, straightLine(q0, qT, 20), world);

    outOld = runTrials(robot, q0, qT, vw, world, model, optsOld, seeds, ee, clrIdx);
    outNew = runTrials(robot, q0, qT, vw, world, model, optsNew, seeds, ee, clrIdx);
    rateOld(s) = mean(outOld.success); rateNew(s) = mean(outNew.success);
    fprintf('%-8s original: %d/%d (mean %.0f it) | improved: %d/%d (mean %.0f it)\n', scenarios{s}, ...
        sum(outOld.success), nTrials, mean(outOld.iter), sum(outNew.success), nTrials, mean(outNew.iter));
    rows = addRow(rows, 'A stop rule', sprintf('%s, original rule', scenarios{s}), cInit.nWp, outOld);
    rows = addRow(rows, 'A stop rule', sprintf('%s, improved rule', scenarios{s}), cInit.nWp, outNew);
end
figA = figure('Name', 'Task4 stop rule', 'Color', 'w');
bar(categorical(scenarios, scenarios), 100 * [rateOld; rateNew]'); grid on; ylim([0 105]);
ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials));
legend({'original rule (|\DeltaQ| < 0.1, \leq 50 it)', 'improved rule'}, 'Location', 'northoutside');
title('Effect of the stopping rule (same scenarios, same seeds)');

%% B. Pillar distance screening (improved rule)
fprintf('\n=== B. Pillar distance screening (%d seeds each) ===\n', nTrials);
rateB = zeros(size(pillarXs));
for k = 1:numel(pillarXs)
    sc = helperScenarioUR5e('pillar', struct('pillarX', pillarXs(k)));
    [world, vw] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, ee, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, ee, sc.taskFinal, sc.seedFinal);
    cInit = stompCheckCollision(robot, straightLine(q0, qT, 20), world);
    out = runTrials(robot, q0, qT, vw, world, model, optsNew, seeds, ee, clrIdx);
    rateB(k) = mean(out.success);
    fprintf('pillar at %.2f m: initial path %2d/20 colliding | success %d/%d | mean %.0f iterations\n', ...
        pillarXs(k), cInit.nWp, sum(out.success), nTrials, mean(out.iter));
    rows = addRow(rows, 'B pillar screening', sprintf('pillarX = %.2f m', pillarXs(k)), cInit.nWp, out);
end
figB = figure('Name', 'Task4 pillar screening', 'Color', 'w');
bar(pillarXs, 100 * rateB, 0.5); grid on; ylim([0 105]);
xlabel('Pillar distance from base (m)'); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials));
title('Scenario screening: pillar distance as a difficulty knob');

%% Save
T = struct2table(rows); disp(T);
writetable(T, fullfile(resultsDir, 'Task4_extras.csv'));
save(fullfile(resultsDir, 'Task4_extras.mat'), 'rows', 'scenarios', 'rateOld', 'rateNew', 'pillarXs', 'rateB');
if saveFigures
    saveFigureSafe(figA, fullfile(resultsDir, 'Task4_extras_stop_rule.png'));
    saveFigureSafe(figB, fullfile(resultsDir, 'Task4_extras_pillar_screening.png'));
end
fprintf('\nDone. Results in %s\n', resultsDir);

%% ---- local functions ----
function rows = addRow(rows, experiment, setting, initWp, out)
r.experiment      = string(experiment);
r.setting         = string(setting);
r.initCollidingWp = initWp;
r.nSeeds          = numel(out.success);
r.nSuccess        = sum(out.success);
r.meanIter        = mean(out.iter);
r.meanTimeSec     = mean(out.time);
r.medianMinClr    = median(out.minClr);
rows = [rows; r];
end

function th = straightLine(q0, qT, N)
th = zeros(numel(q0), N);
for k = 1:numel(q0), th(k, :) = linspace(q0(k), qT(k), N); end
end
