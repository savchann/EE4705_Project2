%% *Task 5: Keep the end-effector y-axis upright ("carry a cup")*
% Adds an orientation constraint cost to STOMP (see stompTrajCost.m in this
% folder) and compares planning WITHOUT and WITH the constraint on the same
% scenario, same start/goal, same random seed. The scenarios are the Task 4
% scenarios (same obstacles), with cup-upright start and goal poses
% (helperScenarioCup.m):
%   window  - Task 4 window, unchanged
%   shelf   - Task 4 shelf, unchanged
%   pillar  - Task 4 pillar, cup-upright endpoints
%   clutter - Task 4 cluttered table, cup carried over the divider
%
% Outputs (Task_5/Results):
%   - overlay figures with the end-effector y-axis drawn at every waypoint
%   - one comparison figure per scenario (without vs with the constraint)
%   - tilt-angle-along-the-path plot, unconstrained vs constrained
%   - summary table (.csv)
%   - optional seed study (success rate + tilt over several seeds)
%   - optional weight sweep: success rate and max tilt vs constraint weight
%
% Uses: Task 1 (STOMP core), Task_2 (UR5e port), Task_4 (scenarios,
% buildVoxelWorld, plotTrajectoryOverlay). This folder is added to the
% path last, so its stompTrajCost.m (with the constraint) is the one used.

clear; clc; close all;

%% Settings
scenarioNames  = {'window', 'pillar', 'clutter'};
qcWeight       = 10;        % constraint weight w_c (0 = constraint off)
runSeedStudy   = true;      % repeat both settings with nTrials seeds
nTrials        = 3;
runWeightSweep = false;     % success / tilt vs w_c on one scenario (Task5_Extras.m does a 2-scenario version)
stopRule       = 'improved';  % see Task_2/stompPlan.m
sweepScenario  = 'clutter';
sweepWeights   = [0 2 5 10 20 50];
saveFigures    = true;

%% Paths
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
task2Dir = fullfile(rootDir, 'Task_2');
task4Dir = fullfile(rootDir, 'Task_4');
assert(isfolder(task1Dir) && isfolder(task2Dir) && isfolder(task4Dir), ...
    'Task 1, Task_2 and Task_4 folders must sit next to Task_5.');
addpath(task1Dir); addpath(task2Dir); addpath(task4Dir); addpath(thisDir);
resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

% Make sure the constrained cost function is the one MATLAB will call
w = which('stompTrajCost');
assert(startsWith(w, thisDir), 'Wrong stompTrajCost on the path: %s', w);

%% Robot
robot = loadrobot('universalUR5e', 'DataFormat', 'column');
endEffector = 'tool0';
baseModel.spheres = ur5eSphereModel(robot);
baseModel.eeName  = endEffector;
baseModel.upAxis  = 2;          % end-effector y-axis ...
baseModel.worldUp = [0; 0; 1];  % ... aligned with world z

opts.nDiscretize = 20;
opts.nPaths = 20;
opts.convergenceThreshold = 0.1;           % used only by the original rule
opts.stopRule = stopRule;
if strcmpi(stopRule, 'original'), opts.maxIterations = 50; else, opts.maxIterations = 150; end

modes   = [0, qcWeight];
modeTxt = {'no constraint', sprintf('constraint (w_c = %g)', qcWeight)};
summary = struct([]);

%% Main comparison
for s = 1:numel(scenarioNames)
    sc = helperScenarioCup(scenarioNames{s});
    fprintf('\n==================== %s ====================\n%s\n', sc.title, sc.description);
    [world, voxel_world] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, endEffector, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, endEffector, sc.taskFinal, sc.seedFinal);
    fprintf('Endpoint clearance (meshes): start %.3f m, goal %.3f m\n', ...
        endpointClearance(robot, q0, world, clrIdx), endpointClearance(robot, qT, world, clrIdx));
    theta_init = cell2mat(arrayfun(@(k) linspace(q0(k), qT(k), opts.nDiscretize), (1:6)', 'UniformOutput', false));
    cInit = stompCheckCollision(robot, theta_init, world);
    tInit = eeTiltDeg(robot, theta_init, endEffector);
    fprintf('Initial path: %d/%d colliding waypoints, max tilt %.1f deg\n', cInit.nWp, opts.nDiscretize, max(tInit));

    colors = [0.85 0.2 0.2; 0.1 0.45 0.85];
    thetaByMode = cell(1, numel(modes)); tiltByMode = cell(1, numel(modes)); freeByMode = false(1, numel(modes));
    sByMode = cell(1, numel(modes)); collByMode = cell(1, numel(modes));

    for m = 1:numel(modes)
        model = baseModel; model.qcWeight = modes(m);
        fprintf('\n-- %s --\n', modeTxt{m});
        rng(42, 'twister');
        res = stompPlan(robot, q0, qT, voxel_world, model, setfield(opts, 'verbose', false)); %#ok<SFLD>
        c = stompCheckCollision(robot, res.theta, world, [], clrIdx);
        [tilt, sPath] = eeTiltDeg(robot, res.theta, endEffector);
        fprintf('%d iterations (%s) | %s | max tilt %.1f deg, mean tilt %.1f deg | min clearance %.3f m\n', ...
            res.iter, res.stopReason, ternary(c.free, 'COLLISION-FREE', sprintf('IN COLLISION (%d dense)', c.nDense)), ...
            max(tilt), mean(tilt), c.minClearance);

        % overlay with the cup axis at every waypoint
        figO = plotTrajectoryOverlay(robot, res.theta, theta_init, world, endEffector, ...
            sprintf('%s | %s | %s, max tilt %.0f deg', sc.title, modeTxt{m}, ...
            ternary(c.free, 'collision-free', 'IN COLLISION'), max(tilt)), ...
            struct('view', sc.view, 'axisLimits', sc.axisLimits, 'showEEAxis', true, ...
                   'step', 3, 'figName', sprintf('Task5 %s w=%g', sc.name, modes(m))));
        if saveFigures
            saveFigureSafe(figO, fullfile(resultsDir, sprintf('Task5_%s_w%g_overlay.png', sc.name, modes(m))));
        end

        % seed study
        rate = NaN; medTilt = NaN;
        if runSeedStudy
            ok = false(nTrials, 1); mt = zeros(nTrials, 1);
            for t = 1:nTrials
                rng(t, 'twister');
                r = stompPlan(robot, q0, qT, voxel_world, model, setfield(opts, 'verbose', false)); %#ok<SFLD>
                cc = stompCheckCollision(robot, r.theta, world, [], clrIdx);
                ok(t) = cc.free; mt(t) = max(eeTiltDeg(robot, r.theta, endEffector, 4));
            end
            rate = mean(ok); medTilt = median(mt);
            fprintf('Seed study (%d seeds): success %d/%d, median of max tilt %.1f deg\n', nTrials, sum(ok), nTrials, medTilt);
        end

        row.scenario        = string(sc.name);
        row.constraintW     = modes(m);
        row.success_rng42   = c.free;
        row.collidingDense  = c.nDense;
        row.minClearance    = c.minClearance;
        row.maxTiltDeg      = max(tilt);
        row.meanTiltDeg     = mean(tilt);
        row.iterations      = res.iter;
        row.timeSec         = sum(res.iteration_time);
        row.costFinal       = res.Qtheta;
        row.successRate     = rate;
        row.medianMaxTilt   = medTilt;
        summary = [summary; row]; %#ok<AGROW>
        save(fullfile(resultsDir, sprintf('Task5_%s_w%g.mat', sc.name, modes(m))), 'res', 'c', 'tilt', 'q0', 'qT', 'sc');
        thetaByMode{m} = res.theta; tiltByMode{m} = tilt; freeByMode(m) = c.free;
        sByMode{m} = sPath; collByMode{m} = c.denseCollision;
    end

    % One picture of the trajectory change: without vs with the constraint
    figCmp = plotConstraintComparison(robot, thetaByMode{1}, thetaByMode{2}, world, endEffector, ...
        max(tiltByMode{1}), max(tiltByMode{2}), freeByMode, sc, qcWeight);
    if saveFigures
        saveFigureSafe(figCmp, fullfile(resultsDir, sprintf('Task5_%s_comparison.png', sc.name)));
    end

    % tilt along the path, both runs (drawn in one go from the stored data)
    figTilt = figure('Name', ['Task5 tilt ' sc.name], 'Color', 'w');
    axT = axes('Parent', figTilt); hold(axT, 'on'); grid(axT, 'on');
    for m = 1:numel(modes)
        plot(axT, sByMode{m}, tiltByMode{m}, '-', 'Color', colors(m, :), 'LineWidth', 2, 'DisplayName', modeTxt{m});
        if any(collByMode{m})
            plot(axT, sByMode{m}(collByMode{m}), tiltByMode{m}(collByMode{m}), 'x', 'Color', colors(m, :), ...
                'MarkerSize', 6, 'HandleVisibility', 'off');
        end
    end
    plot(axT, linspace(1, opts.nDiscretize, numel(tInit)), tInit, 'k:', 'LineWidth', 1.2, 'DisplayName', 'initial straight line');
    xlabel(axT, 'Path progress (waypoint index)'); ylabel(axT, 'Cup tilt (deg)  [0 = upright]');
    title(axT, {sc.title, 'Tilt of end-effector y-axis from vertical (x = in collision)'}, 'Interpreter', 'none');
    legend(axT, 'Location', 'best');
    if saveFigures
        saveFigureSafe(figTilt, fullfile(resultsDir, sprintf('Task5_%s_tilt.png', sc.name)));
    end
end

T = struct2table(summary);
disp(T);
writetable(T, fullfile(resultsDir, 'Task5_summary.csv'));

%% Weight sweep: the trade-off between staying upright and avoiding obstacles
if runWeightSweep
    sc = helperScenarioCup(sweepScenario);
    [world, voxel_world] = buildVoxelWorld(sc.shapes);
    clrIdx = find(~strcmp({sc.shapes.name}, 'table'));
    q0 = ur5eSolveIK(robot, endEffector, sc.taskInit, sc.seedInit);
    qT = ur5eSolveIK(robot, endEffector, sc.taskFinal, sc.seedFinal);
    rate = zeros(size(sweepWeights)); medTilt = zeros(size(sweepWeights));
    for k = 1:numel(sweepWeights)
        model = baseModel; model.qcWeight = sweepWeights(k);
        ok = false(nTrials, 1); mt = zeros(nTrials, 1);
        for t = 1:nTrials
            rng(t, 'twister');
            r = stompPlan(robot, q0, qT, voxel_world, model, setfield(opts, 'verbose', false)); %#ok<SFLD>
            cc = stompCheckCollision(robot, r.theta, world, [], clrIdx);
            ok(t) = cc.free; mt(t) = max(eeTiltDeg(robot, r.theta, endEffector, 4));
        end
        rate(k) = mean(ok); medTilt(k) = median(mt);
        fprintf('w_c = %5g: success %d/%d, median max tilt %.1f deg\n', sweepWeights(k), sum(ok), nTrials, medTilt(k));
    end
    figS = figure('Name', 'Task5 weight sweep', 'Color', 'w');
    xk = 1:numel(sweepWeights);
    yyaxis left;  bar(xk, 100 * rate, 0.5); ylabel(sprintf('Collision-free (%% of %d seeds)', nTrials)); ylim([0 105]);
    yyaxis right; plot(xk, medTilt, '-o', 'LineWidth', 2); ylabel('Median of max cup tilt (deg)');
    set(gca, 'XTick', xk, 'XTickLabel', string(sweepWeights)); xlabel('Constraint weight w_c');
    grid on; title(['Trade-off: ' sc.title]);
    if saveFigures
        saveFigureSafe(figS, fullfile(resultsDir, 'Task5_weight_sweep.png'));
    end
    save(fullfile(resultsDir, 'Task5_weight_sweep.mat'), 'sweepWeights', 'rate', 'medTilt');
end

fprintf('\nTask 5 done. Results in %s\n', resultsDir);

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end