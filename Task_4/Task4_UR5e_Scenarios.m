%% *Task 4: Custom collision-avoidance scenarios for the UR5e*
% Four scenarios of increasing difficulty, each with a straight-line
% initial path that passes through an obstacle (non-trivial initialisation):
%   1. pillar  - swing around a tall pillar            (large detour, seed-dependent)
%   2. clutter - pick-and-place across a cluttered table (many obstacles, tight clearances)
%   3. window  - reach through a window in a wall       (narrow passage)
%   4. shelf   - lower to upper shelf compartment       (needs pull-out / move / push-in)
%
% For every scenario the script:
%   - builds the obstacles (buildVoxelWorld: collision objects + voxel SDF)
%   - solves IK for start/goal and checks the initial path really collides
%   - runs STOMP once with rng(42) and verifies the result with MATLAB's
%     mesh-based checkCollision (at the waypoints and densely in between)
%   - saves an overlay figure of the intermediate configurations and the
%     convergence curve
%   - optionally repeats the planning with nTrials different random seeds
%     and reports the success rate: STOMP is stochastic, so one lucky (or
%     unlucky) run says little about a scenario
%   - optionally sweeps the window size to show where the narrow passage
%     breaks the planner
% Results (figures, .mat, summary .csv) go to Task_4/Results.
%
% Uses: Task 1 folder (unchanged STOMP core), Task_2 folder (UR5e port).

clear; clc; close all;

%% Settings
scenarioNames = {'pillar', 'clutter', 'window', 'shelf'};
runSeedStudy    = false;    % repeat each scenario with nTrials random seeds
nTrials         = 5;
runWindowSweep  = false;    % success rate vs. window size
windowSizes     = [0.34 0.30 0.27];
animate         = false;   % play the planned motion after each scenario
saveVideo       = false;   % write an .mp4 of each planned motion
saveFigures     = true;

%% Paths
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
task2Dir = fullfile(rootDir, 'Task_2');
assert(isfolder(task1Dir) && isfolder(task2Dir), 'Task 1 and Task_2 folders must sit next to Task_4.');
addpath(task1Dir); addpath(task2Dir); addpath(thisDir);   % last added is searched first
resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

%% Robot
robot_name = 'universalUR5e';
robot = loadrobot(robot_name, 'DataFormat', 'column');
endEffector = 'tool0';
model.spheres = ur5eSphereModel(robot);

opts.nDiscretize = 20;
opts.nPaths = 20;
opts.convergenceThreshold = 0.1;
opts.maxIterations = 50;

%% Main runs
summary = struct([]);
for s = 1:numel(scenarioNames)
    sc = helperScenarioUR5e(scenarioNames{s});
    fprintf('\n==================== %s ====================\n%s\n', sc.title, sc.description);

    [world, voxel_world] = buildVoxelWorld(sc.shapes);
    [q0, i0] = ur5eSolveIK(robot, endEffector, sc.taskInit, sc.seedInit);
    [qT, iT] = ur5eSolveIK(robot, endEffector, sc.taskFinal, sc.seedFinal);
    fprintf('IK: start err %.1e m, goal err %.1e m\n', i0.positionError, iT.positionError);

    % Start/goal must be collision-free, the straight line must not be
    startFree = ~checkCollision(robot, q0, world, "IgnoreSelfCollision", "on");
    goalFree  = ~checkCollision(robot, qT, world, "IgnoreSelfCollision", "on");
    theta_init = cell2mat(arrayfun(@(k) linspace(q0(k), qT(k), opts.nDiscretize), (1:6)', 'UniformOutput', false));
    cInit = stompCheckCollision(robot, theta_init, world);
    fprintf('Start free: %d, goal free: %d, initial path colliding waypoints: %d/%d\n', ...
        startFree, goalFree, cInit.nWp, opts.nDiscretize);
    if cInit.nWp == 0
        warning('Scenario %s: the initial path is collision-free (trivial). Adjust the obstacles.', sc.name);
    end

    % Planning (reproducible run)
    rng(42, 'twister');
    res = stompPlan(robot, q0, qT, voxel_world, model, opts);
    cRes = stompCheckCollision(robot, res.theta, world);
    fprintf('-> %d iterations (%s), %.1f s, cost %.1f -> %.1f\n', res.iter, res.stopReason, ...
        sum(res.iteration_time), res.Qtheta_initial, res.Qtheta);
    fprintf('-> colliding waypoints %d/%d, dense %d/%d, min clearance %.3f m => %s\n', ...
        cRes.nWp, opts.nDiscretize, cRes.nDense, numel(cRes.denseCollision), cRes.minClearance, ...
        ternary(cRes.free, 'SUCCESS', 'FAILED'));

    % Figures
    status = ternary(cRes.free, 'collision-free', sprintf('IN COLLISION (%d/%d dense checks)', ...
        cRes.nDense, numel(cRes.denseCollision)));
    figO = plotTrajectoryOverlay(robot, res.theta, theta_init, world, endEffector, ...
        sprintf('%s  [rng 42: %s]', sc.title, status), ...
        struct('view', sc.view, 'axisLimits', sc.axisLimits, 'figName', ['Task4 ' sc.name]));
    figC = figure('Name', ['Task4 convergence ' sc.name], 'Color', 'w');
    yyaxis left;  plot(res.Q_time, '-o', 'LineWidth', 1.5); ylabel('Total cost Q(\theta)');
    yyaxis right; plot(res.RAR_time, '-s', 'LineWidth', 1.2); ylabel('Smoothness cost');
    xlabel('Iteration'); grid on; title(['Convergence: ' sc.title]);
    if saveFigures
        exportgraphics(figO, fullfile(resultsDir, ['Task4_' sc.name '_overlay.png']), 'Resolution', 200);
        exportgraphics(figC, fullfile(resultsDir, ['Task4_' sc.name '_convergence.png']), 'Resolution', 200);
    end

    if animate || saveVideo
        playOrRecord(robot, res.theta, world, sc, saveVideo, fullfile(resultsDir, ['Task4_' sc.name '.mp4']));
    end

    % Seed study
    nSuccess = NaN; meanIter = NaN;
    if runSeedStudy
        ok = false(nTrials, 1); its = zeros(nTrials, 1);
        for t = 1:nTrials
            rng(t, 'twister');
            r = stompPlan(robot, q0, qT, voxel_world, model, setfield(opts, 'verbose', false)); %#ok<SFLD>
            c = stompCheckCollision(robot, r.theta, world);
            ok(t) = c.free; its(t) = r.iter;
            fprintf('   seed %d: %s after %d iterations\n', t, ternary(c.free, 'success', 'collision'), r.iter);
        end
        nSuccess = sum(ok); meanIter = mean(its);
    end

    % Store
    row.scenario          = string(sc.name);
    row.initCollidingWp   = cInit.nWp;
    row.iterations        = res.iter;
    row.stopReason        = string(res.stopReason);
    row.timeSec           = sum(res.iteration_time);
    row.costInitial       = res.Qtheta_initial;
    row.costFinal         = res.Qtheta;
    row.collidingWp       = cRes.nWp;
    row.collidingDense    = cRes.nDense;
    row.minClearance      = cRes.minClearance;
    row.success_rng42     = cRes.free;
    row.successRate       = nSuccess / nTrials;
    row.meanIterSeeds     = meanIter;
    summary = [summary; row]; %#ok<AGROW>

    save(fullfile(resultsDir, ['Task4_' sc.name '.mat']), 'res', 'cRes', 'cInit', 'q0', 'qT', 'sc');
end

%% Summary table
T = struct2table(summary);
disp(T);
writetable(T, fullfile(resultsDir, 'Task4_summary.csv'));

%% Window-size sweep: where does the narrow passage break STOMP?
if runWindowSweep
    rate = zeros(size(windowSizes));
    for w = 1:numel(windowSizes)
        sc = helperScenarioUR5e('window', struct('windowSize', windowSizes(w)));
        [world, voxel_world] = buildVoxelWorld(sc.shapes);
        q0 = ur5eSolveIK(robot, endEffector, sc.taskInit, sc.seedInit);
        qT = ur5eSolveIK(robot, endEffector, sc.taskFinal, sc.seedFinal);
        if checkCollision(robot, qT, world, "IgnoreSelfCollision", "on")
            warning('Window %.2f m: goal configuration itself collides, skipping.', windowSizes(w));
            rate(w) = NaN; continue;
        end
        ok = 0;
        for t = 1:nTrials
            rng(t, 'twister');
            r = stompPlan(robot, q0, qT, voxel_world, model, setfield(opts, 'verbose', false)); %#ok<SFLD>
            c = stompCheckCollision(robot, r.theta, world);
            ok = ok + c.free;
        end
        rate(w) = ok / nTrials;
        fprintf('Window %.2f m: success %d/%d\n', windowSizes(w), ok, nTrials);
    end
    figW = figure('Name', 'Task4 window sweep', 'Color', 'w');
    bar(100 * windowSizes, 100 * rate); grid on;
    xlabel('Window size (cm)'); ylabel(sprintf('Success rate over %d seeds (%%)', nTrials));
    title('Narrow passage: STOMP success vs. window size');
    if saveFigures
        exportgraphics(figW, fullfile(resultsDir, 'Task4_window_sweep.png'), 'Resolution', 200);
    end
end

fprintf('\nTask 4 done. Results in %s\n', resultsDir);

%% ---- local functions ----
function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

function playOrRecord(robot, theta, world, sc, record, fileName)
fig = figure('Name', ['Task4 animation ' sc.name], 'Color', 'w');
ax = show(robot, theta(:, 1), 'PreservePlot', false, 'Frames', 'off');
hold(ax, 'on');
for i = 1:numel(world)
    [~, p] = show(world{i}, 'Parent', ax);
    p.FaceAlpha = 0.4; p.EdgeColor = 'none';
end
view(ax, sc.view); axis(ax, sc.axisLimits);
if record
    v = VideoWriter(fileName, 'MPEG-4'); v.FrameRate = 8; open(v);
end
for t = 1:size(theta, 2)
    show(robot, theta(:, t), 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    title(ax, sprintf('%s  (waypoint %d/%d)', sc.title, t, size(theta, 2)));
    drawnow;
    if record, writeVideo(v, getframe(fig)); else, pause(0.2); end
end
if record, close(v); end
end
