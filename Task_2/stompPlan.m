%% STOMP planner as a function
% The optimisation loop of the Task 1 helperSTOMP.m, wrapped in a function
% so that Tasks 4 and 5 can run many scenarios / settings in a loop:
%   1. sample noisy trajectories        (stompSamples,     Task 1)
%   2. local cost of each sample        (stompTrajCost)
%   3. probabilities                    (stompUpdateProb,  Task 1)
%   4. probability-weighted noise       (stompDTheta,      Task 1)
%   5. smoothed update                  (stompUpdateTheta, Task 1)
%   6. cost of the new trajectory, then decide whether to stop
% Steps 1-5 are unchanged from Task 1. Only the stopping rule (step 6) can
% be switched:
%
%   opts.stopRule = 'original' (Task 1): stop as soon as |Q_k - Q_(k-1)| < 0.1,
%       or after 50 iterations. Problem found in Task 4: Q fluctuates by
%       +-2 between iterations because the update is stochastic, so a change
%       below 0.1 happens by chance while the robot is still deep inside an
%       obstacle (pillar: stopped after 8 iterations, joints moved < 0.07 rad).
%
%   opts.stopRule = 'improved' (extension, default): stop only when
%       (a) STOMP's own sphere model is collision-free at every waypoint AND
%           the cost changed by < 1% over the last 5 iterations (converged), or
%       (b) the best cost has not improved by 1% for 30 iterations (stalled:
%           stuck in a local minimum, more iterations will not help), or
%       (c) 150 iterations.
%
% INPUT:
%   robot        : rigidBodyTree ('column')
%   q0, qT       : start and goal joint configurations (column vectors)
%   voxel_world  : voxel world struct
%   model        : struct passed on to stompTrajCost (spheres, constraint...)
%   opts         : (optional) nDiscretize, nPaths, stopRule,
%                  convergenceThreshold (original), maxIterations,
%                  patience, relTol, stallIters (improved), verbose, thetaInit
% OUTPUT res struct:
%   theta, theta_initial, Q_time, RAR_time, iteration_time, iter,
%   Qtheta_initial, Qtheta, theta_history, stopReason, stopRule
function res = stompPlan(robot, q0, qT, voxel_world, model, opts)

if nargin < 6, opts = struct(); end
nDiscretize = getOpt(opts, 'nDiscretize', 20);
nPaths      = getOpt(opts, 'nPaths', 20);
stopRule    = getOpt(opts, 'stopRule', 'improved');
verbose     = getOpt(opts, 'verbose', true);
convergenceThreshold = getOpt(opts, 'convergenceThreshold', 0.1);  % original rule
patience    = getOpt(opts, 'patience', 5);                          % improved rule
relTol      = getOpt(opts, 'relTol', 0.01);
stallIters  = getOpt(opts, 'stallIters', 30);
if strcmpi(stopRule, 'original')
    maxIterations = getOpt(opts, 'maxIterations', 50);
else
    maxIterations = getOpt(opts, 'maxIterations', 150);
end

q0 = q0(:); qT = qT(:);
numJoints = numel(q0);

% Initial guess: straight line in joint space (as in Task 1)
theta = zeros(numJoints, nDiscretize);
for k = 1:numJoints
    theta(k, :) = linspace(q0(k), qT(k), nDiscretize);
end
if isfield(opts, 'thetaInit') && ~isempty(opts.thetaInit)
    theta = opts.thetaInit;
end
theta_initial = theta;

% Smoothness matrices (identical to Task 1)
A_k = eye(nDiscretize - 1);
A = -2 * eye(nDiscretize);
A(1:nDiscretize-1, 2:nDiscretize) = A(1:nDiscretize-1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize-1) = A(2:nDiscretize, 1:nDiscretize-1) + A_k;
A = A(:, 2:end-1);
R = A' * A;
Rinv = inv(R);
M = (1/nDiscretize) * Rinv ./ max(Rinv, [], 1);
Rinv = 1.5 * Rinv / sum(Rinv, 'all');

% Optimisation
[~, Qtheta] = stompTrajCost(robot, theta, R, voxel_world, model);
Qtheta_initial = Qtheta;
Q_time = []; RAR_time = []; iteration_time = [];
theta_history = {};
iter = 0;
bestQ = Qtheta; bestIter = 0;
stopReason = '';

while isempty(stopReason)
    iter = iter + 1;
    QthetaOld = Qtheta;
    tic;

    [theta_samples, em] = stompSamples(nPaths, Rinv, theta);

    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        Stheta(k, :) = stompTrajCost(robot, theta_samples{k}, R, voxel_world, model);
    end

    trajProb = stompUpdateProb(Stheta);
    dtheta = stompDTheta(trajProb, em);
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);

    [~, Qtheta] = stompTrajCost(robot, theta, R, voxel_world, model);

    iteration_time(end+1) = toc; %#ok<AGROW>
    Q_time(end+1) = Qtheta; %#ok<AGROW>
    th = theta(:, 2:nDiscretize-1);
    RAR_time(end+1) = 0.5 * sum(th * R * th', 'all'); %#ok<AGROW>
    theta_history{iter} = theta; %#ok<AGROW>

    if verbose
        fprintf('  iter %3d | Q = %9.3f | RAR = %7.3f | %.2f s\n', ...
            iter, Qtheta, RAR_time(end), iteration_time(end));
    end

    % ---- stopping rule ----
    if norm(dtheta_smoothed, 'fro') < 1e-10
        stopReason = 'update ~ 0';
    elseif strcmpi(stopRule, 'original')
        if abs(Qtheta - QthetaOld) <= convergenceThreshold
            stopReason = 'converged (|dQ| < threshold)';
        elseif iter >= maxIterations
            stopReason = 'maximum iterations';
        end
    else
        if Qtheta < (1 - relTol) * bestQ
            bestQ = Qtheta; bestIter = iter;
        end
        Qhist = [Qtheta_initial, Q_time];
        settled = iter >= patience && ...
            abs(Qhist(end) - Qhist(end - patience)) < relTol * abs(Qhist(end - patience));
        if settled && stompSphereClearance(robot, theta, voxel_world, model) > 0
            stopReason = 'converged (sphere model collision-free, cost settled)';
        elseif iter - bestIter >= stallIters
            stopReason = sprintf('stalled (no %g%% improvement in %d iterations)', 100 * relTol, stallIters);
        elseif iter >= maxIterations
            stopReason = 'maximum iterations';
        end
    end
end

res.theta          = theta;
res.theta_initial  = theta_initial;
res.Q_time         = Q_time;
res.RAR_time       = RAR_time;
res.iteration_time = iteration_time;
res.iter           = iter;
res.Qtheta_initial = Qtheta_initial;
res.Qtheta         = Qtheta;
res.theta_history  = theta_history;
res.stopReason     = stopReason;
res.stopRule       = stopRule;
res.R              = R;

end

function v = getOpt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
