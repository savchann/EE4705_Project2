%% STOMP planner as a function
% This is the optimisation loop of the Task 1 helperSTOMP.m, unchanged in
% algorithm and parameters, wrapped in a function so that Tasks 4 and 5 can
% run many scenarios / settings in a loop without workspace clashes.
%   1. sample noisy trajectories        (stompSamples,     Task 1)
%   2. local cost of each sample        (stompTrajCost)
%   3. probabilities                    (stompUpdateProb,  Task 1)
%   4. probability-weighted noise       (stompDTheta,      Task 1)
%   5. smoothed update                  (stompUpdateTheta, Task 1)
%   6. cost of the new trajectory; stop when |dQ| < threshold or 50 iterations
%
% INPUT:
%   robot        : rigidBodyTree ('column')
%   q0, qT       : start and goal joint configurations (column vectors)
%   voxel_world  : voxel world struct
%   model        : struct passed on to stompTrajCost (spheres, constraint...)
%   opts         : (optional) struct with fields nDiscretize, nPaths,
%                  convergenceThreshold, maxIterations, verbose, thetaInit
% OUTPUT res struct:
%   theta, theta_initial, Q_time, RAR_time, iteration_time, iter,
%   Qtheta_initial, Qtheta, theta_history, stopReason
function res = stompPlan(robot, q0, qT, voxel_world, model, opts)

if nargin < 6, opts = struct(); end
nDiscretize          = getOpt(opts, 'nDiscretize', 20);
nPaths               = getOpt(opts, 'nPaths', 20);
convergenceThreshold = getOpt(opts, 'convergenceThreshold', 0.1);
maxIterations        = getOpt(opts, 'maxIterations', 50);
verbose              = getOpt(opts, 'verbose', true);

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
QthetaOld = 0;
Q_time = []; RAR_time = []; iteration_time = [];
theta_history = {};
iter = 0;
stopReason = 'converged (|dQ| < threshold)';

while abs(Qtheta - QthetaOld) > convergenceThreshold
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
        fprintf('  iter %2d | Q = %9.3f | RAR = %7.3f | %.2f s\n', ...
            iter, Qtheta, RAR_time(end), iteration_time(end));
    end

    if iter >= maxIterations
        stopReason = 'maximum iterations';
        break;
    end
    if norm(dtheta_smoothed, 'fro') < 1e-10
        stopReason = 'update ~ 0';
        break;
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
res.R              = R;

end

function v = getOpt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
