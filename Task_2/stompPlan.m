
function res = stompPlan(robot, q0, qT, voxel_world, model, opts)
% STOMP trajectory optimization
%
% Supports two stopping rules:
%   'original' - Task 1 cost convergence criterion
%   'improved' - Collision-aware convergence and stagnation detection
%
% Tracks:
%   Total trajectory cost
%   Obstacle cost
%   Smoothness cost
%
% Compatible with Tasks 2, 4 and 5.

if nargin < 6
    opts = struct();
end

%% Optimization parameters

nDiscretize = getOpt(opts, 'nDiscretize', 20);
nPaths = getOpt(opts, 'nPaths', 20);

stopRule = getOpt(opts, 'stopRule', 'improved');
verbose = getOpt(opts, 'verbose', true);

convergenceThreshold = getOpt(opts, 'convergenceThreshold', 0.1);

patience = getOpt(opts, 'patience', 5);
relTol = getOpt(opts, 'relTol', 0.01);
stallIters = getOpt(opts, 'stallIters', 30);

if strcmpi(stopRule, 'original')
    maxIterations = getOpt(opts, 'maxIterations', 50);
else
    maxIterations = getOpt(opts, 'maxIterations', 150);
end

q0 = q0(:);
qT = qT(:);

numJoints = numel(q0);

%% Initial trajectory

theta = zeros(numJoints, nDiscretize);

for k = 1:numJoints
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

if isfield(opts, 'thetaInit') && ~isempty(opts.thetaInit)
    theta = opts.thetaInit;
end

theta_initial = theta;

%% Smoothness matrices

A_k = eye(nDiscretize - 1);
A = -2 * eye(nDiscretize);

A(1:nDiscretize-1, 2:nDiscretize) = ...
    A(1:nDiscretize-1, 2:nDiscretize) + A_k;

A(2:nDiscretize, 1:nDiscretize-1) = ...
    A(2:nDiscretize, 1:nDiscretize-1) + A_k;

A = A(:, 2:end-1);

R = A' * A;
Rinv = inv(R);

M = (1/nDiscretize) * Rinv ./ max(Rinv, [], 1);

Rinv = 1.5 * Rinv / sum(Rinv, 'all');

%% Initial trajectory cost

[~, Qtheta] = stompTrajCost( ...
    robot, theta, R, voxel_world, model);

Qtheta_initial = Qtheta;

th0 = theta_initial(:, 2:end-1);

smoothness_initial = ...
    0.5 * sum(th0 * R * th0', 'all');

obstacle_initial = ...
    Qtheta_initial - smoothness_initial;

%% Initialize optimization history

Q_time = [];
RAR_time = [];
obstacle_time = [];
iteration_time = [];

theta_history = {};

iter = 0;

bestQ = Qtheta;
bestIter = 0;

stopReason = '';

%% STOMP optimization loop

while isempty(stopReason)

    iter = iter + 1;

    QthetaOld = Qtheta;

    tStart = tic;

    %% 1. Generate noisy trajectories

    [theta_samples, em] = ...
        stompSamples(nPaths, Rinv, theta);

    %% 2. Evaluate sampled trajectories

    Stheta = zeros(nPaths, nDiscretize);

    for k = 1:nPaths
        Stheta(k,:) = stompTrajCost( ...
            robot, theta_samples{k}, ...
            R, voxel_world, model);
    end

    %% 3. Calculate probabilities

    trajProb = stompUpdateProb(Stheta);

    %% 4. Calculate weighted trajectory update

    dtheta = stompDTheta(trajProb, em);

    %% 5. Update trajectory

    [theta, dtheta_smoothed] = ...
        stompUpdateTheta(theta, dtheta, M);

    %% 6. Evaluate updated trajectory

    [~, Qtheta] = stompTrajCost( ...
        robot, theta, R, voxel_world, model);

    %% Record cost history

    iteration_time(end+1) = toc(tStart); %#ok<AGROW>

    Q_time(end+1) = Qtheta; %#ok<AGROW>

    th = theta(:, 2:end-1);

    RAR_time(end+1) = ...
        0.5 * sum(th * R * th', 'all'); %#ok<AGROW>

    obstacle_time(end+1) = ...
        Qtheta - RAR_time(end); %#ok<AGROW>

    theta_history{iter} = theta; %#ok<AGROW>

    %% Display progress

    if verbose
        fprintf(['iter %3d | Total = %9.3f | ' ...
                 'Obstacle = %9.3f | ' ...
                 'Smoothness = %9.3f | %.2f s\n'], ...
            iter, Qtheta, obstacle_time(end), ...
            RAR_time(end), iteration_time(end));
    end

    %% Stopping criteria

    if norm(dtheta_smoothed, 'fro') < 1e-10

        stopReason = 'update ~ 0';

    elseif strcmpi(stopRule, 'original')

        % Original Task 1 convergence rule
        if abs(Qtheta - QthetaOld) <= convergenceThreshold

            stopReason = 'converged (|dQ| < threshold)';

        elseif iter >= maxIterations

            stopReason = 'maximum iterations';

        end

    else

        % Improved stopping rule
        if Qtheta < (1 - relTol) * bestQ
            bestQ = Qtheta;
            bestIter = iter;
        end

        Qhist = [Qtheta_initial, Q_time];

        settled = iter >= patience && ...
            abs(Qhist(end) - Qhist(end-patience)) < ...
            relTol * abs(Qhist(end-patience));

        % Only stop as converged if collision-free
        if settled && ...
                stompSphereClearance( ...
                    robot, theta, voxel_world, model) > 0

            stopReason = ...
                'converged (sphere model collision-free, cost settled)';

        elseif iter - bestIter >= stallIters

            stopReason = sprintf( ...
                'stalled (no %g%% improvement in %d iterations)', ...
                100*relTol, stallIters);

        elseif iter >= maxIterations

            stopReason = 'maximum iterations';

        end
    end

end

%% Final cost components

thFinal = theta(:, 2:end-1);

smoothness_final = ...
    0.5 * sum(thFinal * R * thFinal', 'all');

obstacle_final = Qtheta - smoothness_final;

%% Store results

res.theta = theta;
res.theta_initial = theta_initial;

% Cost histories
res.Q_time = Q_time;
res.RAR_time = RAR_time;
res.obstacle_time = obstacle_time;

% Initial costs
res.Qtheta_initial = Qtheta_initial;
res.smoothness_initial = smoothness_initial;
res.obstacle_initial = obstacle_initial;

% Final costs
res.Qtheta = Qtheta;
res.smoothness_final = smoothness_final;
res.obstacle_final = obstacle_final;

% Optimization information
res.iteration_time = iteration_time;
res.iter = iter;
res.theta_history = theta_history;
res.stopReason = stopReason;
res.stopRule = stopRule;
res.R = R;

end

%% Helper function

function v = getOpt(s, name, default)

if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end

end
