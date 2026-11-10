
function res = stompPlan(robot, q0, qT, voxel_world, model, opts)
% STOMP trajectory optimization for UR5e
% Tracks total, obstacle and smoothness costs.

if nargin < 6
    opts = struct();
end

%% Optimization parameters
nDiscretize = getOpt(opts, 'nDiscretize', 20);
nPaths = getOpt(opts, 'nPaths', 20);
convergenceThreshold = getOpt(opts, 'convergenceThreshold', 0.1);
maxIterations = getOpt(opts, 'maxIterations', 50);
verbose = getOpt(opts, 'verbose', true);

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

%% Initial costs
[~, Qtheta] = stompTrajCost( ...
    robot, theta, R, voxel_world, model);

Qtheta_initial = Qtheta;

th0 = theta_initial(:, 2:end-1);
smoothness_initial = 0.5 * sum(th0 * R * th0', 'all');
obstacle_initial = Qtheta_initial - smoothness_initial;

%% Initialize optimization history
QthetaOld = 0;

Q_time = [];
RAR_time = [];
obstacle_time = [];
iteration_time = [];

theta_history = {};
iter = 0;
stopReason = 'converged (|dQ| < threshold)';

%% STOMP optimization loop
while abs(Qtheta - QthetaOld) > convergenceThreshold

    iter = iter + 1;
    QthetaOld = Qtheta;
    tStart = tic;

    % Generate random trajectories
    [theta_samples, em] = ...
        stompSamples(nPaths, Rinv, theta);

    % Evaluate trajectory costs
    Stheta = zeros(nPaths, nDiscretize);

    for k = 1:nPaths
        Stheta(k,:) = stompTrajCost( ...
            robot, theta_samples{k}, ...
            R, voxel_world, model);
    end

    % Calculate probabilities
    trajProb = stompUpdateProb(Stheta);

    % Calculate weighted update
    dtheta = stompDTheta(trajProb, em);

    % Update trajectory
    [theta, dtheta_smoothed] = ...
        stompUpdateTheta(theta, dtheta, M);

    % Evaluate updated trajectory
    [~, Qtheta] = stompTrajCost( ...
        robot, theta, R, voxel_world, model);

    %% Record costs
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
        fprintf(['iter %2d | Total = %8.3f | ' ...
                 'Obstacle = %8.3f | ' ...
                 'Smoothness = %8.3f | %.2f s\n'], ...
            iter, Qtheta, obstacle_time(end), ...
            RAR_time(end), iteration_time(end));
    end

    %% Stopping conditions
    if iter >= maxIterations
        stopReason = 'maximum iterations';
        break;
    end

    if norm(dtheta_smoothed, 'fro') < 1e-10
        stopReason = 'update ~ 0';
        break;
    end
end

%% Final costs
thFinal = theta(:, 2:end-1);

smoothness_final = ...
    0.5 * sum(thFinal * R * thFinal', 'all');

obstacle_final = Qtheta - smoothness_final;

%% Store results
res.theta = theta;
res.theta_initial = theta_initial;

res.Q_time = Q_time;
res.RAR_time = RAR_time;
res.obstacle_time = obstacle_time;

res.Qtheta_initial = Qtheta_initial;
res.Qtheta = Qtheta;

res.smoothness_initial = smoothness_initial;
res.smoothness_final = smoothness_final;

res.obstacle_initial = obstacle_initial;
res.obstacle_final = obstacle_final;

res.iteration_time = iteration_time;
res.iter = iter;
res.theta_history = theta_history;
res.stopReason = stopReason;
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
