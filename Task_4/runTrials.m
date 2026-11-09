%% Run STOMP for several random seeds and collect statistics
% out = runTrials(robot, q0, qT, voxel_world, world, model, opts, seeds, eeName, clrIdx)
% clrIdx: obstacles counted in the clearance number (e.g. all but the table)
% For each seed: rng(seed), plan, verify with the dense mesh collision check.
% out fields (one entry per seed):
%   success   collision-free in the dense check
%   iter      STOMP iterations
%   time      planning time [s]
%   maxTilt   max angle between end-effector y-axis and world z [deg]
%             (only meaningful for the Task 5 "cup" scenarios)
%   minClr    minimum clearance [m] (-Inf if in collision)
%   theta     planned trajectories (cell)
function out = runTrials(robot, q0, qT, voxel_world, world, model, opts, seeds, eeName, clrIdx)

if nargin < 9 || isempty(eeName), eeName = 'tool0'; end
if nargin < 10, clrIdx = []; end
opts.verbose = false;
n = numel(seeds);
out.success = false(n, 1); out.iter = zeros(n, 1); out.time = zeros(n, 1);
out.maxTilt = zeros(n, 1); out.minClr = zeros(n, 1); out.theta = cell(n, 1);
out.stopReason = strings(n, 1);

for t = 1:n
    rng(seeds(t), 'twister');
    r = stompPlan(robot, q0, qT, voxel_world, model, opts);
    c = stompCheckCollision(robot, r.theta, world, [], clrIdx);
    out.success(t) = c.free;
    out.iter(t)    = r.iter;
    out.time(t)    = sum(r.iteration_time);
    out.minClr(t)  = c.minClearance;
    out.theta{t}   = r.theta;
    out.stopReason(t) = string(r.stopReason);
    out.maxTilt(t) = maxTiltDeg(robot, r.theta, eeName);
end
end

function m = maxTiltDeg(robot, theta, eeName)
nInterp = 4; m = 0;
for i = 1:size(theta, 2) - 1
    for j = 0:nInterp - 1
        a = j / nInterp;
        T = getTransform(robot, (1 - a) * theta(:, i) + a * theta(:, i + 1), eeName);
        m = max(m, acosd(max(-1, min(1, T(3, 2)))));
    end
end
T = getTransform(robot, theta(:, end), eeName);
m = max(m, acosd(max(-1, min(1, T(3, 2)))));
end
