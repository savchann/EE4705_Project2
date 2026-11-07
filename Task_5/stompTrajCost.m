% Given a trajectory, calculate its cost (Task 5 version: with end-effector
% orientation constraint). Overrides Task_2/stompTrajCost.m when Task_5 is
% first on the MATLAB path.
%
% Local cost at waypoint i:
%   S(i) = 1000 * q_o(i) + q_c(i)
%   q_o(i): obstacle cost (unchanged from Tasks 1/2)
%   q_c(i) = w_c * || y_ee(i) - z_world ||_1
% where y_ee(i) is the end-effector's y-axis in world coordinates (2nd
% column of the rotation part of the forward kinematics) and z_world =
% [0;0;1]. This is option 2 in the brief's technical tips: penalise the
% l1 norm of the difference between the "cup up" axis and world up.
%   - It is 0 only when the cup is exactly upright. For a tilt angle a
%     towards one horizontal axis it equals sin(a) + 1 - cos(a): about 0.19
%     at 10 deg, 0.63 at 30 deg, 2 at 90 deg (cup on its side). It is not a
%     perfect angle measure (it also depends on the tilt direction and
%     peaks around 135 deg), but it rises steadily over the range we care
%     about (0-90 deg), which is all the optimiser needs.
%   - Rotation of the cup about the vertical axis is NOT penalised, so the
%     planner keeps that freedom for obstacle avoidance.
%
% The weight w_c (model.qcWeight) sets the trade-off against the obstacle
% cost (weighted by 1000). STOMP normalises the costs of the sampled
% trajectories per time step before turning them into probabilities, so
% what matters is the relative size: with w_c = 10, tilting the cup by 30 deg
% costs ~6, about the same as a handful of collision spheres moving 1 cm
% inside an obstacle (each ~1-2 at typical waypoint speeds). The value 10
% was tuned in a simulation of these scenarios; Task5_CupUpright.m
% includes a sweep over w_c to show the trade-off.
%
% INPUT model fields: spheres, qcWeight, eeName, upAxis (1/2/3 = x/y/z of
% the end-effector, default 2), worldUp (default [0;0;1])
function [Stheta, Qtheta] = stompTrajCost(robot, theta, R, voxel_world, model)

[~, nDiscretize] = size(theta);
qo_cost = zeros(1, nDiscretize);
qc_cost = zeros(1, nDiscretize);

wc = 0;
if isfield(model, 'qcWeight'), wc = model.qcWeight; end
eeName = 'tool0';
if isfield(model, 'eeName'), eeName = model.eeName; end
upAxis = 2;
if isfield(model, 'upAxis'), upAxis = model.upAxis; end
worldUp = [0; 0; 1];
if isfield(model, 'worldUp'), worldUp = model.worldUp(:); end

for i = 1:nDiscretize
    if wc > 0
        [~, T, Tee] = updateJointsWorldPosition(robot, theta(:, i), eeName);
        % constraint cost: l1 distance between the EE axis and world up
        qc_cost(i) = wc * norm(Tee(1:3, upAxis) - worldUp, 1);
    else
        [~, T] = updateJointsWorldPosition(robot, theta(:, i));
    end

    [sphere_centers, radi] = stompRobotSphere(T, model.spheres);
    if i == 1
        vel = zeros(size(sphere_centers, 1), 1);
    else
        vel = vecnorm(sphere_centers_prev - sphere_centers, 2, 2);
    end
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
    sphere_centers_prev = sphere_centers;
end

Stheta = 1000 * qo_cost + qc_cost;

theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end
