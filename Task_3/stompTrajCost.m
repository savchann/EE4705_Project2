% Given a trajectory, calculate its cost (Task 2 version)
% Same cost as Task 1: Stheta = 1000*obstacle cost, Qtheta = sum(Stheta) +
% smoothness. Only the robot-specific parts changed:
%   - spheres come from the UR5e sphere model (model.spheres)
%   - the robot is passed in 'column' format for faster FK
% No end-effector constraint here: qc_cost stays 0 (Task 5 overrides this
% file with its own stompTrajCost.m).
%
% INPUT:
%   robot       : rigidBodyTree ('column' DataFormat)
%   theta       : nJoints x nDiscretize trajectory
%   R           : smoothness matrix (interior waypoints)
%   voxel_world : voxel world struct with signed distance field
%   model       : struct with field .spheres (from ur5eSphereModel)
function [Stheta, Qtheta] = stompTrajCost(robot, theta, R, voxel_world, model)

[~, nDiscretize] = size(theta);
qo_cost = zeros(1, nDiscretize);   % obstacle cost
qc_cost = zeros(1, nDiscretize);   % constraint cost (unused in Task 2)

[~, T] = updateJointsWorldPosition(robot, theta(:, 1));
[sphere_centers, radi] = stompRobotSphere(T, model.spheres);
vel = zeros(size(sphere_centers, 1), 1);   % zero velocity at the start
qo_cost(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

for i = 2:nDiscretize
    sphere_centers_prev = sphere_centers;
    [~, T] = updateJointsWorldPosition(robot, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(T, model.spheres);
    % workspace speed of each sphere (finite difference between waypoints)
    vel = vecnorm(sphere_centers_prev - sphere_centers, 2, 2);
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
end

Stheta = 1000 * qo_cost + qc_cost;

theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");

end
