%% Smallest sphere-to-obstacle clearance along a trajectory (STOMP's own view)
% Uses the same spheres and voxel signed distance field as the obstacle
% cost, so it is cheap (one FK per waypoint) and tells whether STOMP itself
% "believes" the path is collision-free. Used by the improved stopping rule
% in stompPlan.m. (The final verdict still comes from stompCheckCollision,
% which uses the robot meshes.)
%
% c = stompSphereClearance(robot, theta, voxel_world, model)
%   c > 0 : no sphere touches an occupied voxel at any waypoint
function c = stompSphereClearance(robot, theta, voxel_world, model)

c = Inf;
corner = voxel_world.Env_size(1, :);
vs = voxel_world.voxel_size;
ws = voxel_world.world_size;
for i = 1:size(theta, 2)
    [~, T] = updateJointsWorldPosition(robot, theta(:, i));
    [centers, radi] = stompRobotSphere(T, model.spheres);
    idx = floor((centers - corner) ./ vs) + 1;
    if any(idx(:) < 1) || any(idx(:, 1) > ws(1)) || any(idx(:, 2) > ws(2)) || any(idx(:, 3) > ws(3))
        c = -Inf; return;                       % outside the voxel world
    end
    d = voxel_world.sEDT(sub2ind(ws, idx(:, 1), idx(:, 2), idx(:, 3)));
    c = min(c, min(d(:) - radi(:)));
end
end
