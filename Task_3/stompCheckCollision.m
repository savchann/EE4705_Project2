%% Collision verification of a joint trajectory with MATLAB's mesh checker
% Same checks as the end of the Task 1 helperSTOMP.m, as a function:
%   1. at the planned waypoints
%   2. densely, at nInterp linearly interpolated configurations between
%      consecutive waypoints (catches collisions STOMP cannot see, because
%      it only evaluates cost at the waypoints)
% It uses the robot's collision MESHES (checkCollision), not the STOMP
% spheres, so it is an independent check of the planner's result.
%
% OUTPUT c struct:
%   wpCollision    (nDiscretize x 1) logical, collision at each waypoint
%   denseCollision (nDense x 1) logical
%   nWp, nDense    counts of colliding configurations
%   minClearance   smallest robot-obstacle distance along the dense path [m]
%                  (negative/NaN-free: set to -Inf if in collision)
%   free           true if no collision in the dense check
function c = stompCheckCollision(robot, theta, world, nInterp)

if nargin < 4, nInterp = 10; end
nDiscretize = size(theta, 2);

c.wpCollision = false(nDiscretize, 1);
for i = 1:nDiscretize
    c.wpCollision(i) = checkCollision(robot, theta(:, i), world, ...
        "IgnoreSelfCollision", "on");
end

nDense = (nDiscretize - 1) * nInterp + 1;
c.denseCollision = false(nDense, 1);
minClr = Inf;
idx = 1;
for i = 1:nDiscretize - 1
    for j = 0:nInterp - 1
        a = j / nInterp;
        q = (1 - a) * theta(:, i) + a * theta(:, i + 1);
        [c.denseCollision(idx), minClr] = checkOne(robot, q, world, minClr);
        idx = idx + 1;
    end
end
[c.denseCollision(end), minClr] = checkOne(robot, theta(:, end), world, minClr);

c.nWp = sum(c.wpCollision);
c.nDense = sum(c.denseCollision);
c.free = ~any(c.denseCollision);
c.minClearance = minClr;
end

function [inColl, minClr] = checkOne(robot, q, world, minClr)
[inColl, sepDist] = checkCollision(robot, q, world, ...
    "IgnoreSelfCollision", "on", "Exhaustive", "on");
if inColl
    minClr = -Inf;
else
    minClr = min(minClr, min(sepDist(:)));
end
end
