%% Clearance of one configuration from the obstacles (robot meshes)
% c = endpointClearance(robot, q, world, clrIdx)
%   c      : smallest distance [m] between the robot meshes and the
%            obstacles world(clrIdx); -Inf if the robot touches ANY obstacle
%   clrIdx : obstacles to measure (e.g. all but the table under the base)
% Used to make sure start and goal are not grazing an obstacle: STOMP never
% moves them, so an endpoint in contact makes every run fail.
function c = endpointClearance(robot, q, world, clrIdx)

if nargin < 4 || isempty(clrIdx), clrIdx = 1:numel(world); end
[inColl, sepDist] = checkCollision(robot, q, world, "IgnoreSelfCollision", "on", "Exhaustive", "on");
if inColl
    c = -Inf;
else
    c = min(sepDist(:, clrIdx), [], 'all');
end
end
