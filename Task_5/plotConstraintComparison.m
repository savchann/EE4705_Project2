%% One picture of how the orientation constraint changes the trajectory
% Draws, in a single 3D view, the reference-run trajectory WITHOUT the
% constraint (w_c = 0, i.e. the Task 4 behaviour) and WITH it:
%   - obstacles (semi-transparent)
%   - the robot at start and goal (faint, shared by both runs)
%   - the robot at the waypoint where the unconstrained cup tilts most,
%     once for each run (red tint = no constraint, teal = with constraint),
%     so the change in arm posture is visible
%   - both end-effector paths (red = no constraint, blue = with constraint)
%   - the cup "up" axis (end-effector y-axis) at every waypoint for both
%     runs (orange = no constraint, green = with constraint)
%
% fig = plotConstraintComparison(robot, theta0, theta1, world, eeName, ...
%                                maxTilt0, maxTilt1, free, sc, wc)
function fig = plotConstraintComparison(robot, theta0, theta1, world, eeName, maxTilt0, maxTilt1, free, sc, wc)

fig = figure('Name', ['Task5 comparison ' sc.name], 'Color', 'w', 'Position', [100 100 950 720]);
ax = axes('Parent', fig); hold(ax, 'on');

% obstacles
for i = 1:numel(world)
    [~, p] = show(world{i}, 'Parent', ax);
    p.FaceColor = [0.85 0.45 0.2]; p.FaceAlpha = 0.30; p.EdgeColor = 'none';
    if isa(world{i}, 'collisionBox') && world{i}.X > 1.0
        p.FaceColor = [0.7 0.7 0.7]; p.FaceAlpha = 0.15;   % table
    end
end

% waypoint where the unconstrained cup tilts most
N = size(theta0, 2);
tw = zeros(1, N);
for i = 1:N
    T = getTransform(robot, theta0(:, i), eeName);
    tw(i) = acosd(max(-1, min(1, T(3, 2))));
end
[~, iMax] = max(tw);

drawRobot(ax, robot, theta1(:, 1),   [0.6 0.6 0.6], 0.18);
drawRobot(ax, robot, theta1(:, end), [0.6 0.6 0.6], 0.18);
drawRobot(ax, robot, theta0(:, iMax), [0.85 0.35 0.30], 0.45);
drawRobot(ax, robot, theta1(:, iMax), [0.15 0.55 0.60], 0.65);

% end-effector paths and cup axes
[P0, Y0] = eeData(robot, theta0, eeName);
[P1, Y1] = eeData(robot, theta1, eeName);
h0 = plot3(ax, P0.dense(:, 1), P0.dense(:, 2), P0.dense(:, 3), '-', 'Color', [0.85 0.2 0.2], 'LineWidth', 2.5);
h1 = plot3(ax, P1.dense(:, 1), P1.dense(:, 2), P1.dense(:, 3), '-', 'Color', [0.1 0.35 0.85], 'LineWidth', 2.5);
q0 = quiver3(ax, P0.wp(:, 1), P0.wp(:, 2), P0.wp(:, 3), Y0(:, 1), Y0(:, 2), Y0(:, 3), 0.10, ...
    'Color', [0.95 0.55 0.1], 'LineWidth', 1.4, 'MaxHeadSize', 0.4);
q1 = quiver3(ax, P1.wp(:, 1), P1.wp(:, 2), P1.wp(:, 3), Y1(:, 1), Y1(:, 2), Y1(:, 3), 0.10, ...
    'Color', [0 0.6 0], 'LineWidth', 1.4, 'MaxHeadSize', 0.4);
plot3(ax, P1.wp(1, 1), P1.wp(1, 2), P1.wp(1, 3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
plot3(ax, P1.wp(end, 1), P1.wp(end, 2), P1.wp(end, 3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 9);

status = {'collision-free', 'IN COLLISION'};
legend(ax, [h0 h1 q0 q1], ...
    {sprintf('no constraint (Task 4 behaviour): %s, max tilt %.0f%s', status{2 - free(1)}, maxTilt0, char(176)), ...
     sprintf('w_c = %g: %s, max tilt %.1f%s', wc, status{2 - free(2)}, maxTilt1, char(176)), ...
     'cup "up" axis, no constraint', sprintf('cup "up" axis, w_c = %g', wc)}, ...
    'Location', 'northoutside', 'Interpreter', 'none');
title(ax, sprintf('%s: trajectory without vs with the cup constraint (arm shown at waypoint %d)', sc.title, iMax), ...
    'Interpreter', 'none');

view(ax, sc.view); axis(ax, 'equal'); axis(ax, sc.axisLimits); grid(ax, 'on');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
camlight(ax, 'headlight'); lighting(ax, 'gouraud');
end

function drawRobot(ax, robot, q, color, alpha)
before = findobj(ax, 'Type', 'patch');
show(robot, q, 'Parent', ax, 'PreservePlot', true, 'Frames', 'off');
after = findobj(ax, 'Type', 'patch');
isNew = true(size(after));
for a = 1:numel(after)
    isNew(a) = ~any(after(a) == before);
end
set(after(isNew), 'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'none');
end

function [P, Y] = eeData(robot, theta, eeName)
N = size(theta, 2); nInterp = 5;
P.wp = zeros(N, 3); Y = zeros(N, 3);
for i = 1:N
    T = getTransform(robot, theta(:, i), eeName);
    P.wp(i, :) = T(1:3, 4)'; Y(i, :) = T(1:3, 2)';
end
P.dense = zeros((N - 1) * nInterp + 1, 3); k = 1;
for i = 1:N - 1
    for j = 0:nInterp - 1
        a = j / nInterp;
        T = getTransform(robot, (1 - a) * theta(:, i) + a * theta(:, i + 1), eeName);
        P.dense(k, :) = T(1:3, 4)'; k = k + 1;
    end
end
P.dense(end, :) = P.wp(end, :);
end
