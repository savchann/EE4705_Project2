%% Overlay of intermediate robot configurations along a planned path
% Draws, in one figure:
%   - the obstacles (semi-transparent)
%   - the robot at every 'step'-th waypoint, faded from light (start) to
%     dark (goal), so the whole motion is visible in a single image
%   - the end-effector trace of the planned path (solid blue)
%   - the end-effector trace of the straight-line initial guess (dashed red)
%   - start (green) and goal (red) markers
%   - optionally, the end-effector y-axis at every waypoint (Task 5: shows
%     whether the "cup" stays upright)
%
% fig = plotTrajectoryOverlay(robot, theta, theta_init, world, eeName, titleStr, opts)
% opts fields (all optional): step (default 3), view, axisLimits,
%   showEEAxis (false), axisColor, figName
function fig = plotTrajectoryOverlay(robot, theta, theta_init, world, eeName, titleStr, opts)

if nargin < 7, opts = struct(); end
step       = getOpt(opts, 'step', 3);
viewAngles = getOpt(opts, 'view', [135 25]);
lims       = getOpt(opts, 'axisLimits', [-0.5 1.0 -0.8 0.8 -0.1 1.0]);
showEEAxis = getOpt(opts, 'showEEAxis', false);
figName    = getOpt(opts, 'figName', titleStr);

fig = figure('Name', figName, 'Color', 'w', 'Position', [100 100 900 700]);
ax = axes('Parent', fig);
hold(ax, 'on');

% Obstacles (skip the large table slab for clarity, draw it flat instead)
for i = 1:numel(world)
    [~, p] = show(world{i}, 'Parent', ax);
    p.FaceColor = [0.85 0.45 0.2];
    p.FaceAlpha = 0.35;
    p.EdgeColor = 'none';
    if isa(world{i}, 'collisionBox') && world{i}.X > 1.0
        p.FaceColor = [0.7 0.7 0.7];      % table
        p.FaceAlpha = 0.15;
    end
end

% Robot at selected waypoints, light -> dark
N = size(theta, 2);
idx = unique([1:step:N, N]);
for j = 1:numel(idx)
    before = findobj(ax, 'Type', 'patch');
    show(robot, theta(:, idx(j)), 'Parent', ax, 'PreservePlot', true, ...
        'Frames', 'off');
    after = findobj(ax, 'Type', 'patch');
    isNew = true(size(after));
    for a = 1:numel(after)
        isNew(a) = ~any(after(a) == before);
    end
    newP = after(isNew);
    shade = 0.85 - 0.6 * (j - 1) / max(numel(idx) - 1, 1);
    set(newP, 'FaceColor', shade * [1 1 1], 'FaceAlpha', 0.35 + 0.5 * (j == numel(idx) || j == 1), ...
        'EdgeColor', 'none');
end

% End-effector traces (dense, linear interpolation between waypoints)
pPlan = eeTrace(robot, theta, eeName);
pInit = eeTrace(robot, theta_init, eeName);
h1 = plot3(ax, pPlan(:, 1), pPlan(:, 2), pPlan(:, 3), 'b-', 'LineWidth', 2.5);
h2 = plot3(ax, pInit(:, 1), pInit(:, 2), pInit(:, 3), 'r--', 'LineWidth', 1.5);
plot3(ax, pPlan(1, 1), pPlan(1, 2), pPlan(1, 3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
plot3(ax, pPlan(end, 1), pPlan(end, 2), pPlan(end, 3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 9);
legendItems = [h1 h2];
legendText = {'planned EE path', 'initial straight-line EE path'};

% End-effector y-axis ("cup up" direction) at every waypoint
if showEEAxis
    P = zeros(N, 3); Y = zeros(N, 3);
    for i = 1:N
        Tq = getTransform(robot, theta(:, i), eeName);
        P(i, :) = Tq(1:3, 4)'; Y(i, :) = Tq(1:3, 2)';
    end
    h3 = quiver3(ax, P(:, 1), P(:, 2), P(:, 3), Y(:, 1), Y(:, 2), Y(:, 3), 0.12, ...
        'Color', getOpt(opts, 'axisColor', [0 0.6 0]), 'LineWidth', 1.5, 'MaxHeadSize', 0.4);
    legendItems = [legendItems, h3];
    legendText{end+1} = 'end-effector y-axis (cup "up")';
end

axis(ax, lims); axis(ax, 'equal'); axis(ax, lims);
view(ax, viewAngles); grid(ax, 'on');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
title(ax, titleStr, 'Interpreter', 'none');
legend(ax, legendItems, legendText, 'Location', 'northoutside');
camlight(ax, 'headlight'); lighting(ax, 'gouraud');
end

function P = eeTrace(robot, theta, eeName)
nInterp = 5;
N = size(theta, 2);
P = zeros((N - 1) * nInterp + 1, 3);
k = 1;
for i = 1:N - 1
    for j = 0:nInterp - 1
        a = j / nInterp;
        Tq = getTransform(robot, (1 - a) * theta(:, i) + a * theta(:, i + 1), eeName);
        P(k, :) = Tq(1:3, 4)';
        k = k + 1;
    end
end
Tq = getTransform(robot, theta(:, end), eeName);
P(end, :) = Tq(1:3, 4)';
end

function v = getOpt(s, name, default)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
