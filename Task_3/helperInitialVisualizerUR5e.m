% Visualize the obstacles and the UR5e at the start configuration
% (UR5e version of helperInitialVisualizerKINOVA.m: larger axis limits for
% the UR5e's reach, start and goal end-effector positions marked.)
positions = x0(1:numJoints)';

hgif = figure('Name', 'UR5e start configuration', 'Position', [375 446 800 600]);
ax1 = show(robot, positions(:, 1), 'PreservePlot', false, 'Frames', 'off');
view(135, 25)
hold on
axis([-0.6 1.0 -0.8 0.8 -0.1 0.9]);

plot3(taskInit(1, 4), taskInit(2, 4), taskInit(3, 4), 'g.', 'MarkerSize', 25)
plot3(poseFinal(1), poseFinal(2), poseFinal(3), 'r.', 'MarkerSize', 25)

for i = 1:numel(world)
    [~, pObj] = show(world{i});
    pObj.LineStyle = 'none';
end
title('UR5e: start configuration (green = start, red = goal)')
