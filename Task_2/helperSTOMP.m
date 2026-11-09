
%% STOMP planning, verification and visualisation (Task 2, UR5e)
% Uses stompPlan.m for trajectory optimization.
% Expects variables initialized by UR5e_STOMP_Path_Planning.m.

%% Parameters
opts.nDiscretize = 20;
opts.nPaths = 20;
opts.convergenceThreshold = 0.1;
opts.maxIterations = 50;
opts.stopRule = 'original';   % Task 2 reproduces Task 1 exactly; Tasks 4/5 use the improved rule

%% Check initial trajectory
nDiscretize = opts.nDiscretize;

theta_initial = zeros(numJoints, nDiscretize);

for k = 1:numJoints
    theta_initial(k,:) = linspace( ...
        currentRobotJConfig(k), ...
        finalRobotJConfig(k), nDiscretize);
end

cInit = stompCheckCollision(robot, theta_initial, world);

fprintf('\nInitial straight-line path: %d / %d waypoints in collision\n', ...
    cInit.nWp, nDiscretize);

%% Run STOMP
fprintf('\n==== STOMP on %s ====\n', robot_name);

res = stompPlan(robot, currentRobotJConfig, ...
    finalRobotJConfig, voxel_world, model, opts);

theta = res.theta;

fprintf('Stopped after %d iterations (%s), %.1f s in total\n', ...
    res.iter, res.stopReason, sum(res.iteration_time));

fprintf('Total cost: %.3f -> %.3f\n', ...
    res.Qtheta_initial, res.Qtheta);

fprintf('Obstacle cost: %.3f -> %.3f\n', ...
    res.obstacle_initial, res.obstacle_final);

fprintf('Smoothness cost: %.3f -> %.3f\n', ...
    res.smoothness_initial, res.smoothness_final);

%% Collision verification
cFinal = stompCheckCollision(robot, theta, world);

inCollision = cFinal.wpCollision;
isTrajectoryInCollision = any(inCollision);

fprintf('Waypoints in collision: %d / %d\n', ...
    cFinal.nWp, nDiscretize);

fprintf(['Dense check: %d / %d configurations in collision, ' ...
    'min clearance %.3f m\n'], ...
    cFinal.nDense, numel(cFinal.denseCollision), ...
    cFinal.minClearance);

%% Animate planned trajectory
displayAnimation = 1;

if displayAnimation
    figure(hgif);

    for t = 1:size(theta,2)
        show(robot, theta(:,t), ...
            'PreservePlot',false, ...
            'Frames','off');

        drawnow;
        pause(0.25);
    end
end

%% Optional video recording
enableVideo = 0;

if enableVideo
    v = VideoWriter( ...
        fullfile(resultsDir,'UR5e_Task2.mp4'), ...
        'MPEG-4');

    v.FrameRate = 5;
    open(v);

    figure(hgif);

    for t = 1:size(theta,2)
        show(robot, theta(:,t), ...
            'PreservePlot',false, ...
            'Frames','off');

        drawnow;
        writeVideo(v,getframe(hgif));
    end

    close(v);
end

%% Convergence plots: Total, Smoothness and Obstacle Costs

figCost = figure( ...
    'Name','Task 2 STOMP Cost Convergence', ...
    'Color','w', ...
    'Position',[100 100 1400 450]);

iterations = 1:res.iter;

% Total cost
subplot(1,3,1);

plot(iterations,res.Q_time,'-o', ...
    'LineWidth',1.8,'MarkerSize',5);

grid on;
xlabel('Iteration');
ylabel('Total Cost');
title('UR5e: Total Cost');

% Smoothness cost
subplot(1,3,2);

plot(iterations,res.RAR_time,'-o', ...
    'LineWidth',1.8,'MarkerSize',5);

grid on;
xlabel('Iteration');
ylabel('Smoothness Cost');
title('UR5e: Smoothness Cost');

% Obstacle cost
subplot(1,3,3);

plot(iterations,res.obstacle_time,'-o', ...
    'LineWidth',1.8,'MarkerSize',5);

grid on;
xlabel('Iteration');
ylabel('Obstacle Cost');
title('UR5e: Obstacle Cost');

%% Figure formatting

ax = findall(figCost,'Type','axes');

for k = 1:numel(ax)
    ax(k).Color = 'w';
    ax(k).XColor = 'k';
    ax(k).YColor = 'k';
    ax(k).Title.Color = 'k';
    ax(k).XLabel.Color = 'k';
    ax(k).YLabel.Color = 'k';
    ax(k).GridColor = [0.7 0.7 0.7];
    ax(k).GridAlpha = 0.2;
    ax(k).LineWidth = 1;
    ax(k).FontSize = 11;
end

set(findall(figCost,'Type','text'),'Color','k');

%% Save convergence graphs

if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

exportgraphics(figCost, ...
    fullfile(resultsDir,'Task2_Cost_Convergence.png'), ...
    'Resolution',300);

savefig(figCost, ...
    fullfile(resultsDir,'Task2_Cost_Convergence.fig'));

%% Save simulation results

save(fullfile(resultsDir,'Task2_UR5e_result.mat'), ...
    'theta','res','cInit','cFinal', ...
    'currentRobotJConfig','finalRobotJConfig');

fprintf('\nSaved results and graphs to %s\n',resultsDir);


%% Combined STOMP Cost Convergence - Slide 2

% Include initial trajectory at iteration 0
iterations = 0:res.iter;

totalCost = [res.Qtheta_initial, res.Q_time];
obstacleCost = [res.obstacle_initial, res.obstacle_time];
smoothnessCost = [res.smoothness_initial, res.RAR_time];

% Create figure
figCombined = figure( ...
    'Name', 'UR5e STOMP Cost Convergence', ...
    'Color', 'w', ...
    'Position', [100 100 1000 550]);

% Plot total cost
plot(iterations, totalCost, '-o', ...
    'Color', [0.02 0.29 0.13], ...
    'LineWidth', 2.2, ...
    'MarkerSize', 5);
hold on;

% Plot obstacle cost
plot(iterations, obstacleCost, '-s', ...
    'Color', [0.85 0.33 0.10], ...
    'LineWidth', 2.2, ...
    'MarkerSize', 5);

% Plot smoothness cost
plot(iterations, smoothnessCost, '-^', ...
    'Color', [0.20 0.45 0.80], ...
    'LineWidth', 2.2, ...
    'MarkerSize', 5);

% Axis labels
xlabel('STOMP Iteration', 'Color', 'k');
ylabel('Trajectory Cost', 'Color', 'k');
title('UR5e STOMP Cost Convergence', ...
    'Color', 'k', 'FontWeight', 'bold');

legend({'Total Cost', 'Obstacle Cost', 'Smoothness Cost'}, ...
    'Location', 'northeast', ...
    'TextColor', 'k', ...
    'Color', 'w');

% Formatting
grid on;
xlim([0 res.iter]);
xticks(0:res.iter);

ax = gca;
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';
ax.GridColor = [0.7 0.7 0.7];
ax.GridAlpha = 0.2;
ax.FontSize = 12;
ax.LineWidth = 1;

set(findall(figCombined, 'Type', 'text'), 'Color', 'k');

% Save to Results
exportgraphics(figCombined, ...
    fullfile(resultsDir, 'Task2_Combined_Cost.png'), ...
    'Resolution', 300);

savefig(figCombined, ...
    fullfile(resultsDir, 'Task2_Combined_Cost.fig'));

fprintf('Combined cost graph saved to Results.\n');
