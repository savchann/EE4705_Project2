%% STOMP planning, verification and visualisation (Task 2, UR5e)
% Same steps as the Task 1 helperSTOMP.m. The optimisation loop itself now
% lives in stompPlan.m (identical algorithm and parameters) so Tasks 4 and 5
% can call it repeatedly. Expects in the workspace: robot, numJoints,
% currentRobotJConfig, finalRobotJConfig, voxel_world, world, model,
% resultsDir.

%% Parameters (same as Task 1)
opts.nDiscretize = 20;
opts.nPaths = 20;
opts.convergenceThreshold = 0.1;
opts.maxIterations = 50;

%% Is the initial (straight-line) path in collision?
nDiscretize = opts.nDiscretize;
theta_initial = zeros(numJoints, nDiscretize);
for k = 1:numJoints
    theta_initial(k, :) = linspace(currentRobotJConfig(k), finalRobotJConfig(k), nDiscretize);
end
cInit = stompCheckCollision(robot, theta_initial, world);
fprintf('\nInitial straight-line path: %d / %d waypoints in collision\n', cInit.nWp, nDiscretize);

%% Run STOMP
fprintf('\n==== STOMP on %s ====\n', robot_name);
res = stompPlan(robot, currentRobotJConfig, finalRobotJConfig, voxel_world, model, opts);
theta = res.theta;
fprintf('Stopped after %d iterations (%s), %.1f s in total\n', res.iter, res.stopReason, sum(res.iteration_time));
fprintf('Cost: %.2f -> %.2f\n', res.Qtheta_initial, res.Qtheta);

%% Collision check of the result (waypoints + dense interpolation)
cFinal = stompCheckCollision(robot, theta, world);
inCollision = cFinal.wpCollision;
isTrajectoryInCollision = any(inCollision);
fprintf('Waypoints in collision: %d / %d\n', cFinal.nWp, nDiscretize);
fprintf('Dense check: %d / %d configurations in collision, min clearance %.3f m\n', ...
    cFinal.nDense, numel(cFinal.denseCollision), cFinal.minClearance);

%% Animate the planned trajectory
displayAnimation = 1;
if displayAnimation
    figure(hgif);
    for t = 1:size(theta, 2)
        show(robot, theta(:, t), 'PreservePlot', false, 'Frames', 'off');
        drawnow;
        pause(0.25);
    end
end

%% Optional: record the planned trajectory to video
enableVideo = 0;
if enableVideo
    v = VideoWriter(fullfile(resultsDir, 'UR5e_Task2.mp4'), 'MPEG-4');
    v.FrameRate = 5;
    open(v);
    figure(hgif);
    for t = 1:size(theta, 2)
        show(robot, theta(:, t), 'PreservePlot', false, 'Frames', 'off');
        drawnow;
        writeVideo(v, getframe(hgif));
    end
    close(v);
end

%% Convergence plots
figure('Name', 'Task 2 convergence');
subplot(1, 2, 1); plot(res.Q_time, '-o', 'LineWidth', 1.5); grid on;
xlabel('Iteration'); ylabel('Total cost Q(\theta)'); title('UR5e: total cost');
subplot(1, 2, 2); plot(res.RAR_time, '-o', 'LineWidth', 1.5); grid on;
xlabel('Iteration'); ylabel('Smoothness cost'); title('UR5e: smoothness cost');

%% Save
save(fullfile(resultsDir, 'Task2_UR5e_result.mat'), 'theta', 'res', 'cInit', 'cFinal', ...
    'currentRobotJConfig', 'finalRobotJConfig');
fprintf('Saved results to %s\n', resultsDir);
