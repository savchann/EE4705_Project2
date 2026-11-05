
%% EE4705 Project 2 - Task 1
% Kinova Gen3 Collision Avoidance using STOMP
% Complete helperSTOMP.m
%
% Based on the lecturer-provided starter code.
% Includes:
%   1. Gaussian trajectory sampling
%   2. Trajectory cost evaluation
%   3. Probability-weighted trajectory updates
%   4. Trajectory smoothing
%   5. Collision verification
%   6. Optional video recording and animation
%   7. Saving optimization results

%% =========================================================
% 1. STOMP PARAMETERS
% ==========================================================

nDiscretize = 20;
nPaths = 20;
convergenceThreshold = 0.1;
maxIterations = 50;

%% =========================================================
% 2. INITIALIZE TRAJECTORY
% ==========================================================

% Initial and final robot joint configurations
q0 = currentRobotJConfig;
qT = finalRobotJConfig;

numJoints = length(q0);

% Initialize joint trajectory
theta = zeros(numJoints, nDiscretize);

% Linear interpolation between initial and final joint angles
for k = 1:numJoints
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% Store initial trajectory for comparison
theta_initial = theta;

% Load robot with structure data format
robot_struct = loadrobot(robot_name);

% Initialize sampled trajectories
theta_samples = cell(1, nPaths);

%% =========================================================
% 3. PRECOMPUTE SMOOTHNESS MATRICES
% ==========================================================

% Finite difference matrix for acceleration
A_k = eye(nDiscretize - 1);

A = -2 * eye(nDiscretize);

A(1:nDiscretize-1, 2:nDiscretize) = ...
    A(1:nDiscretize-1, 2:nDiscretize) + A_k;

A(2:nDiscretize, 1:nDiscretize-1) = ...
    A(2:nDiscretize, 1:nDiscretize-1) + A_k;

% Exclude fixed start and goal configurations
A = A(:, 2:end-1);

% Control cost matrix
R = A' * A;

% Inverse control cost matrix
Rinv = inv(R);

% Smoothing matrix
M = (1/nDiscretize) * Rinv ./ max(Rinv, [], 1);

% Normalize covariance for trajectory exploration
Rinv = 1.5 * Rinv / sum(Rinv, 'all');

%% =========================================================
% 4. INITIALIZE OPTIMIZATION
% ==========================================================

% Trajectory cost history
Q_time = [];
RAR_time = [];
iteration_time = [];

% Store intermediate trajectories
theta_animation = {};

% Calculate initial trajectory cost
[~, Qtheta] = stompTrajCost( ...
    robot_struct, theta, R, voxel_world);

Qtheta_initial = Qtheta;
QthetaOld = 0;

iter = 0;

fprintf('\n====================================\n');
fprintf('Starting STOMP Optimization\n');
fprintf('====================================\n');

fprintf('Number of joints: %d\n', numJoints);
fprintf('Number of waypoints: %d\n', nDiscretize);
fprintf('Sampled trajectories: %d\n', nPaths);
fprintf('Initial trajectory cost: %.4f\n', Qtheta);

%% =========================================================
% 5. STOMP OPTIMIZATION LOOP
% ==========================================================

while abs(Qtheta - QthetaOld) > convergenceThreshold

    iter = iter + 1;

    % Store previous trajectory cost
    QthetaOld = Qtheta;

    tic;

    %% Step 1: Sample noisy trajectories
    [theta_samples, em] = stompSamples( ...
        nPaths, Rinv, theta);

    %% Step 2: Calculate local trajectory costs
    Stheta = zeros(nPaths, nDiscretize);

    for k = 1:nPaths

        [Stheta(k,:), ~] = stompTrajCost( ...
            robot_struct, ...
            theta_samples{k}, ...
            R, ...
            voxel_world);

    end

    %% Step 3: Calculate trajectory probabilities
    trajProb = stompUpdateProb(Stheta);

    %% Step 4: Calculate probability-weighted update
    dtheta = stompDTheta(trajProb, em);

    %% Step 5: Smooth and update the trajectory
    [theta, dtheta_smoothed] = stompUpdateTheta( ...
        theta, dtheta, M);

    %% Step 6: Evaluate the updated trajectory
    [~, Qtheta] = stompTrajCost( ...
        robot_struct, theta, R, voxel_world);

    %% Record iteration runtime
    elapsedTime = toc;

    iteration_time = [iteration_time, elapsedTime];

    %% Record trajectory cost
    Q_time = [Q_time, Qtheta];

    % Calculate control/smoothness cost
    theta_interior = theta(:, 2:nDiscretize-1);

    RAR = 0.5 * sum( ...
        theta_interior * R * theta_interior', 'all');

    RAR_time = [RAR_time, RAR];

    %% Save intermediate trajectory
    theta_animation{iter} = theta;

    %% Display iteration results
    fprintf('\nIteration %d\n', iter);
    fprintf('Elapsed time: %.3f seconds\n', elapsedTime);
    fprintf('Total cost (Qtheta): %.4f\n', Qtheta);
    fprintf('Smoothness cost (RAR): %.4f\n', RAR);
    fprintf('Cost change: %.4f\n', abs(Qtheta-QthetaOld));

    %% Stopping condition 1: Maximum iterations
    if iter >= maxIterations

        disp('Maximum iteration limit reached.');
        break;

    end

    %% Stopping condition 2: Update approximately zero
    if norm(dtheta_smoothed, 'fro') < 1e-10

        disp('Trajectory update is approximately zero.');
        break;

    end

end

fprintf('\n====================================\n');
fprintf('STOMP Finished.\n');
fprintf('====================================\n');

fprintf('Total iterations: %d\n', iter);
fprintf('Initial cost: %.4f\n', Qtheta_initial);
fprintf('Final cost: %.4f\n', Qtheta);

fprintf('Cost reduction: %.2f%%\n', ...
    100 * (Qtheta_initial-Qtheta)/max(abs(Qtheta_initial),eps));

fprintf('Total optimization time: %.2f seconds\n', ...
    sum(iteration_time));

%% =========================================================
% 6. COLLISION CHECK AT ORIGINAL WAYPOINTS
% ==========================================================

inCollision = false(nDiscretize, 1);

worldCollisionPairIdx = cell(nDiscretize, 1);

for i = 1:nDiscretize

    [inCollision(i), sepDist] = checkCollision( ...
        robot, ...
        theta(:,i), ...
        world, ...
        "IgnoreSelfCollision", "on", ...
        "Exhaustive", "on");

    % Identify collision pairs
    [bodyIdx, worldCollisionObjIdx] = find(isnan(sepDist));

    worldCollisionPairIdx{i} = ...
        [bodyIdx, worldCollisionObjIdx];

end

isTrajectoryInCollision = any(inCollision);

fprintf('\nOriginal waypoint collision check:\n');

if isTrajectoryInCollision
    fprintf('Collision detected at one or more waypoints.\n');
else
    fprintf('PASS: No collisions at the original waypoints.\n');
end

fprintf('Colliding waypoints: %d / %d\n', ...
    sum(inCollision), nDiscretize);

%% =========================================================
% 7. ADDITIONAL DENSE COLLISION CHECK
% ==========================================================
% Checks interpolated configurations between waypoints.
% This is an additional verification step.

enableDenseCollisionCheck = 1;

isDenseTrajectoryInCollision = NaN;
denseCollision = [];

if enableDenseCollisionCheck

    nInterp = 10;

    nDense = (nDiscretize-1)*nInterp + 1;

    denseCollision = false(nDense, 1);

    idx = 1;

    for i = 1:nDiscretize-1

        for j = 0:nInterp-1

            alpha = j/nInterp;

            qInterp = (1-alpha)*theta(:,i) ...
                    + alpha*theta(:,i+1);

            denseCollision(idx) = checkCollision( ...
                robot, ...
                qInterp, ...
                world, ...
                "IgnoreSelfCollision", "on");

            idx = idx + 1;

        end
    end

    % Check final configuration
    denseCollision(end) = checkCollision( ...
        robot, ...
        theta(:,end), ...
        world, ...
        "IgnoreSelfCollision", "on");

    isDenseTrajectoryInCollision = any(denseCollision);

    fprintf('\nDense collision verification:\n');

    if isDenseTrajectoryInCollision
        fprintf('WARNING: Collision detected between waypoints.\n');
    else
        fprintf('PASS: No collisions in dense waypoint sampling.\n');
    end

    fprintf('Colliding configurations: %d / %d\n', ...
        sum(denseCollision), nDense);

end

%% =========================================================
% 8. OPTIONAL TRAINING VIDEO
% ==========================================================

enableVideoTraining = 0;

if enableVideoTraining == 1 && ~isempty(theta_animation)

    v = VideoWriter('KinovaGen3_Training.avi');
    v.FrameRate = 15;

    open(v);

    theta_animation_tmp = theta_animation;

    htext = text(-0.2, 0.6, 0.7, ...
        'Iteration = 0', ...
        'HorizontalAlignment', 'left', ...
        'FontSize', 14);

    nTraining = length(theta_animation_tmp);

    for k = 1:5:nTraining

        updatedText = ['Iteration = ', num2str(k)];

        set(htext, 'String', updatedText);

        theta_tmp = theta_animation_tmp{k};

        for t = 1:size(theta_tmp,2)

            show(robot, theta_tmp(:,t), ...
                'PreservePlot', false, ...
                'Frames', 'on');

            drawnow;

            frame = getframe(gcf);

            writeVideo(v, frame);

        end
    end

    close(v);

    fprintf('Training video saved.\n');

end

%% =========================================================
% 9. OPTIONAL FINAL TRAJECTORY VIDEO
% ==========================================================

enableVideo = 0;

if enableVideo == 1

    v = VideoWriter('KinovaGen3_Task1_Final.avi');

    v.FrameRate = 5;

    open(v);

    figure;

    for t = 1:size(theta,2)

        show(robot, theta(:,t), ...
            'PreservePlot', false, ...
            'Frames', 'off');

        view(150,29);
        axis([-0.8 0.8 -0.6 0.7 -0.2 0.7]);

        drawnow;

        frame = getframe(gcf);

        writeVideo(v, frame);

    end

    close(v);

    fprintf('Final trajectory video saved.\n');

end

%% =========================================================
% 10. DISPLAY FINAL TRAJECTORY ANIMATION
% ==========================================================

displayAnimation = 1;

if displayAnimation

    figure;

    for t = 1:size(theta,2)

        show(robot, theta(:,t), ...
            'PreservePlot', false, ...
            'Frames', 'off');

        view(150,29);

        axis([-0.8 0.8 -0.6 0.7 -0.2 0.7]);

        drawnow;

        pause(0.25);

    end

end

%% =========================================================
% 11. PLOT STOMP CONVERGENCE
% ==========================================================

figure;

plot(1:length(Q_time), Q_time, ...
    '-o', 'LineWidth', 1.5);

xlabel('Iteration');
ylabel('Total Trajectory Cost');
title('STOMP Trajectory Cost Convergence - Kinova Gen3');

grid on;

%% =========================================================
% 12. PLOT SMOOTHNESS COST
% ==========================================================

figure;

plot(1:length(RAR_time), RAR_time, ...
    '-o', 'LineWidth', 1.5);

xlabel('Iteration');
ylabel('Smoothness Cost (RAR)');
title('STOMP Smoothness Cost - Kinova Gen3');

grid on;

%% =========================================================
% 13. SAVE RESULTS
% ==========================================================

filename = ['Theta_nDisc', ...
    num2str(nDiscretize), ...
    '_nPaths_', ...
    num2str(nPaths), ...
    '.mat'];

save(filename, ...
    'theta', ...
    'theta_initial', ...
    'Q_time', ...
    'RAR_time', ...
    'iteration_time', ...
    'inCollision', ...
    'isTrajectoryInCollision', ...
    'denseCollision', ...
    'isDenseTrajectoryInCollision', ...
    'Qtheta_initial', ...
    'Qtheta');

fprintf('\nResults saved to: %s\n', filename);

fprintf('\nTask 1 execution completed.\n');
