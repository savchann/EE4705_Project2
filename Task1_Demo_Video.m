
%% EE4705 Project 2 - Task 1
% Kinova Gen3 STOMP Collision Avoidance Demonstration

clear;
clc;
close all;

%% 1. Load saved optimized trajectory
load('Theta_nDisc20_nPaths_20.mat', 'theta');

%% 2. Load Kinova Gen3 robot
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');

%% 3. Create original obstacle environment
helperCreateObstaclesKINOVA;

%% 4. Create visualization
fig = figure('Position', [200 150 1000 700]);

ax = axes('Parent', fig);

%% 5. Initialize video writer
videoName = 'KinovaGen3_Task1_Demo.mp4';

v = VideoWriter(videoName, 'MPEG-4');
v.FrameRate = 10;
open(v);

%% 6. Animate and record trajectory
nWaypoints = size(theta, 2);
framesPerWaypoint = 5;

for t = 1:nWaypoints

    % Display robot at current configuration
    show(robot, theta(:,t), ...
        'Parent', ax, ...
        'PreservePlot', false, ...
        'Frames', 'off');

    hold(ax, 'on');

    % Display obstacles
    for obs = 1:numel(world)
        show(world{obs}, 'Parent', ax);
    end

    % Set consistent camera view
    view(ax, 150, 29);
    axis(ax, [-0.8 0.8 -0.6 0.7 -0.2 0.7]);
    axis(ax, 'equal');

    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');

    title(ax, sprintf( ...
        'Kinova Gen3 - STOMP Collision Avoidance (%d/%d)', ...
        t, nWaypoints));

    drawnow;

    % Capture multiple frames for smoother playback
    frame = getframe(fig);

    for k = 1:framesPerWaypoint
        writeVideo(v, frame);
    end

    hold(ax, 'off');

end

%% 7. Close video writer
close(v);

fprintf('\nDemonstration video saved successfully!\n');
fprintf('Filename: %s\n', videoName);
fprintf('Location: %s\n', fullfile(pwd, videoName));
