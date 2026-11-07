%% Visual check of the UR5e collision-sphere model (Task 2)
% Draws the robot meshes with the STOMP collision spheres on top, at a few
% configurations. Use it to confirm the spheres cover the arm (a good
% figure for the Task 2 slides), and to explain why the Kinova-style
% joint-to-joint spheres would not work on the UR5e: the red line shows
% the joint-origin chain the Task 1 code would have used.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
addpath(task1Dir); addpath(thisDir);

robot = loadrobot('universalUR5e', 'DataFormat', 'column');
sphereModel = ur5eSphereModel(robot);

configs = {zeros(6, 1), ...
           [0; -pi/2; pi/2; -pi/2; -pi/2; 0], ...
           [0.6; -1.0; 1.8; -2.3; -1.57; 0.4]};
names = {'zero (arm stretched out)', 'typical working pose', 'reaching forward'};

[xs, ys, zs] = sphere(12);
for c = 1:numel(configs)
    q = configs{c};
    figure('Name', ['Sphere model: ' names{c}]);
    show(robot, q, 'Frames', 'off', 'PreservePlot', false);
    hold on;
    [X, T] = updateJointsWorldPosition(robot, q);
    [centers, radi] = stompRobotSphere(T, sphereModel);
    for s = 1:numel(radi)
        surf(centers(s, 1) + radi(s) * xs, centers(s, 2) + radi(s) * ys, ...
             centers(s, 3) + radi(s) * zs, 'FaceColor', [0.2 0.5 1], ...
             'FaceAlpha', 0.25, 'EdgeColor', 'none');
    end
    % joint-origin chain (what the Task 1 sphere code would cover)
    plot3([0; X(:, 1)], [0; X(:, 2)], [0; X(:, 3)], 'r-o', 'LineWidth', 2);
    axis equal; view(135, 25); grid on;
    title({['UR5e sphere model: ' names{c}], ...
           sprintf('%d spheres (blue); red = joint-origin chain used by the Kinova code', numel(radi))});
end
