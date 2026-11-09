%% *UR5e Manipulator Path Planning using STOMP (Task 2)*
% Same scenario type as Task 1 (one box between start and goal), with the
% Kinova Gen3 replaced by a Universal Robots UR5e.
%
% What had to change for the UR5e (see README):
%   - robot name and end-effector link ('tool0')
%   - start/goal poses scaled to the UR5e workspace, IK seeded on the
%     elbow-up branch (ur5eSolveIK)
%   - joint/body mapping: the UR5e tree contains fixed bodies, so joint k
%     is not BodyNames{k} (updateJointsWorldPosition)
%   - a new collision-sphere model that follows the real link geometry,
%     including the UR5e's sideways link offsets (ur5eSphereModel)
%   - obstacle moved so the goal itself is collision-free
% The STOMP algorithm (sampling, cost, update) is the one from Task 1.
%
% Run from anywhere: the script adds the Task 1 folder (unchanged STOMP
% functions) and this folder to the MATLAB path.

clear; clc; close all;

%% Paths
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir, 'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir, 'Task_1'); end
assert(isfolder(task1Dir), 'Cannot find the Task 1 folder next to %s', thisDir);
addpath(task1Dir);   % sEDT_3d, stompSamples, stompDTheta, stompUpdateProb, stompUpdateTheta, stompObstacleCost
addpath(thisDir);    % added last = searched first, so Task 2 versions shadow Task 1 versions
resultsDir = fullfile(thisDir, 'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

%% Reproducibility
rng(42, 'twister');

%% Robot Description and Poses
robot_name = 'universalUR5e';
robot = loadrobot(robot_name, 'DataFormat', 'column');
numJoints = numel(homeConfiguration(robot));   % 6 (fixed bodies are not counted)
endEffector = "tool0";

fprintf('Loaded %s: %d bodies, %d moving joints, base frame "%s"\n', ...
    robot_name, robot.NumBodies, numJoints, robot.BaseName);

% Initial end-effector pose: gripper pointing down (tool z = -world z)
R_down = [1 0 0; 0 -1 0; 0 0 -1];
taskInit = trvec2tform([0.45 -0.10 0.20]) * rotm2tform(R_down);
% Final end-effector pose: on the other side of the box, also pointing down
taskFinal = trvec2tform([0.40 0.55 0.30]) * rotm2tform(R_down);
anglesFinal = rotm2eul(taskFinal(1:3,1:3), 'XYZ');
poseFinal = [taskFinal(1:3,4); anglesFinal'];

% IK seeds on the elbow-up branch (found offline; the solver refines them)
seedInit  = [-0.512; -1.407; 2.210; -2.374; -1.571; -2.083];
seedFinal = [ 0.745; -1.081; 1.440; -1.930; -1.571; -0.826];

[currentRobotJConfig, infoInit] = ur5eSolveIK(robot, endEffector, taskInit, seedInit);
[finalRobotJConfig,  infoFinal] = ur5eSolveIK(robot, endEffector, taskFinal, seedFinal);
fprintf('IK start: pos err %.2e m, %.2f rad from seed\n', infoInit.positionError, infoInit.distFromSeed);
fprintf('IK goal : pos err %.2e m, %.2f rad from seed\n', infoFinal.positionError, infoFinal.distFromSeed);

%% Collision Meshes and Obstacles
helperCreateObstaclesUR5e;

%% Collision-sphere model (computed once)
model.spheres = ur5eSphereModel(robot);
fprintf('Sphere model: %d spheres\n', numel(model.spheres.radius));

%% Visualize the robot at the initial configuration with the obstacle
x0 = [currentRobotJConfig', zeros(1, numJoints)];
helperInitialVisualizerUR5e;

%% Path Planning using STOMP
helperSTOMP;

