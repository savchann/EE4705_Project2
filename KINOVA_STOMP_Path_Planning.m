%% *KINOVA Gen3 Manipulator Path Planning using STOMP*
% This example shows how to plan a collision-free robot path from an initial 
% to a desired end-effector pose using STOMP (<https://ieeexplore.ieee.org/document/5980280 
% Stochastic trajectory optimization for motion planning>)[1].
%% Robot Description and Poses

%% Reproducibility
rng(42, 'twister');

% Load the KINOVA Gen3 rigid body tree (RBT) model.

robot_name = 'kinovaGen3';
robot = loadrobot(robot_name, 'DataFormat', 'column'); % Note there are different data formats for loading the robot (chosen in the 2nd argument)


%% 
% Get the number of joints.

numJoints = numel(homeConfiguration(robot));
%% 
% Specify the robot frame where the end-effector is attached.

endEffector = "EndEffector_Link"; 
%% 
% Specify initial and desired end-effector poses. Use inverse kinematics to 
% solve for the initial robot configuration given a desired pose.

% Initial end-effector pose
taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([0 1 0 pi]);

% Compute current robot joint configuration using inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);

% The IK solver respects joint limits, but for those joints with infinite
% range, they must be wrapped to a finite range on the interval [-pi, pi].
% Since the the other joints are already bounded within this range, it is
% sufficient to simply call wrapToPi on the entire robot configuration
% rather than only on the joints with infinite range.
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% Final (desired) end-effector pose
taskFinal = trvec2tform([0.35 0.55 0.35])*axang2tform([0 1 0 pi]);  
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
finalRobotJConfig = ik(endEffector, taskFinal, weights, currentRobotJConfig);
finalRobotJConfig = wrapToPi(finalRobotJConfig);
%% Collision Meshes and Obstacles
% MATLAB has its own collision object functions to check for and avoid collisions 
% (e.g., collisionSphere, collisionBox, etc). Here, we follow the STOMP paper 
% to set up the voxel world and using the signed Euclidean distance transform 
% (sEDT) function for this purpose, which is a common practice in robotics perception, 
% mapping, and collision avoidance. We shall still use the MATLAB built-in collision 
% object functions to visualize the object for the convenience. See more details 
% in the following helper functions.

helperCreateObstaclesKINOVA;
%% 
% Visualize the robot at the initial configuration. You should see the obstacles 
% in the environment as well.

x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;
%% 
% Specify a safety distance away from the obstacles.

safetyDistance = 0.01; 
%% Path Planning using STOMP
% The following helper function contain the setup and execution of the STOMP 
% algorithm for collision-avoidance planning.

helperSTOMP;
%% 
% [1] M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor and S. Schaal, "STOMP: 
% Stochastic trajectory optimization for motion planning," _2011 IEEE International 
% Conference on Robotics and Automation_, 2011, pp. 4569-4574, doi: 10.1109/ICRA.2011.5980280.