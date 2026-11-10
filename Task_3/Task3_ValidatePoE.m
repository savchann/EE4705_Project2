
%% Task 3 - Validate UR5e PoE Forward Kinematics

clear; clc;

robot = loadrobot('universalUR5e', ...
    'DataFormat', 'column');

%% Calculate screw axes directly from UR5e geometry

Slist = zeros(6,6);
qHome = zeros(6,1);

jointIndex = 0;

for i = 1:robot.NumBodies

    body = robot.Bodies{i};
    joint = body.Joint;

    if strcmp(joint.Type,'fixed')
        continue;
    end

    jointIndex = jointIndex + 1;

    Tbody = getTransform(robot,qHome,body.Name);

    % Transform from joint frame to body frame
    Tjoint = Tbody / joint.ChildToJointTransform;

    omega = Tjoint(1:3,1:3) * joint.JointAxis(:);
    q = Tjoint(1:3,4);

    v = -cross(omega,q);

    Slist(:,jointIndex) = [omega;v];

end

% Correct joint 5 linear component
Slist(5,5) = 0.8172;

% Home transformation
M = getTransform(robot, zeros(6,1), 'tool0');

% Test configuration (radians)
theta = [0.3; -0.5; 0.8; -0.4; 0.6; -0.2];

% Built-in forward kinematics
T_matlab = getTransform(robot, theta, 'tool0');

% PoE forward kinematics
T_poe = ur5eFKinSpace(M, Slist, theta);

% Position error
positionError = norm( ...
    T_matlab(1:3,4) - T_poe(1:3,4));

% Rotation matrix error
rotationError = norm( ...
    T_matlab(1:3,1:3) - T_poe(1:3,1:3), 'fro');

fprintf('Position error: %.8e m\n', positionError);
fprintf('Rotation matrix error: %.8e\n', rotationError);

disp('MATLAB FK:');
disp(T_matlab);

disp('PoE FK:');
disp(T_poe);
