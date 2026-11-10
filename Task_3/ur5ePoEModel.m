
function poe = ur5ePoEModel(robot)
% UR5e Product of Exponentials model
%
% Calculates:
%   Slist       - 6 x 6 space screw-axis matrix
%   Mlinks      - Home transformations of moving links
%   Mee         - Home transformation of tool0
%   movingNames - Moving body names
%
% Screw axes and home transforms are calculated once.

%% 1. Home configuration

qHome = zeros(6,1);

nBodies = robot.NumBodies;

Slist = zeros(6,6);
Mlinks = cell(1,6);
movingNames = cell(1,6);

jointIndex = 0;

%% 2. Extract screw axes and link home transforms

for i = 1:nBodies

    body = robot.Bodies{i};
    joint = body.Joint;

    % Skip fixed joints
    if strcmp(joint.Type,'fixed')
        continue;
    end

    jointIndex = jointIndex + 1;

    % Moving body name
    movingNames{jointIndex} = body.Name;

    % Body transformation at home configuration
    Tbody = getTransform(robot,qHome,body.Name);

    % Save link home transformation
    Mlinks{jointIndex} = Tbody;

    % Transformation of joint frame at home
    Tjoint = Tbody / joint.ChildToJointTransform;

    % Joint rotation axis in base frame
    omega = Tjoint(1:3,1:3) * joint.JointAxis(:);

    % Point on joint axis
    qAxis = Tjoint(1:3,4);

    % Linear component
    v = -cross(omega,qAxis);

    % Store screw axis
    Slist(:,jointIndex) = [omega;v];

end

%% 3. End-effector home transformation

Mee = getTransform(robot,qHome,'tool0');

%% 4. Store PoE model

poe.Slist = Slist;
poe.Mlinks = Mlinks;
poe.Mee = Mee;
poe.movingNames = movingNames;
poe.nJoints = jointIndex;

fprintf('PoE model initialized: %d screw axes\n',jointIndex);

end
