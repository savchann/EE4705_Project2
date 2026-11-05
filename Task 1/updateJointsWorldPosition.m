
%% Forward Kinematics
% INPUT:
%   robot_struct: MATLAB rigidBodyTree robot structure
%   theta: Joint rotation angles
%
% OUTPUT:
%   X: Joint positions in the world/base frame
%      Each row contains [x, y, z, 1]
%   T: Homogeneous transformation matrices for each joint
%      relative to the robot base frame

function [X, T] = updateJointsWorldPosition(robot_struct, theta)

    % Ensure joint angles are a column vector
    theta = theta(:);

    % Obtain the robot's home configuration
    tConfiguration = robot_struct.homeConfiguration;

    % Identify the moving joints (exclude fixed joints)
    movingBodyIdx = [];

    for k = 1:numel(robot_struct.Bodies)

        jointType = robot_struct.Bodies{k}.Joint.Type;

        if ~strcmp(jointType, 'fixed')
            movingBodyIdx(end+1) = k;
        end
    end

    % Number of movable joints
    nJoints = numel(movingBodyIdx);

    % Verify the number of joint angles
    assert(numel(theta) == nJoints, ...
        'Number of joint angles does not match moving joints.');

    % Update the joint positions
    for k = 1:nJoints
        tConfiguration(k).JointPosition = theta(k);
    end

    % Initialize outputs
    T = cell(1, nJoints);
    X = zeros(nJoints, 4);

    % Calculate forward kinematics for each joint
    for k = 1:nJoints

        % Get the name of the body associated with joint k
        bodyName = robot_struct.BodyNames{movingBodyIdx(k)};

        % Calculate the homogeneous transformation
        % from joint k's frame to the robot base frame
        T{k} = getTransform(robot_struct, ...
            tConfiguration, bodyName);

        % Extract the joint position in homogeneous coordinates
        % Fourth column of T = [x; y; z; 1]
        X(k,:) = T{k}(:,4)';

    end

end
