%% Forward kinematics (Task 2 version, robot-agnostic)
% INPUT:
%   robot  : rigidBodyTree. Either 'column' or 'struct' DataFormat.
%   theta  : joint angles of the MOVING joints (nJoints x 1)
%   eeName : (optional) name of the end-effector body, e.g. 'tool0'
% OUTPUT:
%   X   : joint positions in the base frame, one row per moving joint [x y z 1]
%   T   : 1 x nJoints cell, T{k} = pose of the k-th moving body in the base frame
%   Tee : pose of the end-effector body eeName (only computed if requested)
%
% Changes from the Task 1 version:
%   1. The list of moving (non-fixed) bodies is found ONCE and cached in a
%      persistent variable. Task 1 looped over robot.Bodies{k} on every call,
%      which the brief notes is slow (this function is called
%      nPaths x nDiscretize times per STOMP iteration).
%   2. Uses robot.BodyNames{k} (fast) instead of robot.Bodies{k}.Name (slow).
%   3. Accepts a 'column' robot, so the joint vector is passed straight to
%      getTransform without rebuilding a configuration struct every call.
%   4. Optional third output: the end-effector pose (needed for Task 5).
%
% The UR5e has fixed bodies (e.g. base_link_inertia, flange, tool0), so the
% k-th joint angle does NOT correspond to robot.BodyNames{k}. The cached
% movingNames list maps joint k -> the body it moves.

function [X, T, Tee] = updateJointsWorldPosition(robot, theta, eeName)

persistent cacheKey movingNames

key = [robot.BaseName, '|', strjoin(robot.BodyNames, ',')];
if isempty(cacheKey) || ~strcmp(cacheKey, key)
    movingNames = {};
    for k = 1:robot.NumBodies
        if ~strcmp(robot.Bodies{k}.Joint.Type, 'fixed')
            movingNames{end+1} = robot.BodyNames{k}; %#ok<AGROW>
        end
    end
    cacheKey = key;
end

theta = theta(:);
nJoints = numel(movingNames);
assert(numel(theta) == nJoints, ...
    'updateJointsWorldPosition: %d joint angles given, robot has %d moving joints.', ...
    numel(theta), nJoints);

% Configuration in the format getTransform expects
if strcmp(robot.DataFormat, 'struct')
    config = robot.homeConfiguration;
    for k = 1:nJoints
        config(k).JointPosition = theta(k);
    end
elseif strcmp(robot.DataFormat, 'row')
    config = theta';
else
    config = theta;
end

T = cell(1, nJoints);
X = zeros(nJoints, 4);
for k = 1:nJoints
    T{k} = getTransform(robot, config, movingNames{k});
    X(k, :) = T{k}(:, 4)';
end

if nargout > 2
    Tee = getTransform(robot, config, eeName);
end

end
