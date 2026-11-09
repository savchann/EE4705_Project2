
function [X, T, Tee] = updateJointsWorldPosition(robot, theta, eeName)
% Task 3 - PoE Forward Kinematics for UR5e
%
% Replaces MATLAB getTransform() during trajectory evaluation.
%
% INPUT:
% robot  - UR5e rigidBodyTree
% theta  - 6x1 joint angles (radians)
% eeName - Optional end-effector name ('tool0')
%
% OUTPUT:
% X   - 6x4 moving joint/body positions
% T   - 1x6 cell containing moving body transformations
% Tee - End-effector transformation (optional)

%% Initialize PoE model once

persistent poeModel

if isempty(poeModel)
    poeModel = ur5ePoEModel(robot);
end

%% Validate input

theta = theta(:);

nJoints = poeModel.nJoints;

assert(numel(theta) == nJoints, ...
    'Expected %d joint angles, received %d.', ...
    nJoints, numel(theta));

%% Calculate link transformations using PoE

T = cell(1,nJoints);
X = zeros(nJoints,4);

for k = 1:nJoints

    % PoE transformation for moving link k
    T{k} = ur5eFKinSpace( ...
        poeModel.Mlinks{k}, ...
        poeModel.Slist(:,1:k), ...
        theta(1:k));

    % Homogeneous position of link frame
    X(k,:) = T{k}(:,4)';

end

%% Optional end-effector transformation

Tee = [];

if nargout > 2

    if nargin < 3 || ~strcmp(char(eeName),'tool0')
        error('Task 3 PoE currently supports tool0 only.');
    end

    Tee = ur5eFKinSpace( ...
        poeModel.Mee, ...
        poeModel.Slist, ...
        theta);

end

end
