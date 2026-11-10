
%% Task 3 - Inspect UR5e Joint Geometry

clear; clc;

robot = loadrobot('universalUR5e', ...
    'DataFormat', 'column');

qHome = homeConfiguration(robot);

fprintf('UR5e Joint Geometry\n');
fprintf('-------------------\n');

for i = 1:robot.NumBodies

    body = robot.Bodies{i};
    joint = body.Joint;

    if strcmp(joint.Type, 'fixed')
        continue;
    end

    % Transform of body frame at home configuration
    T = getTransform(robot, qHome, body.Name);

    % Joint axis in its local joint frame
    axisLocal = joint.JointAxis(:);

    % Account for fixed joint-to-child transformation
    Tjoint = T / joint.ChildToJointTransform;

    % Convert joint axis into base coordinates
    omega = Tjoint(1:3,1:3) * axisLocal;

    % Point on the joint axis in base coordinates
    q = Tjoint(1:3,4);

    % Revolute screw axis
    v = -cross(omega, q);
    S = [omega; v];

    fprintf('\nJoint: %s\n', joint.Name);
    fprintf('Omega: [% .5f % .5f % .5f]\n', omega);
    fprintf('Point: [% .5f % .5f % .5f]\n', q);
    fprintf('Screw: [% .5f % .5f % .5f % .5f % .5f % .5f]\n', S);

end
