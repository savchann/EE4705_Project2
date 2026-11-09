%% Inverse kinematics for a start/goal pose with an explicit seed
% The UR5e has up to 8 IK solutions for one pose (shoulder left/right,
% elbow up/down, wrist flip). MATLAB's solver returns whichever is closest
% to the initial guess, so starting it from robot.homeConfiguration (as in
% the Kinova code) can land on an odd branch, e.g. elbow-down, and then the
% straight-line initial path swings the arm through strange places.
% We therefore pass a seed close to the branch we want and check the result.
%
% INPUT:  robot ('column'), eeName, Tgoal (4x4), seed (6x1)
% OUTPUT: q (6x1, wrapped to [-pi, pi]), info (solver info + pose error)
function [q, info] = ur5eSolveIK(robot, eeName, Tgoal, seed)

% A new solver object on every call (only a few calls per script). Reusing
% one across calls breaks when the end-effector name switches between a
% string ("tool0") and a char array ('tool0'): MATLAB System objects do not
% allow input types to change after the first call.
eeName = char(eeName);
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;

weights = [1 1 1 1 1 1];
[q, info] = ik(eeName, Tgoal, weights, seed(:));
q = wrapToPi(q);

% Independent check of the achieved pose
Tq = getTransform(robot, q, eeName);
info.positionError = norm(Tq(1:3, 4) - Tgoal(1:3, 4));
info.angleError    = rad2deg(norm(rotm2axang(Tq(1:3, 1:3)' * Tgoal(1:3, 1:3)) .* [0 0 0 1]));
info.distFromSeed  = norm(wrapToPi(q - seed(:)));

if info.positionError > 1e-3 || info.angleError > 0.5
    warning('ur5eSolveIK: pose not reached (pos err %.4f m, ang err %.2f deg).', ...
        info.positionError, info.angleError);
end
end
