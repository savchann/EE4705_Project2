%% Collision spheres around the robot (Task 2 version)
% Replaces the Kinova version, which spread spheres between joint origins.
% Here the sphere centres are fixed points on each link (see
% ur5eSphereModel), moved with that link's current pose.
% INPUT:
%   T           : 1 x nJoints cell of link poses from updateJointsWorldPosition
%   sphereModel : struct from ur5eSphereModel (fields link, pLocal, radius)
% OUTPUT:
%   centers : (N x 3) sphere centres in the base frame
%   radi    : (N x 1) sphere radii
function [centers, radi] = stompRobotSphere(T, sphereModel)

centers = zeros(size(sphereModel.pLocal));
for k = 1:numel(T)
    idx = (sphereModel.link == k);
    if any(idx)
        centers(idx, :) = (T{k}(1:3, 1:3) * sphereModel.pLocal(idx, :)' + T{k}(1:3, 4))';
    end
end
radi = sphereModel.radius;

end
