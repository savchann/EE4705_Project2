%% Tilt of the "cup" along a trajectory
% tilt = angle (deg) between the end-effector y-axis and world up (+z),
% evaluated densely (nInterp linearly interpolated configurations between
% consecutive waypoints), so tilt between waypoints is also visible.
% 0 deg = upright, 90 deg = cup on its side.
%
% [tilt, s] = eeTiltDeg(robot, theta, eeName, nInterp)
%   s : path parameter in waypoint units (1 ... nDiscretize), for plotting
function [tilt, s] = eeTiltDeg(robot, theta, eeName, nInterp)

if nargin < 4, nInterp = 10; end
N = size(theta, 2);
nDense = (N - 1) * nInterp + 1;
tilt = zeros(nDense, 1);
s = zeros(nDense, 1);
k = 1;
for i = 1:N - 1
    for j = 0:nInterp - 1
        a = j / nInterp;
        tilt(k) = tiltOf(robot, (1 - a) * theta(:, i) + a * theta(:, i + 1), eeName);
        s(k) = i + a;
        k = k + 1;
    end
end
tilt(end) = tiltOf(robot, theta(:, end), eeName);
s(end) = N;
end

function t = tiltOf(robot, q, eeName)
Tq = getTransform(robot, q, eeName);
t = acosd(max(-1, min(1, Tq(3, 2))));   % y-axis . world z = R(3,2)
end
