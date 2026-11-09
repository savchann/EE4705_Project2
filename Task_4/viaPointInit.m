%% Initial trajectory through one via-point (extension to Task 1's straight line)
% STOMP is a local optimiser: it only searches near its initial trajectory.
% Task 1 always starts from a straight line in joint space. Here the
% initial guess is two straight joint-space segments, q0 -> qVia -> qT,
% with the via-point at the middle waypoint. The start and goal stay
% fixed; STOMP then optimises this trajectory exactly as before.
%
% theta = viaPointInit(q0, qVia, qT, nDiscretize)
function theta = viaPointInit(q0, qVia, qT, nDiscretize)

q0 = q0(:); qVia = qVia(:); qT = qT(:);
n1 = floor(nDiscretize / 2) + 1;           % waypoints in the first segment (incl. via)
n2 = nDiscretize - n1 + 1;                 % second segment (incl. via)
theta = zeros(numel(q0), nDiscretize);
for k = 1:numel(q0)
    a = linspace(q0(k), qVia(k), n1);
    b = linspace(qVia(k), qT(k), n2);
    theta(k, :) = [a, b(2:end)];
end
end
