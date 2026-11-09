%% Collision-sphere model for the UR5e
% The Task 1 sphere code (stompRobotSphere for Kinova) places spheres on
% straight lines between consecutive JOINT ORIGINS. That works for the
% Kinova but not reliably for the UR5e: the UR5e links are offset sideways
% from the joint axes (the upper arm runs 0.138 m to the side), and in the
% ur_description URDF the joint frames sit on the axes, so a joint-to-joint
% chain misses the real upper arm by up to ~14 cm (see checkSphereModelUR5e).
%
% Instead we describe the physical arm as 7 capsule-like segments, written
% in the BASE frame at the zero configuration (all joints = 0, arm
% stretched out along +x). Coordinates come from the UR5e kinematic
% parameters (d1 = 0.1625, a2 = 0.425, a3 = 0.3922, d4 = 0.1333,
% d5 = 0.0997, d6 = 0.0996) and the visual offsets in the UR5e URDF
% (shoulder offset 0.138 m, elbow offset 0.007 m). Each segment is assigned
% to the moving link that carries it, converted ONCE into that link's local
% frame, and afterwards moved with the link: p_world = T_link(theta) * p_local.
% Because the points are defined at the zero pose in the base frame, this
% does not depend on how the URDF happens to place its joint frames.
%
% Radii are slightly larger than the real housings (conservative).
% Spheres are spaced at most one radius apart along each segment.
%
% OUTPUT model struct:
%   model.link    (N x 1) index of the moving link (1..6) carrying sphere i
%   model.pLocal  (N x 3) sphere centre in that link's frame
%   model.radius  (N x 1) sphere radius [m]

function model = ur5eSphereModel(robot)

%        link   start point (base frame, q = 0)   end point                  radius
segs = { 1, [0      0      0.1625], [0      0.138  0.1625], 0.065;   % shoulder housing
         2, [0      0.138  0.1625], [0.425  0.138  0.1625], 0.055;   % upper arm
         2, [0.425  0.138  0.1625], [0.425  0.007  0.1625], 0.050;   % elbow housing
         3, [0.425  0.007  0.1625], [0.8172 0.007  0.1625], 0.045;   % forearm
         4, [0.8172 0.007  0.1625], [0.8172 0.1333 0.1625], 0.045;   % wrist 1 housing
         5, [0.8172 0.1333 0.1625], [0.8172 0.1333 0.0628], 0.045;   % wrist 2 housing
         6, [0.8172 0.1333 0.0628], [0.8172 0.2329 0.0628], 0.040 }; % wrist 3 + flange

% Link poses at the zero configuration
q0 = zeros(6, 1);
if strcmp(robot.DataFormat, 'row'), q0 = q0'; end
[~, T0, Ttool] = updateJointsWorldPosition(robot, q0, 'tool0');

% Sanity check: the UR5e tool0 at q = 0 should be at (0.8172, 0.2329, 0.0628)
% in base_link. If not, the MATLAB model uses a different base frame and the
% segment table above (and the scenario coordinates) must be re-checked.
pExpected = [0.8172; 0.2329; 0.0628];
if norm(Ttool(1:3, 4) - pExpected) > 0.01
    warning(['ur5eSphereModel: tool0 at q=0 is at [%.3f %.3f %.3f], expected ' ...
        '[0.817 0.233 0.063]. The base frame differs from the one assumed. ' ...
        'Run checkSphereModelUR5e to inspect the sphere coverage.'], Ttool(1:3, 4));
end

link = []; pLocal = []; radius = [];
for s = 1:size(segs, 1)
    [k, a, b, r] = segs{s, :};
    n = ceil(norm(b - a) / r) + 1;                 % spacing <= one radius
    pts = [linspace(a(1), b(1), n); linspace(a(2), b(2), n); linspace(a(3), b(3), n)];
    Tinv = T0{k} \ eye(4);                          % base -> link k at q = 0
    loc = Tinv(1:3, 1:3) * pts + Tinv(1:3, 4);
    link   = [link;   k * ones(n, 1)];              %#ok<AGROW>
    pLocal = [pLocal; loc'];                        %#ok<AGROW>
    radius = [radius; r * ones(n, 1)];              %#ok<AGROW>
end

model.link   = link;
model.pLocal = pLocal;
model.radius = radius;

end
