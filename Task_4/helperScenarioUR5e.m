%% Task 4 collision-avoidance scenarios for the UR5e
% sc = helperScenarioUR5e(name)            name: 'pillar' | 'clutter' | 'window' | 'shelf'
% sc = helperScenarioUR5e(name, params)    optional parameters (see below)
%
% Every scenario is designed so that the straight-line joint-space
% initialisation (STOMP's starting guess) passes through an obstacle,
% i.e. a non-trivial initial path, as the brief requires.
% The robot base stands on a table (top at z = -0.01, 1 cm below the base so
% the resting base is not reported as a collision).
%
% Returned struct:
%   name, title, description, shapes (for buildVoxelWorld),
%   taskInit, taskFinal (4x4 end-effector poses of tool0),
%   seedInit, seedFinal (IK seeds selecting the intended arm branch),
%   view ([az el]), axisLimits
%
% End-effector orientations used:
%   Rdown(yaw) : tool z pointing down (top-down grasp), rotated by yaw about z
%   Rcup(dir)  : tool z horizontal along direction dir (deg), tool y pointing
%                straight up. This is the "holding a cup" orientation used
%                again in Task 5.
function sc = helperScenarioUR5e(name, params)

if nargin < 2, params = struct(); end
table = shp('box', [0 0 -0.04], [1.6 1.6 0.06], 'table');
sc.name = name;
sc.view = [135 25];
sc.axisLimits = [-0.5 1.0 -0.8 0.8 -0.1 1.0];

switch lower(name)

    case 'pillar'
        % A tall pillar directly in front of the robot. Start and goal are
        % on opposite sides; the straight-line path swings the arm through
        % the pillar. Getting around needs a large change of arm shape
        % (pull the arm in, pass between base and pillar, reach out again),
        % so the result depends strongly on the random samples.
        % params.pillarX (default 0.55 m): distance of the pillar from the
        % base. Closer = less room to pass between base and pillar = harder.
        px = 0.55;
        if isfield(params, 'pillarX'), px = params.pillarX; end
        sc.title = 'Pillar: swing around a tall pillar';
        sc.description = sprintf(['Tall 10x10x90 cm pillar %.2f m in front of the base; ' ...
            'move from one side to the other with the gripper pointing down.'], px);
        sc.shapes = [table, shp('box', [px 0 0.45], [0.10 0.10 0.90], 'pillar')];
        sc.taskInit  = trvec2tform([0.45 -0.45 0.35]) * rotm2tform(Rdown(-pi/4));
        sc.taskFinal = trvec2tform([0.45  0.45 0.35]) * rotm2tform(Rdown( pi/4));
        sc.seedInit  = [-0.996; -1.218; 1.508; -1.860; -1.571; -1.782];
        sc.seedFinal = [ 0.574; -1.218; 1.508; -1.860; -1.571; -1.782];

    case 'clutter'
        % Pick-and-place across a cluttered table: a divider in the middle,
        % a ball, a bottle, a crate and a can around the start and goal,
        % and an overhead bar above the divider that blocks the simple
        % "lift high over everything" route. Many small obstacles at once.
        % v2: divider narrowed (30 -> 20 cm) and ball moved out, because in
        % v1 the goal pose cleared the divider by only 8 mm and the start
        % the ball by 28 mm: the robot meshes touched the divider at the
        % goal, so no run could succeed. Now both endpoints clear all
        % obstacles by > 5 cm (sphere model).
        sc.title = 'Cluttered tabletop';
        sc.description = ['Divider, ball, bottle, crate, can and an overhead bar; ' ...
            'top-down pick on one side, place on the other.'];
        sc.shapes = [table, ...
            shp('box',      [0.45  0.00 0.175], [0.10 0.20 0.35], 'divider'), ...
            shp('sphere',   [0.15 -0.42 0.07],  0.07,             'ball'), ...
            shp('cylinder', [0.17  0.36 0.12],  [0.04 0.24],      'bottle'), ...
            shp('box',      [0.62 -0.20 0.07],  [0.14 0.14 0.14], 'crate'), ...
            shp('cylinder', [0.62  0.20 0.10],  [0.05 0.20],      'can'), ...
            shp('box',      [0.45  0.00 0.62],  [0.10 0.60 0.04], 'overhead bar')];
        sc.taskInit  = trvec2tform([0.40 -0.35 0.10]) * rotm2tform(Rdown(-pi/2));
        sc.taskFinal = trvec2tform([0.40  0.35 0.10]) * rotm2tform(Rdown( pi/2));
        sc.seedInit  = [-0.972; -1.059; 2.075; -2.587; -1.571; -0.972];
        sc.seedFinal = [ 0.465; -1.059; 2.075; -2.587; -1.571; -2.676];

    case 'window'
        % A wall with a square window. The goal is on the far side, so the
        % forearm and wrist must thread through the opening: a narrow
        % passage, a known weak spot of sampling-based optimisers.
        % params.windowSize (default 0.36 m). Below ~0.34 m the goal pose
        % itself (arm resting in the opening) comes within 3 cm of the frame.
        W = 0.36;
        if isfield(params, 'windowSize'), W = params.windowSize; end
        xw = 0.50; t = 0.04; zc = 0.35; ytop = 0.6; ztop = 0.9;
        sc.title = sprintf('Window: reach through a %.0f cm window', 100 * W);
        sc.description = sprintf(['Wall at x = %.2f m with a %.0fx%.0f cm window centred ' ...
            'at z = %.2f m; reach from the near side to a point behind the wall.'], xw, 100*W, 100*W, zc);
        sc.shapes = [table, ...
            shp('box', [xw, -(ytop + W/2)/2, ztop/2], [t, ytop - W/2, ztop], 'wall left'), ...
            shp('box', [xw,  (ytop + W/2)/2, ztop/2], [t, ytop - W/2, ztop], 'wall right'), ...
            shp('box', [xw, 0, (zc - W/2)/2],         [t, W, zc - W/2],      'wall below'), ...
            shp('box', [xw, 0, (ztop + zc + W/2)/2],  [t, W, ztop - zc - W/2], 'wall above')];
        sc.taskInit  = trvec2tform([0.30 -0.35 0.45]) * rotm2tform(Rcup(0));
        sc.taskFinal = trvec2tform([0.72  0.05 0.30]) * rotm2tform(Rcup(0));
        sc.seedInit  = [-1.388; -1.594; 1.690; -0.096; 0.183; 0.000];
        sc.seedFinal = [-0.135; -0.988; 1.293; -0.305; 1.435; 0.000];

    case 'shelf'
        % A two-compartment shelf. Move from inside the lower compartment to
        % inside the upper one. The straight-line path drives the wrist
        % straight up through the middle board. A good solution must pull
        % out, move up, and push back in: a long detour that a local,
        % smoothness-regularised optimiser struggles to discover.
        sc.title = 'Shelf: lower to upper compartment';
        sc.description = ['30 cm deep shelf with boards at z = 0.32 and 0.64 m, ' ...
            'side and back walls; move from the lower to the upper compartment.'];
        sc.shapes = [table, ...
            shp('box', [0.65  0.00 0.32], [0.30 0.50 0.02], 'middle board'), ...
            shp('box', [0.65  0.00 0.64], [0.30 0.50 0.02], 'top board'), ...
            shp('box', [0.81  0.00 0.33], [0.02 0.50 0.66], 'back wall'), ...
            shp('box', [0.65 -0.26 0.33], [0.30 0.02 0.66], 'side wall'), ...
            shp('box', [0.65  0.26 0.33], [0.30 0.02 0.66], 'side wall')];
        sc.taskInit  = trvec2tform([0.60 0 0.12]) * rotm2tform(Rcup(0));
        sc.taskFinal = trvec2tform([0.60 0 0.44]) * rotm2tform(Rcup(0));
        sc.seedInit  = [-0.270; -0.999; 1.871; -0.872; 1.301; 0.000];
        sc.seedFinal = [-0.270; -1.353; 1.449; -0.096; 1.301; 0.000];
        % via-point in front of the shelf, between the two compartment
        % heights (used by the "better initial guess" test in Task4_Extras)
        sc.taskVia = trvec2tform([0.32 0 0.30]) * rotm2tform(Rcup(0));
        sc.seedVia = [-0.650; -2.033; 2.407; -0.374; 0.921; 0.000];

    otherwise
        error('helperScenarioUR5e: unknown scenario "%s"', name);
end
end

%% ---- local helpers ----
function s = shp(type, center, dims, name, yaw)
if nargin < 5, yaw = 0; end
s = struct('type', type, 'center', center, 'dims', dims, 'name', name, 'yaw', yaw);
end

function R = Rdown(yaw)
% tool z = -world z, tool x rotated by yaw about world z
c = cos(yaw); s = sin(yaw);
R = [c s 0; s -c 0; 0 0 -1];
end

function R = Rcup(dirDeg)
% tool z horizontal along dirDeg, tool y = world z (upright), x = y cross z
a = deg2rad(dirDeg);
z = [cos(a); sin(a); 0];
y = [0; 0; 1];
x = cross(y, z);
R = [x y z];
end
