%% Task 5 scenarios: carrying a cup (end-effector y-axis must stay upright)
% sc = helperScenarioCup(name)   name: 'window' | 'board_low' | 'board_high'
%
% All start and goal poses use the "cup" orientation: tool z horizontal,
% tool y pointing straight up (the brief asks to adjust the fixed start and
% goal orientations to match the constraint, since they are not planned).
%
%   window     : the Task 4 window scenario (already uses the cup pose).
%                Expected: constraint works, cup stays nearly upright.
%   board_low  : carry the cup from one side of a low board to the other.
%                The board (top at z = 0.25 m, from 0.42 to 0.95 m in x) sits in front of the robot,
%                like a low bench top or a wall-mounted shelf board (its
%                supports are not modelled). Expected: without the
%                constraint STOMP lifts the cup over the board but tips it
%                ~40 deg (it would spill); with the constraint it stays
%                nearly upright.
%   board_high : same, board 8 cm higher (top at z = 0.33 m). Expected
%                (mostly) FAILURE case: with the constraint STOMP gets stuck trying to
%                slide the cup UNDER the board (staying low and level is
%                cheap for the constraint), a local minimum it never leaves,
%                whereas the unconstrained planner sometimes finds the lift-over
%                route by tilting the cup.
%
% sc = helperScenarioCup(name, params): params.boardZ overrides the height
% of the board underside (m), used by the board-height screening in
% Task5_Extras.m. Each scenario also carries a via-point pose
% (sc.taskVia, sc.seedVia) used for the "better initial guess" experiment.
function sc = helperScenarioCup(name, params)

if nargin < 2, params = struct(); end

switch lower(name)
    case 'window'
        sc = helperScenarioUR5e('window');
        sc.name = 'window';

    case {'board_low', 'board_high'}
        if strcmpi(name, 'board_low'), Zs = 0.22; else, Zs = 0.30; end
        if isfield(params, 'boardZ'), Zs = params.boardZ; end
        sc.name = lower(name);
        sc.title = sprintf('Cup over a low board (board top at z = %.2f m)', Zs + 0.03);
        sc.description = sprintf(['53 x 24 x 3 cm board (underside at z = %.2f m) in front of the robot; ' ...
            'carry an upright cup from y = -0.42 m to y = +0.42 m, 10 cm above the table.'], Zs);
        % v2: board starts 0.42 m from the base (was 0.30 m) and the
        % endpoints moved out to y = +-0.42 m (was +-0.35 m). In v1 the start
        % pose of the high board cleared the board by < 1 cm (the elbow
        % passed its near edge), so failures could not be blamed on the
        % constraint alone. Now both endpoints clear the board by > 5 cm.
        sc.shapes = [shp('box', [0 0 -0.04], [1.6 1.6 0.06], 'table'), ...
                     shp('box', [0.685 0 Zs + 0.015], [0.53 0.24 0.03], 'board')];
        sc.taskInit  = trvec2tform([0.62 -0.42 0.10]) * rotm2tform(Rcup(0));
        sc.taskFinal = trvec2tform([0.62  0.42 0.10]) * rotm2tform(Rcup(0));
        sc.seedInit  = [-0.880; -0.666; 1.277; -0.612; 0.691; 0.000];
        sc.seedFinal = [ 0.478; -0.666; 1.277; -0.612; 2.049; 0.000];
        % via-point above the board centre, cup upright (Task5_Extras)
        sc.taskVia = trvec2tform([0.62 0 0.48]) * rotm2tform(Rcup(0));
        sc.seedVia = [-0.259; -1.307; 1.289; 0.018; 1.312; 0.000];
        sc.view = [120 20];
        sc.axisLimits = [-0.3 1.0 -0.7 0.7 -0.1 0.9];

    otherwise
        error('helperScenarioCup: unknown scenario "%s"', name);
end
end

function s = shp(type, center, dims, name, yaw)
if nargin < 5, yaw = 0; end
s = struct('type', type, 'center', center, 'dims', dims, 'name', name, 'yaw', yaw);
end

function R = Rcup(dirDeg)
a = deg2rad(dirDeg);
z = [cos(a); sin(a); 0];
y = [0; 0; 1];
R = [cross(y, z) y z];
end
