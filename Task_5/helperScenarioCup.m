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
%                The board (top at z = 0.25 m) sits in front of the robot,
%                like a low bench top or a wall-mounted shelf board (its
%                supports are not modelled). Expected: without the
%                constraint STOMP lifts the cup over the board but tips it
%                ~40 deg (it would spill); with the constraint it stays
%                nearly upright.
%   board_high : same, board 8 cm higher (top at z = 0.33 m). Expected
%                FAILURE case: with the constraint STOMP gets stuck trying to
%                slide the cup UNDER the board (staying low and level is
%                cheap for the constraint), a local minimum it never leaves,
%                whereas the unconstrained planner sometimes finds the lift-over
%                route by tilting the cup.
function sc = helperScenarioCup(name)

switch lower(name)
    case 'window'
        sc = helperScenarioUR5e('window');
        sc.name = 'window';

    case {'board_low', 'board_high'}
        if strcmpi(name, 'board_low'), Zs = 0.22; else, Zs = 0.30; end
        sc.name = lower(name);
        sc.title = sprintf('Cup over a low board (board top at z = %.2f m)', Zs + 0.03);
        sc.description = sprintf(['65 x 24 x 3 cm board (underside at z = %.2f m) in front of the robot; ' ...
            'carry an upright cup from y = -0.35 m to y = +0.35 m, 10 cm above the table.'], Zs);
        sc.shapes = [shp('box', [0 0 -0.04], [1.6 1.6 0.06], 'table'), ...
                     shp('box', [0.625 0 Zs + 0.015], [0.65 0.24 0.03], 'board')];
        sc.taskInit  = trvec2tform([0.62 -0.35 0.10]) * rotm2tform(Rcup(0));
        sc.taskFinal = trvec2tform([0.62  0.35 0.10]) * rotm2tform(Rcup(0));
        sc.seedInit  = [-0.806; -0.747; 1.443; -0.696; 0.765; 0.000];
        sc.seedFinal = [ 0.378; -0.747; 1.443; -0.696; 1.949; 0.000];
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
