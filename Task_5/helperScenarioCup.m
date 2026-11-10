%% Task 5 scenarios: carrying a cup (end-effector y-axis must stay upright)
% sc = helperScenarioCup(name)          name: 'window' | 'shelf' | 'pillar' | 'clutter'
% sc = helperScenarioCup(name, params)  params.pillarX: pillar distance (m), 'pillar' only
%
% Task 5 reuses the Task 4 scenarios (same obstacles), so that the effect of
% the constraint can be compared directly with Task 4. Only the start and
% goal ORIENTATIONS differ where needed: both endpoints must hold the cup
% upright (tool z horizontal, tool y = world z), because STOMP never moves
% the endpoints.
%
%   window  : Task 4 window scenario, unchanged (already cup-upright).
%   shelf   : Task 4 shelf scenario, unchanged (already cup-upright).
%   pillar  : Task 4 pillar obstacle; same start/goal positions, gripper
%             changed from pointing down to holding the cup upright, tool
%             pointing radially outward (-45 / +45 deg).
%   clutter : Task 4 cluttered table; the cup is carried from (0.40,-0.40)
%             to (0.40, 0.40) m at z = 0.20 m, tool pointing radially outward.
%             (The Task 4 top-down pick poses cannot be used: with the cup
%             upright, the wrist sits above the tool and the Task 4 poses
%             would put it inside the clutter objects.)
%
% IK seeds put start and goal on the same arm/wrist configuration, with
% joint 6 near 0. (If one endpoint had joint 6 = +180 deg and the other
% -180 deg, the straight-line initial path would spin the wrist a full turn
% and turn the cup upside down.)
%
% sc.taskVia / sc.seedVia (pillar only): a via-point for the "better initial
% guess" experiment in Task5_Extras.m.
function sc = helperScenarioCup(name, params)

if nargin < 2, params = struct(); end

switch lower(name)
    case 'window'
        sc = helperScenarioUR5e('window');

    case 'shelf'
        sc = helperScenarioUR5e('shelf');

    case 'pillar'
        sc = helperScenarioUR5e('pillar', params);
        sc.title = [sc.title ' (cup upright)'];
        sc.taskInit  = trvec2tform([0.45 -0.45 0.35]) * rotm2tform(Rcup(-45));
        sc.taskFinal = trvec2tform([0.45  0.45 0.35]) * rotm2tform(Rcup( 45));
        sc.seedInit  = [-1.036; -1.224; 1.516; -0.291; 1.320; 0.000];
        sc.seedFinal = [ 0.534; -1.224; 1.516; -0.291; 1.320; 0.000];
        % via-point between base and pillar, cup upright
        sc.taskVia = trvec2tform([0.28 0 0.40]) * rotm2tform(Rcup(0));
        sc.seedVia = [-0.831; -2.261; 2.237; 0.024; 0.739; 0.000];

    case 'clutter'
        sc = helperScenarioUR5e('clutter');
        sc.title = 'Cluttered tabletop (cup upright)';
        sc.description = ['Task 4 cluttered table; carry an upright cup from (0.40, -0.40) ' ...
            'to (0.40, 0.40) m at z = 0.20 m, over the divider and under the overhead bar.'];
        sc.taskInit  = trvec2tform([0.40 -0.40 0.20]) * rotm2tform(Rcup(-45));
        sc.taskFinal = trvec2tform([0.40  0.40 0.20]) * rotm2tform(Rcup( 45));
        sc.seedInit  = [-1.075; -1.204; 1.927; -0.723; 1.281; 0.000];
        sc.seedFinal = [ 0.495; -1.204; 1.927; -0.723; 1.281; 0.000];

    otherwise
        error('helperScenarioCup: unknown scenario "%s"', name);
end
sc.name = lower(name);
end

function R = Rcup(dirDeg)
% tool z horizontal along dirDeg, tool y = world z (cup upright)
a = deg2rad(dirDeg);
z = [cos(a); sin(a); 0];
y = [0; 0; 1];
R = [cross(y, z) y z];
end
