%% Task 2: repeated-seed STOMP validation test
% Copy this file into Task_2 and run it in MATLAB.
% Does not overwrite Task2_UR5e_result.mat.
clear; clc;
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
rootDir = fileparts(thisDir);
task1Dir = fullfile(rootDir,'Task 1');
if ~isfolder(task1Dir), task1Dir = fullfile(rootDir,'Task_1'); end
assert(isfolder(task1Dir), 'Task 1 folder not found beside Task_2');
addpath(task1Dir);
addpath(thisDir);
resultsDir = fullfile(thisDir,'Results');
if ~isfolder(resultsDir), mkdir(resultsDir); end

robot = loadrobot('universalUR5e','DataFormat','column');
endEffector = 'tool0';
R_down = diag([1 -1 -1]);
taskInit = trvec2tform([0.45 -0.10 0.20]) * rotm2tform(R_down);
taskFinal = trvec2tform([0.40 0.55 0.30]) * rotm2tform(R_down);
seedInit = [-0.512; -1.407; 2.210; -2.374; -1.571; -2.083];
seedFinal = [0.745; -1.081; 1.440; -1.930; -1.571; -0.826];
[q0, ~] = ur5eSolveIK(robot,endEffector,taskInit,seedInit);
[qT, ~] = ur5eSolveIK(robot,endEffector,taskFinal,seedFinal);
helperCreateObstaclesUR5e;
model.spheres = ur5eSphereModel(robot);

opts = struct('nDiscretize',20,'nPaths',20, ...
    'convergenceThreshold',0.1,'maxIterations',50,'verbose',false);
seeds = [1 2 3 4 5 42 10 20 30 100];
n = numel(seeds);
iterations = zeros(n,1); runtime_s = zeros(n,1);
initial_cost = zeros(n,1); final_cost = zeros(n,1);
colliding_waypoints = zeros(n,1); colliding_dense = zeros(n,1);
min_clearance_m = zeros(n,1); success = false(n,1);
stop_reason = strings(n,1);
runResults = cell(n,1);
for i = 1:n
    rng(seeds(i),'twister');
    t = tic;
    res = stompPlan(robot,q0,qT,voxel_world,model,opts);
    runtime_s(i) = toc(t);
    c = stompCheckCollision(robot,res.theta,world);
    iterations(i) = res.iter;
    initial_cost(i) = res.Qtheta_initial;
    final_cost(i) = res.Qtheta;
    colliding_waypoints(i) = c.nWp;
    colliding_dense(i) = c.nDense;
    min_clearance_m(i) = c.minClearance;
    success(i) = c.free;
    stop_reason(i) = string(res.stopReason);
    runResults{i} = struct('seed',seeds(i),'planner',res,'collision',c);
    fprintf('Seed %3d | success %d | iterations %2d | Q %.2f | clearance %.3f m | %.2f s\n', ...
        seeds(i),success(i),iterations(i),final_cost(i),min_clearance_m(i),runtime_s(i));
end
summary = table(seeds(:),success,iterations,runtime_s,initial_cost,final_cost, ...
    colliding_waypoints,colliding_dense,min_clearance_m,stop_reason, ...
    'VariableNames',{'Seed','Success','Iterations','Runtime_s','InitialCost', ...
    'FinalCost','CollidingWaypoints','CollidingDense','MinClearance_m','StopReason'});
stamp = datestr(now,'yyyymmdd_HHMMSS');
base = fullfile(resultsDir,['Task2_Validation_' stamp]);
writetable(summary,[base '.csv']);
save([base '.mat'],'summary','runResults','opts','seeds');
fprintf('\nSuccess rate: %d/%d (%.1f%%)\n',sum(success),n,100*mean(success));
fprintf('Saved: %s.csv and .mat\n',base);
disp(summary);

%% Validation figures (saved separately; existing results are untouched)
seedLabels = string(seeds);
fig = figure('Color','w','Name','Task 2 STOMP Validation','Position',[100 100 1100 800]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile;
bar(final_cost); grid on;
set(gca,'XTick',1:n,'XTickLabel',seedLabels);
xlabel('Random seed'); ylabel('Final trajectory cost');
title('Final cost by seed');
nexttile;
bar(runtime_s); grid on;
set(gca,'XTick',1:n,'XTickLabel',seedLabels);
xlabel('Random seed'); ylabel('Runtime (s)');
title('Optimization runtime');
nexttile;
bar(min_clearance_m*1000); grid on;
set(gca,'XTick',1:n,'XTickLabel',seedLabels);
xlabel('Random seed'); ylabel('Minimum clearance (mm)');
title('Sphere-model obstacle clearance');
nexttile;
bar(iterations); grid on;
set(gca,'XTick',1:n,'XTickLabel',seedLabels);
xlabel('Random seed'); ylabel('Iterations');
title('Iterations to stop');
sg = sgtitle(sprintf( ...
    'UR5e STOMP validation | %d/%d collision-free | mean runtime %.2f s', ...
    sum(success),n,mean(runtime_s)));

sg.Color = 'k';
sg.FontWeight = 'bold';

%% Figure formatting - White background, black text

% Set figure background to white
fig.Color = 'w';

% Set all subplot axes
ax = findall(fig, 'Type', 'axes');

for k = 1:numel(ax)
    ax(k).Color = 'w';          % White plot background
    ax(k).XColor = 'k';         % Black X-axis
    ax(k).YColor = 'k';         % Black Y-axis
    ax(k).Title.Color = 'k';    % Black subplot title
    ax(k).XLabel.Color = 'k';   % Black X-axis label
    ax(k).YLabel.Color = 'k';   % Black Y-axis label
    ax(k).LineWidth = 1;
    ax(k).GridColor = [0.7 0.7 0.7];
    ax(k).GridAlpha = 0.2;
    ax(k).FontSize = 11;
end

% Set all text, including main title, to black
allText = findall(fig, 'Type', 'text');
set(allText, 'Color', 'k');

% Explicitly set the main title to black
mainTitle = findall(fig, 'Type', 'Text', ...
    'String', 'UR5e STOMP validation');

exportgraphics(fig,[base '_graphs.png'],'Resolution',300);
savefig(fig,[base '_graphs.fig']);
fprintf('Saved graphs: %s_graphs.png and .fig\n',base);
