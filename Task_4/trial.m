load('Results/Task4_window.mat')            % then repeat with Task4_window.mat
robot = loadrobot('universalUR5e','DataFormat','column');
world = buildVoxelWorld(sc.shapes);  names = {sc.shapes.name};
for i = find(cRes.wpCollision)'
    for o = 1:numel(world)
        if checkCollision(robot, res.theta(:,i), world(o), 'IgnoreSelfCollision','on')
            fprintf('waypoint %d hits %s\n', i, names{o});
        end
    end
end