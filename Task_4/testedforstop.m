sc = helperScenarioUR5e('pillar');  [world, vw] = buildVoxelWorld(sc.shapes);
model.spheres = ur5eSphereModel(robot);
q0 = ur5eSolveIK(robot,'tool0',sc.taskInit,sc.seedInit);  qT = ur5eSolveIK(robot,'tool0',sc.taskFinal,sc.seedFinal);
opts = struct('convergenceThreshold', -1, 'maxIterations', 150);   % never stop early
out = runTrials(robot, q0, qT, vw, world, model, opts, 1:3);  disp(out.success')