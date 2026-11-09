%% Construct voxel obstacle representation for STOMP planner (UR5e, Task 2)
% Same method as helperCreateObstaclesKINOVA.m (Task 1). Only the box
% position changed: for the UR5e the Kinova box at (0.4, 0.46, 0.26) would
% overlap the UR5e's goal configuration, so it is moved between the new
% start and goal poses.
%
% The UR5e reaches ~0.85 m from the shoulder, so the original 2 m voxel
% cube (-1 m to +1 m) still contains the whole arm and the sampled paths.
Env_size = [-1, -1, -1; 2, 2, 2];   % [xmin ymin zmin; x y z lengths]
voxel_size = [0.02, 0.02, 0.02];    % m
binary_world = zeros(Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3));

%% Static obstacle
lbox = 0.16;                       % cube edge length [m]
box_center = [0.45 0.25 0.25];     % between start (y = -0.10) and goal (y = 0.55)
aObs = collisionBox(lbox, lbox, lbox);
aObs.Pose = trvec2tform(box_center);
world = {aObs};

%% Voxelize the box (as in Task 1)
cube_metric = [box_center - lbox/2; lbox, lbox, lbox];
cube_voxel = [ceil((cube_metric(1, :) - Env_size(1, :)) ./ voxel_size); ...
              ceil((cube_metric(1, :) + cube_metric(2, :) - Env_size(1, :)) ./ voxel_size)];
[xc, yc, zc] = meshgrid(cube_voxel(1, 1):cube_voxel(2, 1), cube_voxel(1, 2):cube_voxel(2, 2), cube_voxel(1, 3):cube_voxel(2, 3));
binary_world(sub2ind(size(binary_world), xc, yc, zc)) = 1;

%% Signed Euclidean distance field
voxel_world_sEDT = prod(voxel_size) ^ (1/3) * sEDT_3d(binary_world);
voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size;
voxel_world.sEDT = voxel_world_sEDT;
