
function cost = stompObstacleCost(sphere_centers, radius, voxel_world, vel)

    % Safety margin around obstacles (meters)
    safety_margin = 0.05;

    % Initialize total obstacle cost
    cost = 0;

    %% Step 1: Retrieve signed distance field and voxel properties
    voxel_world_sEDT = voxel_world.sEDT;
    world_size = voxel_world.world_size;

    env_corner = voxel_world.Env_size(1,:);
    voxel_size = voxel_world.voxel_size;

    %% Step 2: Convert sphere centers into voxel indices
    env_corner_vec = repmat(env_corner, size(sphere_centers,1), 1);

    idx = floor((sphere_centers - env_corner_vec) ...
        ./ voxel_size) + 1;

    %% Step 3: Check whether sphere centers are inside voxel world
    valid = all(idx >= 1 & idx <= world_size, 2);

    if any(~valid)
        cost = 1e6;
        return;
    end

    %% Step 4: Obtain signed distance for each sphere center
    linearIdx = sub2ind(world_size, ...
        idx(:,1), idx(:,2), idx(:,3));

    distance = voxel_world_sEDT(linearIdx);

    %% Step 5: Calculate clearance from sphere surface
    clearance = distance(:) - radius(:);

    %% Step 6: Calculate obstacle penalty
    penalty = zeros(size(clearance));

    % Case 1: Sphere is within the safety margin
    near = clearance > 0 & clearance <= safety_margin;

    penalty(near) = ...
        (clearance(near) - safety_margin).^2 ...
        / (2 * safety_margin);

    % Case 2: Sphere is touching or penetrating an obstacle
    collision = clearance <= 0;

    penalty(collision) = ...
        safety_margin/2 - clearance(collision);

    %% Step 7: Multiply penalty by workspace speed
    cost_array = penalty .* vel(:);

    %% Step 8: Sum the obstacle costs
    cost = sum(cost_array);

end
