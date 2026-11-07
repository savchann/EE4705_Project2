%% Build the obstacle world from a list of primitive shapes (Task 4)
% The Task 1/2 obstacle script only handles ONE axis-aligned cube. This
% function takes any number of boxes (optionally rotated about z), spheres
% and cylinders, and builds BOTH representations the code needs, from the
% same shape list so they always agree:
%   world       : cell array of MATLAB collision objects, used for
%                 checkCollision (verification) and for plotting
%   voxel_world : occupancy grid + signed distance field, used by STOMP's
%                 obstacle cost
% Adding a shape to one but not the other is an easy mistake (the obstacle
% is drawn but the planner ignores it), so both are generated here.
%
% INPUT shapes: struct array with fields
%   type   : 'box' | 'sphere' | 'cylinder'
%   center : [x y z] (m)
%   dims   : box [lx ly lz] | sphere r | cylinder [r h] (cylinder axis = z)
%   yaw    : rotation about z (rad), boxes only (optional, default 0)
%   name   : label (optional)
% OUTPUT: world, voxel_world (same fields as in Task 1)
%
% Voxelisation: a voxel is occupied if its centre lies within half a voxel
% of the shape, i.e. shapes are slightly inflated (like the Task 1 ceil()
% indexing). Thin plates (2 cm shelf boards) are therefore never lost
% between voxel centres.
function [world, voxel_world] = buildVoxelWorld(shapes, Env_size, voxel_size)

if nargin < 2, Env_size = [-1, -1, -1; 2, 2, 2]; end
if nargin < 3, voxel_size = [0.02, 0.02, 0.02]; end

n = round(Env_size(2, :) ./ voxel_size);
xv = Env_size(1, 1) + voxel_size(1) * ((1:n(1)) - 0.5);
yv = Env_size(1, 2) + voxel_size(2) * ((1:n(2)) - 0.5);
zv = Env_size(1, 3) + voxel_size(3) * ((1:n(3)) - 0.5);
% ndgrid (not meshgrid) so that array index (i,j,k) <-> (x,y,z), matching
% the sub2ind(world_size, ix, iy, iz) lookup in stompObstacleCost
[Xw, Yw, Zw] = ndgrid(xv, yv, zv);

binary_world = false(n);
infl = 0.5 * max(voxel_size);
world = cell(1, numel(shapes));

for s = 1:numel(shapes)
    sh = shapes(s);
    c = sh.center(:)';
    yaw = 0;
    if isfield(sh, 'yaw') && ~isempty(sh.yaw), yaw = sh.yaw; end

    switch lower(sh.type)
        case 'box'
            d = sh.dims;
            dx = Xw - c(1); dy = Yw - c(2);
            lx =  cos(yaw) * dx + sin(yaw) * dy;     % coordinates in the box frame
            ly = -sin(yaw) * dx + cos(yaw) * dy;
            inside = abs(lx) <= d(1)/2 + infl & abs(ly) <= d(2)/2 + infl & ...
                     abs(Zw - c(3)) <= d(3)/2 + infl;
            obj = collisionBox(d(1), d(2), d(3));
            obj.Pose = trvec2tform(c) * axang2tform([0 0 1 yaw]);
        case 'sphere'
            r = sh.dims(1);
            inside = (Xw - c(1)).^2 + (Yw - c(2)).^2 + (Zw - c(3)).^2 <= (r + infl)^2;
            obj = collisionSphere(r);
            obj.Pose = trvec2tform(c);
        case 'cylinder'
            r = sh.dims(1); h = sh.dims(2);
            inside = (Xw - c(1)).^2 + (Yw - c(2)).^2 <= (r + infl)^2 & ...
                     abs(Zw - c(3)) <= h/2 + infl;
            obj = collisionCylinder(r, h);
            obj.Pose = trvec2tform(c);
        otherwise
            error('buildVoxelWorld: unknown shape type "%s"', sh.type);
    end
    binary_world = binary_world | inside;
    world{s} = obj;
end

binary_world = double(binary_world);
voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size;
voxel_world.sEDT = prod(voxel_size) ^ (1/3) * sEDT_3d(binary_world);

end
