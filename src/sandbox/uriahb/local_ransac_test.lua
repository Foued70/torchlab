--[[ 
	Soon to be deleted test for my local ransac 
]]-- 

plane_from_point = Plane.util.plane_from_point

job_id = "precise-transit-6548"
scan_num = 22 

scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)

pc = PointCloud.PointCloud.new( scan_fname )
points = pc:get_xyz_map()
normals = pc:get_normal_map()

imgh = normals:size(2)
imgw = normals:size(3)

x_ind = 500
y_ind = 500

point = points[{{},y_ind, x_ind}]:squeeze()
normal = normals[{{},y_ind, x_ind}]:squeeze()

-- Gotta validate my plane eqn somehow 
plane_eqn = plane_from_point( point, normal )
print("plane_eqn: ", plane_eqn)

-- Use local region to compute plane ... if I've chosen a smooth region this should be similar
idx = torch.IntTensor(2)
idx[1] = y_ind
idx[2] = x_ind
reference_window = 2
patch_mask = torch.ByteTensor(points:size(2), points:size(3)):zero()
bbx = Plane.finder_utils.compute_bbx(idx,reference_window,reference_window,imgh,imgw)
patch_mask:fill(0)
patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1
-- Configure plane validator and iterative fitting 
validate = Plane.Validate.new{
   residual_threshold   = 250,
   normal_threshold     = math.pi,
   min_points_for_seed  = 0,
   min_points_for_plane = 0  
}
current_plane, error_string =
        validate:seed(points, normals, patch_mask, debug_info)	


image.display( patch_mask )

-- Create plane, set eqn to the one from plane_from_point, center is just the point 
seed_plane = Plane.Plane.new{
		residual_threshold = 500,
	    normal_threshold   = math.pi/3
}
seed_plane.eqn = plane_eqn
seed_plane.center = point 

inlier_pts, inlier_n_pts, inlier_mask =
        seed_plane:filter_points(points, normals)

inlier_map = inlier_mask:reshape(imgh, imgw):repeatTensor(3,1,1)

image.display( points )
image.display( normals ) 
image.display( inlier_map )

if current_plane then 
	print("fitted plane: ", current_plane.eqn)
	inlier_pts, inlier_n_pts, inlier_mask =
	        current_plane:filter_points(points, normals)

	inlier_map = inlier_mask:reshape(imgh, imgw):repeatTensor(3,1,1)

	image.display( inlier_map )
end
