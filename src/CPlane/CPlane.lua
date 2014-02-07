--[[
	CPlane Interface:
		- Ties into to low level cpp code which currently classifies each point in the scan and
		  provides some basic region growing code
]]--

cplane_ffi = require('./cplane_ffi')

Class()

function classifyPoints( points, window, dist_thresh )
	errors = torch.Tensor(points:size(2), points:size(3))
	means_ret = torch.Tensor(points:size())
	normals_ret = torch.Tensor(points:size())
	second_moments_ret = torch.Tensor(9, points:size(2), points:size(3))

	cplane_ffi.classifyPoints( torch.cdata(points), window, dist_thresh, torch.cdata(errors),
							   torch.cdata(normals_ret), torch.cdata(means_ret), torch.cdata(second_moments_ret) ) 

	return errors, means_ret, normals_ret, second_moments_ret	
end

function growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )
	region_mask = torch.Tensor(normals:size(2), normals:size(3))
	front_mask = torch.Tensor(normals:size(2), normals:size(3))

	-- TODO: should actually return the plane parameters 
	cplane_ffi.grow_plane_region( torch.cdata(inds), torch.cdata(normals), torch.cdata(means), torch.cdata(second_moments), torch.cdata(region_mask), torch.cdata(front_mask), normal_thresh, residual_thresh )

	return region_mask
end

function  test_cplane( job_id, scan_num )
	scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)
	pc = PointCloud.PointCloud.new( scan_fname )
	points = pc:get_xyz_map()	

	-- some default params	
	window = 19
	dist_thresh = 81.0

	normal_thresh = 0.5;
	residual_thresh = 20;

	errors, means, normals, second_moments = classifyPoints( points, window, dist_thresh )

	y = (torch.rand(1):mul(points:size(2)):int()+1):squeeze()
	x = (torch.rand(1):mul(points:size(3)):int()+1):squeeze()
	print("y: ",y)
	print("x: ",x)
	inds = torch.LongTensor({y,x})
	print(inds)

	-- Randomly generate start indices
	region_mask = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )

	image.display(errors)
	image.display(means)
	image.display(normals)
	image.display(region_mask)
end

function extract_planes_test( job_id, scan_num )

	scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)
	pc = PointCloud.PointCloud.new( scan_fname )
	points = pc:get_xyz_map()	

	window = 19
	dist_thresh = 81.0

	normal_thresh = 0.5;
	residual_thresh = 15;

	errors, means, normals, second_moments = classifyPoints( points, window, dist_thresh )
	image.display(errors)
	min_err, min_ind = errors:min();
	print("min_err: ", min_err);
	print("min_ind: ", min_ind);

end

