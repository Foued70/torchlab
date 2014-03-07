--[[
	CPlane Interface:
		- Ties into to low level cpp code which currently classifies each point in the scan and
		  provides some basic region growing code
]]--

-- Setting up gnuplot
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")  
gnuplot.setterm("x11")

io = require('io')
cplane_ffi = require('./cplane_ffi')

Plane = Plane.Plane
ArcIO = data.ArcIO
specs = data.ArcSpecs
meshlist = util.torch.meshlist

Class()

function classifyPoints( points, window, dist_thresh )
	invalid_mask = torch.Tensor(points:size(2), points:size(3))
	eigenvalues_ret = torch.Tensor(points:size())
	means_ret = torch.Tensor(points:size())
	normals_ret = torch.Tensor(points:size())
	second_moments_ret = torch.Tensor(9, points:size(2), points:size(3))

	cplane_ffi.classifyPoints( torch.cdata(points), window, dist_thresh, torch.cdata(invalid_mask), torch.cdata(eigenvalues_ret),
							   torch.cdata(normals_ret), torch.cdata(means_ret), torch.cdata(second_moments_ret) ) 

	return invalid_mask, eigenvalues_ret, means_ret, normals_ret, second_moments_ret	
end

function growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )
	region_mask = torch.Tensor(normals:size(2), normals:size(3))
	front_mask = torch.Tensor(normals:size(2), normals:size(3))

	-- Return values for plane 
	plane_centroid = torch.Tensor(3)
	plane_equation = torch.Tensor(4)
	plane_eigenvalues = torch.Tensor(3)

	cplane_ffi.grow_plane_region( torch.cdata(inds), torch.cdata(normals), torch.cdata(means), 
								  torch.cdata(second_moments), torch.cdata(region_mask), torch.cdata(front_mask), 
								  torch.cdata(plane_eigenvalues), torch.cdata(plane_equation), torch.cdata(plane_centroid), 
								  normal_thresh, residual_thresh )

	return plane_centroid, plane_equation, plane_eigenvalues, region_mask
end

function  cullPoints( normals, points, window, normal_thresh, residual_thresh )
	cull_mask = torch.Tensor(points:size(2), points:size(3))
	cplane_ffi.cullPoints( torch.cdata(cull_mask), torch.cdata(normals), torch.cdata(points), 
						   window, normal_thresh, residual_thresh )	
	return cull_mask
end

function test_classification( job_id, scan_num )
	arc_io = ArcIO.new( job_id, 'test_classification' )
	pc = arc_io:getScan( scan_num )

	pts = pc:get_xyz_map()
	points = pts:sub(1,3,50,pts:size(2), 1,pts:size(3)):clone()
	print(points)

	-- some default params	
	window = 3
	dist_thresh = 81.0

	normal_thresh = math.pi/8
	residual_thresh = 15;

	invalid_mask, eigenvalues, means, normals, second_moments = classifyPoints( points, window, dist_thresh )

	sml = 0.000000000000001
	eigenvalues = eigenvalues + sml

	image.display( invalid_mask )
	--image.display( eigenvalues )

	eig0 = eigenvalues:select(1,1)
	eig1 = eigenvalues:select(1,2)
	eig2 = eigenvalues:select(1,3)

	ones = torch.ones(eig0:size())

	smoothness = torch.cdiv(eig0, torch.add(eig1, eig2))
	cornerness = torch.cdiv(eig0, eig1)
	edgeness = torch.cdiv(eig1, eig2)

	image.display( means )
	image.display( normals )	
	image.display( cornerness )
	image.display( edgeness )
	print(smoothness:max()) 
	print(smoothness:min())
	print(torch.cdiv(ones, smoothness):min())
	print(torch.cdiv(ones, smoothness):max())

	-- view const used to rescale range since we are inverting
	view_const = 1

	image.display( torch.cdiv( ones, edgeness+view_const ) ) 
	image.display( torch.cdiv( ones, smoothness+view_const ) ) 
	image.display( torch.cdiv( ones, cornerness+view_const ) ) 

	-- Output cornerness values 
	xyz_list = points:reshape(3, points:size(2)*points:size(3))
	rgb_list = torch.cdiv( ones, cornerness+view_const ):reshape(1, points:size(2)*points:size(3)):repeatTensor(3,1)

	-- rescale rgb list to be from 0-255
	mx = rgb_list:max()
	mn = rgb_list:min()
	scl = 255.0/(mx-mn)
	rgb_list = rgb_list:add(-mn):mul(scl):int()

	print(rgb_list:min())
	print(rgb_list:max())

	print(xyz_list)
	print(rgb_list)

	print(xyz_list:size(2))
	
	output_fname = arc_io:workStr('cornerness_pts', string.format("%.3d.pts",scan_num))
	output_fh = io.open( output_fname, "w" )										
	pointcloud.pointcloud.write_to_ascii(output_fh, xyz_list:size(2), xyz_list:t(), rgb_list:t())
	

end

function  test_cplane( job_id, scan_num )
	--[[
	scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)
	pc = PointCloud.PointCloud.new( scan_fname )
	]]--

	arc_io = ArcIO.new( job_id, scan_num )
	pc = arc_io:getScan( scan_num )

	points = pc:get_xyz_map()		


	-- some default params	
	window = 3
	dist_thresh = 81.0

	normal_thresh = math.pi/8
	residual_thresh = 15;

	errors, means, normals, second_moments = classifyPoints( points, window, dist_thresh )

	--cull_mask = cullPoints( normals, means, 3, normal_thresh, residual_thresh )

	y = (torch.rand(1):mul(points:size(2)):int()+1):squeeze()
	x = (torch.rand(1):mul(points:size(3)):int()+1):squeeze()
	print("y: ",y)
	print("x: ",x)
	inds = torch.LongTensor({y,x})
	print(inds)

	-- Randomly generate start indices
	--plane_centroid, plane_equation, region_mask = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )
	pcent, peq, pegv, reg_msk = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )

	print("plane_centroid: ", plane_centroid)
	print("plane_equation: ", plane_equation)
	print("errors:max(): ", errors:max())
	print("errors:min(): ", errors:min())

	image.display( means )
	image.display( normals )
	image.display( reg_msk )
	--image.display( errors )
	--image.display( cull_mask )
end

function extract_planes_deterministic( job_id, scan_num )
	-- Extract planes by extracting planes following the smoothest regions first
	arc_io = ArcIO.new( job_id, 'test_classification' )
	pc = arc_io:getScan( scan_num )

	points = pc:get_xyz_map()		
	-- Ignore top 50 rows since this data is super noisy
	points = points:sub(1,3,50,points:size(2), 1,points:size(3)):clone()

	print(points)

	areas = pc:get_area_map()
	areas = areas:sub(1,points:size(2), 1,points:size(3)):clone()		

	print(areas)

	-- some default params	
	window = 3
	dist_thresh = 81.0

	normal_thresh = math.pi/8
	residual_thresh = 15;

	invalid_mask, eigenvalues, means, normals, second_moments = classifyPoints( points, window, dist_thresh )

	sml = 0.000000000000001
	view_const = 2
	eigenvalues = eigenvalues + sml

	image.display( invalid_mask )
	--image.display( eigenvalues )

	eig0 = eigenvalues:select(1,1)
	eig1 = eigenvalues:select(1,2)
	eig2 = eigenvalues:select(1,3)

	ones = torch.ones(eig0:size())
	cornerness = torch.cdiv(eig0, eig1)
	smoothness = torch.cdiv(ones, cornerness)

	smoothness[invalid_mask:byte()] = smoothness:max()

	max_planes = 200
	coverage = 0.98

	coverage_mask = torch.ByteTensor(points:size(2), points:size(3)):zero()

	print(smoothness:size(1))
	print(smoothness:size(2))

	plane_cnt = 1
	while plane_cnt < max_planes do 	
		-- Find x,y to start sample at 
		min_v, min_i = torch.min( smoothness:reshape(smoothness:nElement()), 1 )
		
		min_x = math.floor((min_i:squeeze()-1)/smoothness:size(2))
		min_y = (min_i:squeeze()-1) - min_x*smoothness:size(2)
		min_x = min_x + 1
		min_y = min_y + 1

		print(min_v)
		print(smoothness[{{min_x},{min_y}}])
		inds = torch.LongTensor({min_x,min_y})

		pcent, peq, pegv, reg_msk = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )

		smoothness[reg_msk:byte()] = smoothness:max()		

		print(reg_msk)
		print("weight: ", areas[reg_msk:byte()]:sum())

		if areas[reg_msk:byte()]:sum() > 2500 then 
			coverage_mask = (coverage_mask + reg_msk:byte()):ge(1)
			plane_cnt = plane_cnt + 1
		end		
		collectgarbage()
	end

	
	image.display( normals )
	image.display( invalid_mask )
	image.display( torch.cdiv( ones, cornerness+view_const ) )


	image.display(smoothness)
	image.display(reg_msk)
	image.display(coverage_mask)
	
end

function extract_planes_random( job_id, scan_num )
	-- Extract planes by extracting planes following the smoothest regions first
	arc_io = ArcIO.new( job_id, 'test_classification' )
	pc = arc_io:getScan( scan_num )

	points = pc:get_xyz_map()		
	-- Ignore top 50 rows since this data is super noisy
	points = points:sub(1,3,50,points:size(2), 1,points:size(3)):clone()

	print(points)

	areas = pc:get_area_map()
	areas = areas:sub(1,points:size(2), 1,points:size(3)):clone()		

	print(areas)

	-- some default params	
	window = 3
	dist_thresh = 81.0

	normal_thresh = math.pi/8
	residual_thresh = 15;

	invalid_mask, eigenvalues, means, normals, second_moments = classifyPoints( points, window, dist_thresh )

	sml = 0.000000000000001
	view_const = 2
	eigenvalues = eigenvalues + sml

	image.display( invalid_mask )
	--image.display( eigenvalues )

	eig0 = eigenvalues:select(1,1)
	eig1 = eigenvalues:select(1,2)
	eig2 = eigenvalues:select(1,3)

	ones = torch.ones(eig0:size())
	cornerness = torch.cdiv(eig0, eig1)
	smoothness = torch.cdiv(ones, cornerness)

	smoothness[invalid_mask:byte()] = smoothness:max()

	max_planes = 200
	coverage = 0.98

	coverage_mask = torch.ByteTensor(points:size(2), points:size(3)):zero()

	print(smoothness:size(1))
	print(smoothness:size(2))

	plane_cnt = 1
	while plane_cnt < max_planes do 	
		-- Find x,y to start sample at 
		min_v, min_i = torch.min( smoothness:reshape(smoothness:nElement()), 1 )
		
		min_x = math.floor((min_i:squeeze()-1)/smoothness:size(2))
		min_y = (min_i:squeeze()-1) - min_x*smoothness:size(2)
		min_x = min_x + 1
		min_y = min_y + 1

		print(min_v)
		print(smoothness[{{min_x},{min_y}}])
		inds = torch.LongTensor({min_x,min_y})

		pcent, peq, pegv, reg_msk = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )

		smoothness[reg_msk:byte()] = smoothness:max()		

		print(reg_msk)
		print("weight: ", areas[reg_msk:byte()]:sum())

		if areas[reg_msk:byte()]:sum() > 2500 then 
			coverage_mask = (coverage_mask + reg_msk:byte()):ge(1)
			plane_cnt = plane_cnt + 1
		end		
		collectgarbage()
	end

	
	image.display( normals )
	image.display( invalid_mask )
	image.display( torch.cdiv( ones, cornerness+view_const ) )


	image.display(smoothness)
	image.display(reg_msk)
	image.display(coverage_mask)
end

-- TODO: move!
function extract_planes_directed( scan_num )

	arc_io = ArcIO.new( specs.job_id, specs.work_id )
	pc = arc_io:getScan( scan_num )

	--[[
	scan_fname = string.format('/Users/uriah/Downloads/' .. job_id .. '/source/po_scan/a/%.3d/sweep.xyz', scan_num)
	pc = PointCloud.PointCloud.new( scan_fname )
	]]--

	points = pc:get_xyz_map()	

	window = 3
	dist_thresh = 81.0

	normal_thresh = 0.3
	residual_thresh = 15

	min_patch_size = 0

	n_planes = 20

	errors, means, normals, second_moments = classifyPoints( points, window, dist_thresh )
	--image.display(errors)

	cmap = image.colormap(n_planes)

	regions_rgb = torch.Tensor(points:size())
	regions_r = regions_rgb[1]
	regions_g = regions_rgb[2]
	regions_b = regions_rgb[3]

	--[[
	region_masks = torch.ByteTensor( n_planes, points:size(2), points:size(3) )
	plane_centroids = torch.Tensor( n_planes, 3 )
	plane_equations = torch.Tensor( n_planes, 4 )
	]]--
	plane_inds = torch.LongTensor( n_planes, 2 )
	planes = {}

	region_cnt = 1 	
	-- Randomly generate start indices
	for i=1,n_planes do 

		min_cols, min_col_inds = torch.min(errors,1)
		min_rows, min_j = torch.min(min_cols,2)
		min_j = min_j:squeeze()
		min_i = min_col_inds:squeeze()[min_j]
		inds = torch.LongTensor({min_i,min_j})
		print(inds)

		plane_centroid, plane_equation, region_mask = growPlane( inds, normals, points, second_moments, normal_thresh, residual_thresh )

		-- Update errors mask
		errors = torch.add( errors,region_mask )
		errors[errors:gt(1)] = 1

		plane_inds[{{i}, {}}] = inds
		if region_mask:sum() > min_patch_size then
			-- Update regions so that we can view the output
			region_mask = region_mask:byte()
			regions_r[region_mask] = cmap[{i,1}]
			regions_g[region_mask] = cmap[{i,2}]
			regions_b[region_mask] = cmap[{i,3}]

			plane = Plane.new()
			plane.sample_inds = inds
			plane.centroid = plane_centroid
			plane.eqn = plane_equation
			plane.inlier_map = region_mask
			table.insert(planes, plane)

			--plane_inds[{{region_cnt}, {}}] = inds

			--[[
			regions_r[{{min_i}, min_j}}] = 1.0;
			regions_g[{{min_i}, min_j}}] = 1.0;
			regions_b[{{min_i}, min_j}}] = 1.0;

			-- Copy region mask over 
			region_masks[{{region_cnt}, {},{}}] = region_mask
			plane_centroids[{{region_cnt},{}}] = plane_centroid
			plane_equations[{{region_cnt},{}}] = plane_equation
			]]--

			region_cnt = region_cnt+1
		end
		collectgarbage()
	end

	-- DEBUG overlay sample points on normals
	normals_x = normals[1]
	normals_y = normals[2]
	normals_z = normals[3]

	for i=1,n_planes-1 do 
		min_i = plane_inds[{{i},{1}}]:squeeze()
		min_j = plane_inds[{{i},{2}}]:squeeze()
		regions_r[{{min_i}, {min_j}}] = 1.0;
		regions_g[{{min_i}, {min_j}}] = 1.0;
		regions_b[{{min_i}, {min_j}}] = 1.0;

		normals_x[{{min_i}, {min_j}}] = 1.0;
		normals_y[{{min_i}, {min_j}}] = 1.0;
		normals_z[{{min_i}, {min_j}}] = 1.0;
	end

	print("region_cnt: ", region_cnt )

	-- Output region masks 
	output = {}
	output.n_planes = region_cnt
	output.planes = planes
	output.normal_thresh = normal_thresh
	output.residual_thresh = residual_thresh
	arc_io:dumpTorch( output, "region_masks",string.format("%.3d", scan_num) ) 

	-- Output images	
	arc_io:dumpImage( means, "scan_means", string.format("%.3d", scan_num) )
	arc_io:dumpImage( normals:add(1):mul(0.5), "scan_normals", string.format("%.3d", scan_num) )
	arc_io:dumpImage( regions_rgb, "scan_regions", string.format("%.3d", scan_num) )
end

function extract_planes_ransac( job_id, scan_num )
	arc_io = ArcIO.new( job_id, 'extract_planes_ransac' )
	pc = arc_io:getScan( scan_num )

	points = pc:get_xyz_map()	
	
	print(points)
	-- Ignore top 50 rows since this data is super noisy
	--points = points:sub(1,3,50,points:size(2), 1,points:size(3)):clone()
	print(points)

	area = pc:get_area_map()
	--area = area:sub(50,area:size(1),1,area:size(2)):clone()

	window = 9
	dist_thresh = 9.0

	normal_thresh = math.pi/8
	residual_thresh = 15

	min_region_size = 2500

	n_planes = 300
	coverage_thresh = 0.9 -- TODO: have a pointcloud coverage threshold 
	
	cull_map, eigenvalues, means, normals, second_moments = classifyPoints( points, window, dist_thresh )
	-- Only sample from smooth regions in the cull mask 	
	--cull_map = cullPoints( normals, means, 3, normal_thresh, residual_thresh )
	--cull_map = cullPoints( normals, means, 3, math.pi/4, 40)
	--cull_map = cullPoints( normals, points, 3, math.pi/8, 20)
	--cull_map_cp = cull_map:clone()

	-- DEBUG test without cull map 

	cull_map = cull_map:byte():eq(0)
	--[[
	image.display(cull_map)
	image.display(errors:gt(0.3))
	]]--
	-- Unroll cull_mask
	cull_mask = torch.reshape( cull_map, 1,cull_map:nElement()):squeeze():byte()

	-- Draw cull_mask before I destroy it
	-- image.display( cull_mask )

	-- List of indices to sample from 	
	y_rng, x_rng = meshlist(1,points:size(3),1,points:size(2))
	x_rng = x_rng:squeeze()
	y_rng = y_rng:squeeze()

	-- Cull indices based on cull_mask 
	x_inds = x_rng[cull_mask]
	y_inds = y_rng[cull_mask]

	n_points = points:size(3)*points:size(2)

	-- For drawing, etc ... 
	cmap = image.colormap(n_planes)

	plane_inds = torch.LongTensor( n_planes, 2 )

	regions_rgb = torch.Tensor(points:size()):zero()
	regions_r = regions_rgb[1]
	regions_g = regions_rgb[2]
	regions_b = regions_rgb[3]

	samples_im = torch.Tensor(points:size(2), points:size(3)):zero()

	plane_sizes = torch.Tensor({0})
	plane_residuals = torch.Tensor({0})
	plane_normal_residuals = torch.Tensor({0})
	plane_curvatures = torch.Tensor({0})

	planes = {}

	finished = false

	plane_ind = 1
	while true do 
	-- for plane_ind = 1,n_planes do 

		-- Sample 25 times and choose the one with the smallest residual
		min_residual = nil
		max_size = nil
		min_curvature = nil
		for j = 1,1 do
			sample_ind = torch.rand(1):mul(x_inds:nElement()):long()+1

			--print("size inds: ", x_inds:nElement())

			-- Check if we have reached our coverage threshold 
			if x_inds:nElement() < (1 - coverage_thresh)*n_points then
				print( "coverage_thresh reached, exiting ")
				n_planes = plane_ind
				finished = true 
				break
			end

			x_rand = x_inds[sample_ind]:squeeze()
			y_rand = y_inds[sample_ind]:squeeze()

			inds = torch.LongTensor({x_rand,y_rand})
			print(points:size())
			print(inds)
			--plane_centroid, plane_equation, plane_eigenvalues, region_mask = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )
			pcent, peq, pegv, reg_msk = growPlane( inds, normals, means, second_moments, normal_thresh, residual_thresh )
			region_size = reg_msk:sum()
			plane_residual = pegv[{1}]/region_size
			curvature = pegv[{1}]/(pegv[{1}]+pegv[{2}]+pegv[{3}])

			--[[
			if not min_residual then
				min_residual = plane_residual
			end
			]]--
			if not max_size then
				max_size = region_size
			end
			if region_size >= max_size then 
			--if plane_residual <= min_residual then 
				plane_centroid = pcent
				plane_equation = peq
				plane_eigenvalues = pegv
				region_mask = reg_msk

				-- For minimum via RANSAC
				min_curvature = curvature
				max_size = region_size -- gota remember to do this!
				min_residual = plane_residual
			end
		end		

		if finished == true then
			break
		end

		print("plane_ind: ", plane_ind)
		print("coverage: ", x_inds:nElement()/n_points)

		--region_size = region_mask:sum()
		region_size = area[region_mask:byte()]:sum()
		if region_size > min_region_size then

			-- Update eigenvalues
			plane_residual = plane_eigenvalues[{{1},{}}]/region_size
			plane_residuals = torch.cat(plane_residuals, plane_residual,1)
			plane_curvatures = torch.cat(plane_curvatures, torch.Tensor({min_curvature}),1)
			--plane_normal_residuals = torch.cat(plane_normal_residuals, )

			plane_sizes = torch.cat(plane_sizes, torch.Tensor({region_size}))

			plane = Plane.new()
			plane.sample_inds = inds
			plane.centroid = plane_centroid
			plane.eqn = plane_equation
			plane.inlier_map = region_mask:byte()
			table.insert(planes, plane)

			plane_inds[{{plane_ind}, {}}] = inds

			regions_r[{{x_rand}, {y_rand}}] = 1.0
			regions_g[{{x_rand}, {y_rand}}] = 1.0
			regions_b[{{x_rand}, {y_rand}}] = 1.0

			-- Update plots
			region_mask = region_mask:byte()
			regions_r[region_mask] = cmap[{plane_ind,1}]
			regions_g[region_mask] = cmap[{plane_ind,2}]
			regions_b[region_mask] = cmap[{plane_ind,3}]

			samples_im[{{x_rand}, {y_rand}}] = cmap[{plane_ind,3}]

			plane_ind = plane_ind+1

		end

		if plane_ind > n_planes then
			break
		end

		-- Add region to cull_map and create new cull_mask 
		--print(cull_map)
		--print(region_mask)
		cull_map = cull_map:byte():add(region_mask:byte():eq(0)):gt(1)
		cull_mask = torch.reshape( cull_map, 1,cull_map:nElement()):squeeze():byte()
		x_inds = x_rng[cull_mask]
		y_inds = y_rng[cull_mask]


		collectgarbage()
	end

	-- Draw sample points 
	for i=1,n_planes-1 do 
		min_i = plane_inds[{{i},{1}}]:squeeze()
		min_j = plane_inds[{{i},{2}}]:squeeze()
		regions_r[{{min_i}, {min_j}}] = 1.0;
		regions_g[{{min_i}, {min_j}}] = 1.0;
		regions_b[{{min_i}, {min_j}}] = 1.0;
	end

	-- Output region masks 
	output = {}
	output.n_planes = n_planes
	output.planes = planes
	output.normal_thresh = normal_thresh
	output.residual_thresh = residual_thresh
	arc_io:dumpTorch( output, "region_masks",string.format("%.3d", scan_num) ) 


	-- Output images	
	arc_io:dumpImage( means, "scan_means", string.format("%.3d", scan_num) )
	arc_io:dumpImage( normals:add(1):mul(0.5), "scan_normals", string.format("%.3d", scan_num) )
	arc_io:dumpImage( regions_rgb, "scan_regions", string.format("%.3d", scan_num) )

	--arc_io:dumpImage( cull_map_cp, "cull_maps", string.format("%.3d", scan_num) )

	--[[
	image.display(cull_map)
	image.display(samples_im)
	]]--

	--[[
	gnuplot.figure(1)
	gnuplot.title("Plane Residuals")
	gnuplot.plot(plane_residuals)

	gnuplot.figure(2)
	gnuplot.title("Plane Sizes")
	gnuplot.plot(plane_sizes)

	gnuplot.figure(3)
	gnuplot.title("Plane Curvatures")
	gnuplot.plot(plane_curvatures)
	]]--


end	







