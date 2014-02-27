Class()
local path = require "path"

function get_formulated_problem(scan, tree)
	local ncam = scan.numfiles
	local scores = align_floors_endtoend.Scan.get_distance_scores(tree)
	scan:calculate_alignments(scores)
	scores = align_floors_endtoend.BestGraph.get_good_alignments_and_scores(tree)

	local cameras = torch.zeros(ncam, 7)
	local camera_H = {}
	for i =1,ncam do
		local node =  t:get_node(scan:get_ith_sweep(i,true):get_name())
		local transf = torch.inverse(node:get_transformation_to_root())
		local quat = geom.quaternion.from_rotation_matrix(transf)
		local rot = geom.quaternion.to_rotation_matrix(quat)
		local final = torch.eye(4)
		final[{{1,3},{1,3}}] = rot
		final[{{1,3},{4}}] = transf[{{1,3},{4}}]
		
		camera_H[i] = torch.inverse(final)

		local quat_new = torch.zeros(quat:size())
		quat_new[1]=quat[4]
		quat_new[2]=quat[1]
		quat_new[3]=quat[2]
		quat_new[4]=quat[3]
		
		cameras[i] = torch.cat(quat_new, transf[{{1,3},{4}}])
	end

	local npts = 0
	local nobs = 0
	local nparams = 0
	local nparams_old = 0
	local camera_index_table = {}
	local pts_index_table = {}
	local points_table = {}
	local observations_table = {}
	local num_iterations = 1
	for i =1,scores:size(1) do
		for j=i+1,scores:size(2) do
			if(scores[i][j] < math.huge) then
				print(i,j)
				local pair=scan:get_sweeppair(i,j)
				local node =  t:get_node(pair:get_sweep1():get_name())

				local H = pair:get_transformation()
				local pc1 = pair:get_sweep1():get_pc()
				local pc2=pair:get_sweep2():get_pc()
				local t1,t2 = align_floors_endtoend.ClosestPointFinder.findClosePoints(pc1, pc2,H)
				if(t1:size(1) > 0) then
					npts = npts+t1:size(1)
					nobs=nobs+t1:size(1)*2
					nparams_old = nparams
					nparams = nparams_old+t1:size(1)
					camera_index_table[num_iterations] = torch.cat(torch.ones(t1:size(1))*i, torch.ones(t1:size(1))*(j))
					pts_index_table[num_iterations] = torch.cat(torch.range(nparams_old+1, nparams), torch.range(nparams_old+1, nparams))
					local H_root_1 = node:get_transformation_to_root() --sweep1 to root, --camera_H[i]
					local good_pts = align_floors_endtoend.ClosestPointFinder.getFromLong(pc1:get_xyz_map(),t1)
					local global_points =  (H_root_1*torch.cat(good_pts,torch.ones(good_pts:size(1),1)):t()):t():sub(1,-1,1,3)
					points_table[num_iterations] = global_points --get global points and put them here

					local pc1_points = align_floors_endtoend.ClosestPointFinder.getFromLong(pc1:get_xyz_map(),t1)
					local pc2_points = align_floors_endtoend.ClosestPointFinder.getFromLong(pc2:get_xyz_map(),t2)

					observations_table[num_iterations] = torch.cat(pc1_points,pc2_points,1) -- get first set and seconds set local coordinates
					num_iterations = num_iterations+1
				end
			end
		end
	end



	local cam_index = torch.zeros(nobs)
	local pts_index = torch.zeros(nobs)
	local points = torch.zeros(nparams,3)
	local observations = torch.zeros(nobs,3)
	local start_obs = 0
	local start_obs_old = 0
	local start_params = 0
	local start_params_old = 0
	for i=1,num_iterations-1 do
		start_obs_old = start_obs+1
		start_obs = start_obs_old+observations_table[i]:size(1)-1

		start_params_old = start_params+1
		start_params = start_params_old+points_table[i]:size(1)-1
		
		cam_index[{{start_obs_old, start_obs}}] = camera_index_table[i] 
		pts_index[{{start_obs_old, start_obs}}] = pts_index_table[i] 
		observations[{{start_obs_old, start_obs},{1,3}}] = observations_table[i] 
		points[{{start_params_old, start_params},{1,3}}] = points_table[i] 
	end

	cam_index = cam_index-1
	pts_index = pts_index-1
	parameters = torch.cat(cameras:reshape(7*ncam),points:reshape(npts*3))
	cameras = parameters:narrow(1,1,7*ncam):reshape(ncam,7)
	points  = parameters:narrow(1,7*ncam+1,npts*3):resize(npts,3)


	return { ncam         = ncam,
	         npts         = npts,
	         nobs         = nobs,
	         cam_index    = cam_index:int(),
	         pts_index    = pts_index:int(),
	         parameters   = parameters,
	         observations = observations,
	         cameras      = cameras,
	         points       = points }
end


function get_formulated_problem_without_correspondences(scan,tree)
	local ncam = scan.numfiles
	local scores = align_floors_endtoend.Scan.get_distance_scores(tree)
	scan:calculate_alignments(scores)
	scores = align_floors_endtoend.BestGraph.get_good_alignments_and_scores(tree)

	local cameras = torch.zeros(ncam, 7)
	local points
	local start_index = torch.zeros(ncam)
	local end_index = torch.zeros(ncam)
	local nobs = 0
	for i =1,ncam do
		--node =  t:get_node(scan:get_ith_sweep(i,true):get_name())
		local quat = torch.load(path.join("good_data", "quat_transformations", scan:get_ith_sweep(i,true):get_name() .. ".data"))
		cameras[i] = quat--torch.inverse(transf):sub(1,3,1,4)
		--torch.inverse(node:get_transformation_to_root()):sub(1,3,1,4)
		if(i==1 ) then
			points = scan:get_ith_sweep(i,true):get_pc():get_xyz_list()
		else
			points = torch.cat(points, scan:get_ith_sweep(i,true):get_pc():get_xyz_list(), 1)
		end
		start_index[i] = nobs+1
		nobs = points:size(1)
		end_index[i] = nobs
	end
	bal = {cameras:reshape(7*ncam), start_index, end_index, points, torch.lt(scores,math.huge)}
	return bal
end

function get_formulation_for_rigid_transformation(scan,tree, i)
	local node =  t:get_node(scan:get_ith_sweep(i,true):get_name())
	local H = node:get_transformation_to_root()
	local camera = solve.ceres.get_our_quat_from_ceres(H)
	local pc1 = scan:get_ith_sweep(i,true):get_pc()
	local original_points = pc1:get_xyz_list()
	local transformed_points = pc1:get_xyz_list_transformed(nil,H)
	local parameters = camera:reshape(7)
	return {          parameters      = parameters,
	         original_points       = original_points, 
	     	 transformed_points = transformed_points}
end
