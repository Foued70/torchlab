--[[
	Plane merging for single and multiple scans
]]--

ArcIO = data.ArcIO
angle_between = geom.util.angle_between

local MergePlanes = Class()

default_normal_thresh = math.pi/8
default_residual_thresh = 15

default_input_io = 'extract_planes_ransac'

function MergePlanes:__init( job_id, input_io, normal_thresh, residual_thresh ) 
	self.job_id = job_id	

	self.normal_thresh = normal_thresh or default_normal_thresh
	self.residual_thresh = residual_thresh or default_residual_thresh	
	self.input_io = input_io or default_input_io

	self.input_io = ArcIO.new( job_id, self.input_io )	
end

function MergePlanes:preparePlanes()
	error('preparePlanes unimplemented')
end

function MergePlanes:overlapCull()
	error('overlapCull unimplemented')
end

function MergePlanes:fitMergedPlanes()
	error('fitMergedPlanes not implemented')
end

function MergePlanes:outputPlanePoints()
	error('outputPlanePoints not implemented')
end

function MergePlanes:matchPlanes()
	-- Prepare planes for matching 
	--self:preparePlanes()

	tmr = torch.Timer()
	threshold_time = 0 
	overlap_time = 0 	

	-- Compute overlaps between all planes 
	print("Computing overlaps between all planes")
	local overlap = 0
	local smaller = 0
	tt0 = tmr:time()['real']	
	match_list = {}
	for i=1, #self.planes do
		print(string.format("computing overlaps for plane: %d",i))
		for j=i, #self.planes do
			if j ~= i then
				th0 = tmr:time()['real']
				angle = angle_between( self.planes[i].eqn:sub(1,3) , self.planes[j].eqn:sub(1,3) )				
				residual = math.abs(torch.dot( self.planes[i].eqn:sub(1,3), torch.add(self.planes[i].centroid, -self.planes[j].centroid) ))				
				threshold_time = threshold_time + (tmr:time()['real']-th0)
				if angle < self.normal_thresh and residual < self.residual_thresh then										
					-- Fill in adjecency matrix 					
					if match_list[i] == nil then
						match_list[i] = {}
					end
					table.insert(match_list[i], j)										
				end				
				collectgarbage()
			end
		end
	end
	print("total_time: ", tmr:time()['real'] - tt0)	
	print("threshold_time: ", threshold_time)
	print("overlap_time: ", overlap_time)	

	--match_matrix, match_list_culled = self:overlapCull( match_list )
	return match_list
end

function MergePlanes:recursiveMerge( ind, ind_set, match_matrix )
	local h_set = self.plane_rng[match_matrix[{ind,{}}]]
	local v_set = self.plane_rng[match_matrix[{{},ind}]]	
	local cur_ind_set = self:unique_set(h_set, v_set)	
	-- Only explore where what we haven't already seen
	local ind_set = self:unique_set( ind_set, torch.Tensor({ind}) )	
	local cur_ind_set = self:subtract_sets(cur_ind_set, ind_set)
	for j=1, cur_ind_set:nElement() do		
		ind_set = self:recursiveMerge(cur_ind_set[j], ind_set, match_matrix)
	end
	-- Don't forget to add our ind 
	return self:unique_set(ind_set, cur_ind_set)		
end

function MergePlanes:computeMergeSets( match_matrix )
	if not match_matrix then
		error('match_matrix is nil, need to return match_matrix after overlapCull')
	end

	self.plane_rng = torch.range(1,#self.planes)	

	print("Computing merge sets")
	local to_merge
	local ind_set
	local merged = nil
	local rm_mask = torch.ByteTensor(self.plane_rng:nElement()):fill(1)
	local plane_sets = {}
	while true do
		if merged == nil then
			to_merge = self.plane_rng:clone():long()
		else
			-- TODO: replace with subtract 
			rm_mask[merged:long()] = 0
			to_merge = self.plane_rng[rm_mask]
			if to_merge:nElement() == 0 then 
				break
			end
		end		
		ind_set = self:recursiveMerge(to_merge[1],torch.Tensor({}), match_matrix)
		if merged == nil then
			merged = ind_set
		else
			merged = self:unique_set( merged, ind_set )
		end		
		table.insert(plane_sets, ind_set)
		collectgarbage()
	end	
	self.cmap = image.colormap(#plane_sets)
	return plane_sets			
end

-- Utility set functions 
function MergePlanes:unique_set( s0, s1 )
	-- Check if s0 or s1 are empty if so return the other, or empty
	if s0:dim() == 0 and s1:dim() == 0 then
		return torch.Tensor({})
	end
	if s0:dim() == 0 then
		return s1
	end
	if s1:dim() == 0 then
		return s0
	end
	local rng = torch.range(1,math.max(s0:max(),s1:max()))
	local mask = torch.ByteTensor(rng:nElement()):zero()
	mask[s0:cat(s1):long()] = 1
	return rng[mask]
end

function MergePlanes:subtract_sets( s0, s1 )
	if s0:dim() == 0 then
		return s0 
	end
	if s1:dim() == 0 then
		return s0 
	end
	local mx = math.max(s0:max(), s1:max())	
	local mask = torch.ByteTensor(mx):zero()
	mask[s0:long()] = 1
	mask[s1:long()] = 0 
	return torch.range(1,mx)[mask]
end

function MergePlanes:printPlaneSets( plane_sets )
	-- pretty printing 
	print("Number of merged plane sets: ", #plane_sets)	
	str = ""
	for k,v in pairs(plane_sets) do		
		str = str .. string.format("%d:", k)
		for j=1, v:nElement() do 
			str = str .. string.format(" %d ", v[j])
		end
		str = str .. string.format("\n")
	end
	print(str)
end
