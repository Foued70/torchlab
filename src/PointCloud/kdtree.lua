local log = require '../util/log'
local kdnode = PointCloud.kdnode

local kdtree = Class()

function kdtree:__init(points, numaxis)
	self.points = points;
	self.head = nil;
	self.numaxis = numaxis
	if points then
		--self:make_tree_from_points()
		self:make_median_tree_from_points()
	else
		log.error('points is nil')
	end
end

function kdtree:make_median_tree_from_points()
	self.head = nil
	local points_table = {}
	self:make_left_median_partial_tree(nil, self.points, 1)
end

function kdtree:make_left_median_partial_tree(parent_node, points, axis)

	if points and points:size(1)>0 then
		
		local sz = points:size(1)
		
		local sorted_vals,sorted_indexes = points:select(2,axis):sort()
		local sorted_pts = points[sorted_indexes]
		
		local med = math.ceil(sz/2)
		local med_ind = sorted_indexes[med]
		
		if parent_node then
			parent_node:add_left_child(med_ind,points[med_ind])
		else
			self.head = kdnode.new(med_ind, points[med_ind], nil, self.numaxis)
		end
		
		local left_points = nil
		local right_points = nil
			
		if med-1 > 0 then
			left_points = sorted_pts:sub(1,med-1)
		end
		if med < sz then
			right_points = sorted_pts:sub(med+1,sz)
		end
		
		axis = axis % self.numaxis + 1
		
		if parent_node then
			self:make_left_median_partial_tree(parent_node.left_child, left_points, axis)
			self:make_right_median_partial_tree(parent_node.left_child, right_points, axis)
		else
			self:make_left_median_partial_tree(self.head, left_points, axis)
			self:make_right_median_partial_tree(self.head, right_points, axis)
		end
	end
	collectgarbage()
end

function kdtree:make_right_median_partial_tree(parent_node, points, axis)
	
	if points and points:size(1)>0 then
	
		local sz = points:size(1)
		
		local sorted_vals,sorted_indexes = points:select(2,axis):sort()
		local sorted_pts = points[sorted_indexes]
		
		local med = math.ceil(sz/2)
		local med_ind = sorted_indexes[med]
		
		if parent_node then
			parent_node:add_right_child(med_ind,points[med_ind])
		else
			self.head = kdnode.new(med_ind, points[med_ind], nil, self.numaxis)
		end
		
		local left_points = nil
		local right_points = nil
			
		if med-1 > 0 then
			left_points = sorted_pts:sub(1,med-1)
		end
		if med < sz then
			right_points = sorted_pts:sub(med+1,sz)
		end
		
		axis = axis % self.numaxis + 1
		
		if parent_node then
			self:make_left_median_partial_tree(parent_node.right_child, left_points, axis)
			self:make_right_median_partial_tree(parent_node.right_child, right_points, axis)
		else
			self:make_left_median_partial_tree(self.head, left_points, axis)
			self:make_right_median_partial_tree(self.head, right_points, axis)
		end
	end
	collectgarbage()
end

function kdtree:make_tree_from_points()
	if self.points then
		local count = self.points:size(1)
		if count > 0 then
			self.head = kdnode.new(1, self.points[1], nil, self.numaxis)
			for i = 2,count do
				self.head:add_child(i, self.points[i])
			end
		end
	else
		log.error('points is nil')
	end
	collectgarbage()
end

function kdtree:get_nn(xyz, exclusion)
	local bestnb, bestdst, bestind = self.head:NNS(xyz, nil, math.huge, -1, exclusion)
	return bestind 
end

function kdtree:get_nearest_neighbors(xyz, num)
	if num > 0 and self.points then
		local ret = torch.Tensor(num)
		if num > 1 then
			local exclusion = torch.zeros(self.points:size(1))
			for i=1,num do
				local ind = self:get_nn(xyz, exclusion)
				ret[i] = ind
				exclusion[ind] = 1
			end
		else
			ret[1]=self:get_nn(xyz,nil)
		end
		return ret
	else
		error('either points is nil or num is <= 0')
	end
	collectgarbage()
end

function kdtree:get_neighbors_in_radius(xyz,rad)
	return torch.Tensor(self.head:neighbors_in_radius(xyz,rad,{}))
end
