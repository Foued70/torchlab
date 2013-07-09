local log = require '../util/log'
local kdnode = PointCloud.kdnode

local kdtree = Class()

function kdtree:__init(points)
	self.points = nil;
	self.head = nil;
	if points then
		self:make_tree_from_points(points)
	else
		log.error('points is nil')
	end
end

function kdtree:make_tree_from_points(points)
	if points then
		local count = points:size(1)
		self.points = points
		if count > 0 then
			self.head = kdnode.new(1, points[1])
			for i = 2,count do
				self.head:add_child(i, points[i])
			end
		end
	else
		log.error('points is nil')
	end
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
end

