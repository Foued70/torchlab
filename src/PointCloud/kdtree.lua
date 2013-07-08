local kdnode = PointCloud.kdnode

local kdtree = Class()

function kdtree:__init(points)
	self.head = nil;
	if points then
		self:make_tree_from_points(points)
	end
end

function kdtree:make_tree_from_points(points)
	local count = points:size(1)
	if count > 0 then
		self.head = kdnode.new(1, points[1])
		for i = 2,count do
			self.head:add_child(i, points[i])
		end
	end
end

