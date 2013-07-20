local log = require '../util/log'
local kdnode = Class()

function kdnode:__init(index, xyz, parent, numaxis)	
	if index and xyz then
		self.index = index
		self.xyz = xyz
		self.left_child = nil
		self.right_child = nil
		self.parent = parent
		self.axis = 1
		self.height = 1
		self.descendents = 0
		self.numaxis = numaxis
		if self.parent and self.parent.axis then
			if self.parent.axis <= 3 and self.parent.axis > 0 then
				self.axis = self.parent.axis % numaxis + 1
			end
			self:increase_ancestors_height_count()
			self:increase_ancestors_descendents_count()
		end
	else
		log.error('no values given')
	end	
end

function kdnode:increase_ancestors_height_count()
	if self.parent then
		if self.parent.height < self.height + 1 then
			self.parent.height = math.max(self.parent.height, self.height + 1)
			self.parent:increase_ancestors_height_count()
		end
	end
end

function kdnode:increase_ancestors_descendents_count()
	if self.parent then
		self.parent.descendents = self.parent.descendents+1
		self.parent:increase_ancestors_descendents_count()
	end
end

function kdnode:add_child(index, xyz)
	if index and xyz then
		if xyz[self.axis] < self.xyz[self.axis] then
			add_left_child(self, index, xyz)
		elseif xyz[self.axis] > self.xyz[self.axis] then
			add_right_child(self, index, xyz)
		elseif not self.left_child then
			add_left_child(self, index, xyz)
		elseif not self.right_child then
			add_right_child(self, index, xyz)
		elseif self.left_child.descendents <= self.right_child.descendents then
			add_left_child(self, index, xyz)
		else
			add_right_child(self, index, xyz)
		end
	else
		log.error('no values given')
	end
end

function kdnode:add_left_child(index, xyz)
	if self.left_child == nil then
		self.left_child = kdnode.new(index, xyz, self, self.numaxis)
	else
		self.left_child:add_child(index, xyz)
	end
end

function kdnode:add_right_child(index, xyz)
	if self.right_child == nil then
		self.right_child = kdnode.new(index, xyz, self, self.numaxis)
	else
		self.right_child:add_child(index, xyz)
	end
end

function kdnode:NNS(pt, bestnb, bestdist, bestind, exclusion)
	local dst = pt:dist(self.xyz)
	if dst < bestdist then
		if (not exclusion) or exclusion[self.index] == 0 then
			bestdist = dst
			bestind = self.index
			bestnb = self.xyz
		end
	end
	if self.left_child or self.right_child then
		if pt[self.axis] < self.xyz[self.axis] then
			if self.left_child and pt[self.axis] - bestdist <= self.xyz[self.axis] then
				bestnb, bestdist, bestind = self.left_child:NNS(pt, bestnb, bestdist, bestind, exclusion)
			end
			if self.right_child and pt[self.axis] + bestdist > self.xyz[self.axis] then
				bestnb, bestdist, bestind = self.right_child:NNS(pt, bestnb, bestdist, bestind, exclusion)
			end
		else
			if self.right_child and pt[self.axis] + bestdist > self.xyz[self.axis] then
				bestnb, bestdist, bestind = self.right_child:NNS(pt, bestnb, bestdist, bestind, exclusion)
			end
			if self.left_child and pt[self.axis] - bestdist <= self.xyz[self.axis] then
				bestnb, bestdist, bestind = self.left_child:NNS(pt, bestnb, bestdist, bestind, exclusion)
			end
		end
	end
	collectgarbage()
	return bestnb, bestdist, bestind
end

function kdnode:neighbors_in_radius(pt, radius, ind_table)
	local dst = pt:dist(self.xyz)
	if dst < radius then
		table.insert(ind_table, self.index)
	end
	if self.left_child or self.right_child then
		if pt[self.axis] < self.xyz[self.axis] then
			if self.left_child and pt[self.axis] - radius <= self.xyz[self.axis] then
				ind_table = self.left_child:neighbors_in_radius(pt, radius, ind_table)
			end
			if self.right_child and pt[self.axis] + radius > self.xyz[self.axis] then
				ind_table = self.right_child:neighbors_in_radius(pt, radius, ind_table)
			end
		else
			if self.right_child and pt[self.axis] + radius > self.xyz[self.axis] then
				ind_table = self.right_child:neighbors_in_radius(pt, radius, ind_table)
			end
			if self.left_child and pt[self.axis] - radius <= self.xyz[self.axis] then
				ind_table = self.left_child:neighbors_in_radius(pt, radius, ind_table)
			end
		end
	end
	collectgarbage()
	return ind_table
end
	