local log = require '../util/log'
local kdnode = Class()

function kdnode:__init(index, xyz, parent)	
	if index and xyz then
		self.index = index
		self.xyz = xyz
		self.left_child = nil
		self.right_child = nil
		self.parent = parent
		self.axis = 1
		self.height = 1
		self.descendents = 0
		if self.parent and self.parent.axis then
			if self.parent.axis <= 3 and self.parent.axis > 0 then
				self.axis = self.parent.axis % 3 + 1
			end
		end
	else
		log.error('no values given')
	end	
end
	
function kdnode:add_child(index, xyz)
	if index and xyz then
		if xyz[self.axis] < self.xyz[self.axis] then
			add_left_child(self, index, xyz)
		else
			add_right_child(self, index, xyz)
		end
		self.descendents = self.descendents + 1
	else
		log.error('no values given')
	end	
end

function kdnode:add_left_child(index, xyz)
	if self.left_child == nil then
		self.left_child = kdnode.new(index, xyz, self)
	else
		self.left_child:add_child(index, xyz)
	end
	self.height = math.max(self.height, self.left_child.height+1)
end

function kdnode:add_right_child(index, xyz)
	if self.right_child == nil then
		self.right_child = kdnode.new(index, xyz, self)
	else
		self.right_child:add_child(index, xyz)
	end
	self.height = math.max(self.height, self.right_child.height+1)
end

function kdnode:NNS(pt, bestnb, bestdist, bestind, exclusion)
	local dst = pt:dist(self.xyz)
	if dst < bestdist then
		if (not exclusion) or exclusion[self.index] == 0 then
			bestdist = dst
			bestind = self.index
			bestnb = self.xyz
			--log.info('new best distance '..bestdist..' is not excluded: '..bestind)
		end
	end
	if self.left_child or self.right_child then
		if pt[self.axis] < self.xyz[self.axis] then
			if self.left_child and self.left_child and pt[self.axis] - bestdist <= self.xyz[self.axis] then
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
	return bestnb, bestdist, bestind
end
	