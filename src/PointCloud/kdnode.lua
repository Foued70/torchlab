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
		error('no values given')
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
		error('no values given')
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
	