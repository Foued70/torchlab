SweepTreeNode = Class()

--for root keep parent nil!
function SweepTreeNode:__init(name, sweep, pair, inverse, from, id)
	self.name = name
	self.sweep = sweep
	self.pair = pair
	self.from = from
	self.inverse = inverse
	self.children = {}
	self.id = id
end

function SweepTreeNode:__write_keys()
	return {'name', 'sweep', 'pair', 'from', 'inverse', 'children', 'id'}
end

function SweepTreeNode:get_parent()
	return self.from
end

function SweepTreeNode:set_root()
	 self.from = nil
	 self.pair = nil
end
function SweepTreeNode:get_name()
	return self.name
end
function SweepTreeNode:get_sweep()
	return self.sweep
end
function SweepTreeNode:get_pair()
	return self.pair
end

function SweepTreeNode:get_inverse()
	return self.inverse
end

function SweepTreeNode:get_id()
	return self.id
end
function SweepTreeNode:add_child(name, pair, inverse, id, children)
	if type(name)~= "string" or not(pair.__classname__ == align_floors_endtoend.SweepPair.__classname__) then
		error("wrong input type to sweeptreenode")
	end
	if(inverse) then
		if not(pair:get_sweep2():get_name() == self.name)  then
			error("child must have sweep pair with one of the sweeps having name " .. self.name)
		end
		if not(pair:get_sweep1():get_name() == name) then
			error("child must have sweep pair with one of the sweeps having name " .. name)
		end
	else
		if not(pair:get_sweep1():get_name() == self.name ) then
			error("child must have sweep pair with one of the sweeps having name " .. self.name)
		end
		if not(pair:get_sweep2():get_name() == name ) then
			error("child must have sweep pair with one of the sweeps having name " .. name)
		end
	end
	local sweep
	if(pair:get_sweep1():get_name() == self.name)  then
		sweep = pair:get_sweep2()
	else
		sweep = pair:get_sweep1()
	end
	self.children[name] = SweepTreeNode.new(name,sweep, pair, inverse, self, id)
	if(children) then
		self.children[name].children = children
	end
end

function SweepTreeNode:get_child(name)
	if type(name)~= "string" then
		error("wrong type for get_child")
	end
	return self.children[name]
end
function SweepTreeNode:get_children()
	return self.children
end

function SweepTreeNode:remove_child(name)
	if type(name)~= "string" then
		error("wrong type for removeChild")
	end
	local cur = self.children[name]
	self.children[name] = nil
	return cur
end

function SweepTreeNode:is_root()
	return not(self.from)
end

function SweepTreeNode:is_leaf()
	return self:get_children_length() == 0
end

function SweepTreeNode:get_children_length()
  local count = 0
  for k,v in pairs(self:get_children()) do count = count + 1 end
  return count
end

function SweepTreeNode:get_child_i(i)
  local count = 0
  for k,v in pairs(self:get_children()) do count = count + 1 if(count == i) then return v end end
  return count
end

function SweepTreeNode:get_transformation_to_root(icp)
	if(self:is_root()) then
		return self.pair:get_transformation(true)
	else
		return self.from:get_transformation_to_root(icp)*self.pair:get_transformation(false,icp,self.inverse)
	end
end