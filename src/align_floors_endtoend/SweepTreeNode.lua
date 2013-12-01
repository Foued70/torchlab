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

function SweepTreeNode:getParent()
	return self.from
end

function SweepTreeNode:setRoot()
	 self.from = nil
	 self.pair = nil
end
function SweepTreeNode:getName()
	return self.name
end
function SweepTreeNode:getSweep()
	return self.sweep
end
function SweepTreeNode:getPair()
	return self.pair
end

function SweepTreeNode:getInverse()
	return self.inverse
end

function SweepTreeNode:getId()
	return self.id
end
function SweepTreeNode:addChild(name, pair, inverse, id, children)
	if type(name)~= "string" or not(pair.__classname__ == align_floors_endtoend.SweepPair.__classname__) then
		error("wrong input type to sweeptreenode")
	end
	if(inverse) then
		if not(pair.sweep2:getName() == self.name)  then
			error("child must have sweep pair with one of the sweeps having name " .. self.name)
		end
		if not(pair.sweep1:getName() == name) then
			error("child must have sweep pair with one of the sweeps having name " .. name)
		end
	else
		if not(pair.sweep1:getName() == self.name ) then
			error("child must have sweep pair with one of the sweeps having name " .. self.name)
		end
		if not(pair.sweep2:getName() == name ) then
			error("child must have sweep pair with one of the sweeps having name " .. name)
		end
	end
	local sweep
	if(pair.sweep1:getName() == self.name)  then
		sweep = pair.sweep2
	else
		sweep = pair.sweep1
	end
	self.children[name] = SweepTreeNode.new(name,sweep, pair, inverse, self, id)
	if(children) then
		self.children[name].children = children
	end
end

function SweepTreeNode:getChild(name)
	if type(name)~= "string" then
		error("wrong type for getChild")
	end
	return self.children[name]
end
function SweepTreeNode:getChildren()
	return self.children
end

function SweepTreeNode:removeChild(name)
	if type(name)~= "string" then
		error("wrong type for removeChild")
	end
	local cur = self.children[name]
	self.children[name] = nil
	return cur
end

function SweepTreeNode:isRoot()
	return not(self.from)
end

function SweepTreeNode:isLeaf()
	return self:getChildrenLength() == 0
end

function SweepTreeNode:getChildrenLength()
  local count = 0
  for k,v in pairs(self:getChildren()) do count = count + 1 end
  return count
end

function SweepTreeNode:getChildI(i)
  local count = 0
  for k,v in pairs(self:getChildren()) do count = count + 1 if(count == i) then return v end end
  return count
end

function SweepTreeNode:getTransformationToRoot(icp)
	if(self:isRoot()) then
		return self.pair:getTransformation(true)
	else
		return self.from:getTransformationToRoot(icp)*self.pair:getTransformation(false,icp,self.inverse)
	end
end