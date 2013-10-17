SweepTree = Class()

SweepTreeNode = align_floors_endtoend.SweepTreeNode

function SweepTree:__init(root)
	 if not(root.__classname__ == SweepTreeNode.__classname__) then
	    error("Expected to be given sweep tree node as root!")
    end
	self.root = root
end

function SweepTreeNode:__write_keys()
	return {'root'}
end

function SweepTree:getRoot()
	return self.root
end

function SweepTree.traverse(root, func)
	func(root)
	if(root:isLeaf()) then
		return
	end
	for k,v in pairs(root:getChildren()) do
		SweepTree.traverse(v, func)
	end
end

function SweepTree.traverseBreadth(root, func)
	local nodes = {}
	local vals = {}
	SweepTree.traverseBreadthHelper(root, 0, nodes, vals)
	vals = torch.Tensor(vals)
	local temp,order = torch.sort(vals)
	for i=1,order:size(1) do
		func(nodes[order[i]], vals[order[i]])
	end
end

function SweepTree.traverseBreadthHelper(root, depth, nodes, vals)
	nodes[table.getn(nodes)+1] = root
	vals[table.getn(vals)+1] = depth
	if(root:isLeaf()) then
		return
	end
	for k,v in pairs(root:getChildren()) do
		SweepTree.traverseBreadthHelper(v, depth+1, nodes, vals)
	end
end

function SweepTree:getNode(name)
	local nodeToReturn
	local function getNode(node)
		if(node:getName() == name) then
			nodeToReturn = node
		end
	end
	SweepTree.traverse(self.root, getNode)
	return nodeToReturn
end
function SweepTree:getAllNodeNames()
	local names = {}
	local function accumulateNames(node)
		names[node:getName()] = node:getName()
	end
	SweepTree.traverse(self.root, accumulateNames)
	return names
end

function SweepTree:getAllNodes()
	local nodes = {}
	local function accumulateNodes(node)
		nodes[node:getName()] = node
	end
	SweepTree.traverse(self.root, accumulateNodes)
	return nodes
end

function SweepTree.swapParentAndChild(par, child)
	if not(par.__classname__ == SweepTreeNode.__classname__) or not(child.__classname__ == SweepTreeNode.__classname__) then
		error("wrong input type to swapParentAndChild")
	end
	local childNode = par:removeChild(child:getName())
	child:addChild(par:getName(), childNode.pair, not(childNode:getInverse()), par:getId(), par:getChildren())
	return child:getChild(par:getName())
end

function SweepTree:changeRoot(newRoot)
	print("changing root to" .. newRoot:getName())
	SweepTree.swapChain(newRoot, self.root)
	newRoot:setRoot()
	self.root = newRoot
end

function SweepTree.swapChain(curNode, oldRoot)
	if(curNode:getName() == oldRoot:getName()) then
		return
	end
	SweepTree.swapChain(curNode:getParent(), oldRoot)
	SweepTree.swapParentAndChild(curNode:getParent(), curNode)
end
