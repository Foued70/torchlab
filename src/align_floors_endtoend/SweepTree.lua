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

function SweepTree:get_root()
	return self.root
end

function SweepTree.traverse(root, func)
	func(root)
	if(root:is_leaf()) then
		return
	end
	for k,v in pairs(root:get_children()) do
		SweepTree.traverse(v, func)
	end
end

function SweepTree.traverse_breadth(root, func)
	local nodes = {}
	local vals = {}
	SweepTree.traverseBreadthHelper(root, 0, nodes, vals)
	vals = torch.Tensor(vals)
	local temp,order = torch.sort(vals)
	for i=1,order:size(1) do
		func(nodes[order[i]], vals[order[i]])
	end
end

function SweepTree.traverse_breadth_helper(root, depth, nodes, vals)
	nodes[table.getn(nodes)+1] = root
	vals[table.getn(vals)+1] = depth
	if(root:is_leaf()) then
		return
	end
	for k,v in pairs(root:get_children()) do
		SweepTree.traverseBreadthHelper(v, depth+1, nodes, vals)
	end
end

function SweepTree:get_node(name)
	local nodeToReturn
	local function getNode(node)
		if(node:get_name() == name) then
			nodeToReturn = node
		end
	end
	SweepTree.traverse(self.root, getNode)
	return nodeToReturn
end
function SweepTree:get_all_node_names()
	local names = {}
	local function accumulateNames(node)
		names[node:get_name()] = node:get_name()
	end
	SweepTree.traverse(self.root, accumulateNames)
	return names
end

function SweepTree:get_all_nodes()
	local nodes = {}
	local function accumulateNodes(node)
		nodes[node:get_name()] = node
	end
	SweepTree.traverse(self.root, accumulateNodes)
	return nodes
end

function SweepTree.swap_parent_and_child(par, child)
	if not(par.__classname__ == SweepTreeNode.__classname__) or not(child.__classname__ == SweepTreeNode.__classname__) then
		error("wrong input type to swapParentAndChild")
	end
	local childNode = par:remove_child(child:get_name())
	child.children[par:get_name()] = par
	par.from = childNode
	par.inverse = not(childNode:get_inverse())
	par.pair = childNode.pair
	return child:get_child(par:get_name())
end

function SweepTree:change_root(newRoot)
	print("changing root to" .. newRoot:get_name())
	SweepTree.swap_chain(newRoot, self.root)
	newRoot:set_root()
	self.root = newRoot
end

function SweepTree.swap_chain(curNode, oldRoot)
	if(curNode:get_name() == oldRoot:get_name()) then
		return
	end
	SweepTree.swap_chain(curNode:get_parent(), oldRoot)
	SweepTree.swap_parent_and_child(curNode:get_parent(), curNode)
end
