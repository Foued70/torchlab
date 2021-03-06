SweepForest = Class()

SweepTreeNode = align_floors_endtoend.SweepTreeNode
SweepTree = align_floors_endtoend.SweepTree

function SweepForest:__init()
	self.tree_list = {}
end
function SweepForest:__write_keys()
	return {'tree_list'}
end

function SweepForest:add(sweep_tree)
	if not(sweep_tree.__classname__ == SweepTree.__classname__) then
	    error("Expected to be given sweep tree node as root!")
    end
    self.tree_list[sweep_tree:get_root():get_name()] = sweep_tree
end

function SweepForest:remove(sweep_tree)
	local success = false
	for k,v in pairs(self.tree_list) do
		if v:get_root():get_name() == (sweep_tree:get_root():get_name()) then --if not the tree we are trying to match to
			 self.tree_list[k] = nil
			success = true
			break;
		end
	end
	if not(success) then
		error("cannot remove, not in tree")
	end
    collectgarbage()
end

function SweepForest:get_trees()
	return self.tree_list
end
function SweepForest:find_tree_with_node(name)
	for k,v in pairs(self.tree_list) do
		if(v:get_node(name)) then
			return v
		end
	end
    collectgarbage()
end

function SweepForest:traverse_based_on_distance_from(nodeId, func, numfiles)
	local all_nodes = {}
	local node_distances = {}
	function getNodesAndDistances(v)
		all_nodes[table.getn(all_nodes)+1] = v
		local depth= torch.abs(nodeId-v:get_id())
		node_distances[table.getn(node_distances)+1] = math.min(depth%numfiles,(-depth)%numfiles)
	end
	for k,v in pairs(self.tree_list) do
		SweepTree.traverse(v:getRoot(), getNodesAndDistances)
	end
	node_distances = torch.Tensor(node_distances)
	local temp,order = torch.sort(node_distances)
	for i=1,order:size(1) do
		func(all_nodes[order[i]], node_distances[order[i]])
	end
    collectgarbage()
end

function SweepForest:get_number_forests()
	local count = 0
	for k,v in pairs(self.tree_list) do
		count = count+1
	end
	return count
end

function SweepForest:get_tree(i)
	local count = 1
	for k,v in pairs(self.tree_list) do
		if(count == i) then
			return v
		end
		count = count + 1
	end
end

--func should take three arguments, tree1 node, node in other tree, 
function SweepForest:traverse_based_on_distance_from_tree(tree1, func, numfiles)
	if(self:get_number_forests() == 1) then
		return
	end
	local all_nodes = {}
	local all_nodes_cur_tree = {}
	local node_distances = {}
	local tree = {}
	local t
	local nodes = tree1:get_all_nodes()
	function getNodesAndDistances(v)
		for k2,v2 in pairs(nodes) do
			all_nodes_cur_tree[table.getn(all_nodes_cur_tree)+1] = v2
			all_nodes[table.getn(all_nodes)+1] = v
			depth = torch.abs(v2:get_id()-v:get_id())
			node_distances[table.getn(node_distances)+1] = math.min(depth%numfiles,(-depth)%numfiles)
			tree[table.getn(tree)+1] = t
		end
	end
	for k,v in pairs(self.tree_list) do
		if not(v:get_node(tree1:get_root():get_name())) then --if not the tree we are trying to match to
			t = v
			SweepTree.traverse(v:get_root(), getNodesAndDistances) --get distance between all elements of tree1 and all elements of all other trees
		end
	end
	node_distances = torch.Tensor(node_distances)
	local temp,order = torch.sort(node_distances)
	--traverse in order of distance
	for i=1,order:size(1) do
		func(all_nodes_cur_tree[order[i]], all_nodes[order[i]], node_distances[order[i]], tree[order[i]])
	end
    collectgarbage()
end
