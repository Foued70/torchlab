local path = require 'path'
local pcl = pointcloud.pointcloud

local Sweep = align_floors_endtoend.Sweep
local SweepPair = align_floors_endtoend.SweepPair
local SweepTreeNode = align_floors_endtoend.SweepTreeNode
local SweepTree = align_floors_endtoend.SweepTree
local SweepForest = align_floors_endtoend.SweepForest 
 
Scan = Class()
--arcs = data.ArcIO.new(data.ArcSpecs.job_id, data.ArcSpecs.work_id)
--align_floors_endtoend.Scan.new(arcs)

function Scan:__init(arc, complete_loop, maxDepth)
    self.arc = arc
    self.complete_loop = complete_loop or false    
    self.numfiles = arc:getNumberScans()
    self.scores = {}
    self.first_sweep = first_sweep or 1
    self.maxDepth = 10 or maxDepth -- a check of diff of ids <= depth is done to determine which pairs will be attempted, so min depth should be 1 if want to check just adjacent pairs
    self.complete_loop_num = self.num_files
    if(not(self.complete_loop)) then
        self.complete_loop_num  = 10^10
    end
end

function Scan:organize_in_trees()
    local s_11 = self:get_sweeppair(1,1)
    local s_1 = self:get_ith_sweep(1, true)
    local tree_root = SweepTreeNode.new(s_11:get_sweep1():get_name(), s_1, s_11, false, nil, 1)
    local tree = SweepTree.new(tree_root)
    local forest = SweepForest.new()
    forest:add(tree)
    local i_list = torch.range(1,self.numfiles-1) --self.numfiles-1
    local j_list = i_list+1
    for k = 1,i_list:size(1) do
        local i = i_list[k]
        local j = j_list[k]
        
        local prevName = self:get_ith_sweep(i, true):get_name()
        local curName = self:get_ith_sweep(j, true):get_name()
        local curSweep = self:get_ith_sweep(j, true)

        local tree_containing_previous = forest:find_tree_with_node(prevName)      
        local node = tree_containing_previous:get_node(prevName)
        local success, pair = self:attempt_to_align_pair(self:get_sweeppair(i,j,true))
        if(success) then
            node:add_child(curName, pair, false, j)
        else
            local s_jj = self:get_sweeppair(j,j)
            local s_j = self:get_ith_sweep(j, true)
            print("creating new tree since couldn't match to any existing for" .. s_jj:get_sweep1():get_name())
            
            local new_root = SweepTreeNode.new(s_jj:get_sweep1():get_name(), s_j, s_jj, false, nil, j)
            local new_tree = SweepTree.new(new_root)
            forest:add(new_tree)
        end
        collectgarbage()
    end
    local maxDepth = self.maxDepth
    for i=2, maxDepth do
        self.maxDepth = i
        for k,v in pairs(forest:get_trees()) do
            while(self:merge_forest_with_tree(forest, v)) do end --while this tree can be merged with others...
        end
    end
    return forest
end

--i<j
function Scan:manually_merge_pair(forest, i, j)
    local t1 = forest:findTreeWithNode(self:get_ith_sweep(i,true):get_name())
    local t2 = forest:findTreeWithNode(self:get_ith_sweep(j,true):get_name())
    local smallerNode = t1:get_node(self:get_ith_sweep(i,true):get_name())
    local largerNode = t2:get_node(self:get_ith_sweep(j,true):get_name())
    local inverse = false
    local s_ij = self:get_sweeppair(i,j,true)
    local success, pair = self:attempt_to_align_pair(s_ij)
    if(success) then
        t2:change_root(largerNode)
        smallerNode.children[largerNode:get_name()] = largerNode
        largerNode.from = smallerNode
        largerNode.inverse = inverse
        largerNode.pair = pair
        forest:remove(t2)
    end
end

function Scan:merge_forest_with_tree(forest, tree_orig)
    local foundMatch = false
    local function checkWithOtherNodes(curtreenode, node,depth, tree)
        if not(foundMatch) and depth<=self.maxDepth then
            local smallerNode = curtreenode
            local largerNode = node
            local inverse = false
            if(node:get_id() < curtreenode:get_id()) then
                smallerNode = node
                largerNode = curtreenode
                inverse = true
            end
            local s_ij = SweepPair.new_or_load(self.base_dir, smallerNode.sweep, largerNode.sweep)
            local success, pair = self:attempt_to_align_pair(s_ij)
            if(success) then
                print("merging forests based on nodes" .. curtreenode:get_name() .. " and " .. node:get_name())
                foundMatch = true
                winningTree = tree
                winningTree:change_root(node)
                
                curtreenode.children[node:get_name()] = node
                node.from = curtreenode
                node.inverse = inverse
                node.pair = pair
            end
        end
    end
    forest:traverse_based_on_distance_from_tree(tree_orig, checkWithOtherNodes, self.complete_loop_num)
    if(foundMatch) then
        forest:remove(winningTree)
    end
    return foundMatch
end

function Scan:align_with_forest(forest, curnode, i)
    local foundMatch = false
    local goodPair
    local nodeToMatch
    --for fun choose depth less than 20 as how far we go to check
    local function checkWithOtherNodes(node,depth)
        if not(foundMatch) and depth<= self.maxDepth then
            print("matching " .. curnode:get_name() .. " with " .. node:get_name() .. " at distance " .. depth)
            local s_ij = SweepPair.new_or_load(self.base_dir, node.sweep,curnode)
            local success, pair = self:attempt_to_align_pair(s_ij)
            if(success) then
                foundMatch = true
                goodPair = pair
                nodeToMatch = node
            end
        end
    end
    forest:traverse_based_on_distance_from(i, checkWithOtherNodes, self.complete_loop_num)
    if(foundMatch) then
        nodeToMatch:add_child(goodPair:get_sweep2():get_name(), goodPair, false, i)
    end
    return foundMatch
end

function Scan:attempt_to_align_pair(s_ij)
    print('Attempting to align ' .. s_ij:get_sweep1():get_name() .. " and " .. s_ij:get_sweep2():get_name())
    collectgarbage()

    local success    
    s_ij:set_best_picked_diff_transformation()
    local score =  s_ij:get_3d_validation_score(true, 10)
    if(score[1]<.7 or score[4]<.1) then
            print("BAD ALIGNMENT---------" .. s_ij:get_sweep1():get_name() .. " and " .. s_ij:get_sweep2():get_name())
        return false, s_ij
    end
    s_ij:get_icp()
    score =  s_ij:get_3d_validation_score(false, 10)
    if (score[1]<.85 or score[4]<.2) then 
        print("BAD ALIGNMENT POST ICP---------" .. s_ij:get_sweep1():get_name() .. " and " .. s_ij:get_sweep2():get_name())
        return false, s_ij
    else
        print("GOOD ALIGNMENT---------" .. s_ij:get_sweep1():get_name() .. " and " .. s_ij:get_sweep2():get_name())
        return true, s_ij
    end
end

function Scan:attempt_to_align(i, j)
    return self:attempt_to_align_pair(self:get_sweeppair(i,j))
end

function Scan:get_ith_sweep(i)
    return Sweep.new_or_load(self.arc,i)
end

--always return smaller/larger
function Scan:get_ith_sweeppair(i)
    return SweepPair.new_or_load(self.arc, i, i+1) 
end

function Scan:get_sweeppair(i, j)
   return SweepPair.new_or_load(self.arc, i, j)  
end

function Scan:save_global(tree, save_dir)
    local nodes = tree:get_all_nodes()
    local io = require 'io'
    local dir =  path.join(save_dir, tree:get_root():get_name())
    util.fs.mkdir_p(dir)
    for k,v in pairs(nodes) do
        local transf = v:get_transformation_to_root()
        v:get_sweep():get_pc():save_to_xyz_file(path.join(dir,v:get_sweep():get_name() .. ".xyz"),transf)
        print(path.join(dir,v:get_sweep():get_name() .. "_transf.dat"))
        torch.save(path.join(dir,v:get_sweep():get_name() .. "_transf.dat"), transf)
       local file = io.open(path.join(dir,v:get_sweep():get_name() .. "_transf.txt"), 'w')
       transf = transf:reshape(16)
        file:write(''..transf[1]..' '..transf[2]..' '..transf[3]..' '..transf[4]..'\n'..
            transf[5]..' '..transf[6]..' '..transf[7]..' '..transf[8]..'\n'..
            transf[9]..' '..transf[10]..' '..transf[11]..' '..transf[12]..'\n'..
            transf[13]..' '..transf[14]..' '..transf[15]..' '..transf[16])
       file:close()
    end
end


--these methods are designed to after you have an initial tree of pairs that work, create a graph of pairs that work
--so we will look at sweeps that are close by in distance to a given sweep and try to align them

--get the distance between the camera centers of all pairs in the tree
function Scan.get_distance_scores(tree)
    local transformations = {}
    local nodes = tree:get_all_nodes()
    local numTransformations = 0
    for k,v in pairs(nodes) do
        local transf = v:get_transformation_to_root()
        numTransformations = numTransformations+1
        transformations[v:get_id()] = transf
    end
    local scores=torch.zeros(numTransformations, numTransformations)
    for i=1,numTransformations do
        for j = i+1, numTransformations do
            local score = torch.dist(transformations[i]:sub(1,3,4,4),transformations[j]:sub(1,3,4,4))
            scores[i][j] = score
        end
    end
    return scores
end

--for each of the good pairs, make sure we have calculated the alignment
function Scan:calculate_alignments(scores)
    for i=1,scores:size(1) do
        for j=i+1, scores:size(1) do
            if(scores[i][j] < 10^4) then
                self:attempt_to_align_pair(self:get_sweeppair(i,j))
                collectgarbage()
            end
        end
    end

end

--returns a matrix of size nxn where each sweep pair has it's score if it's good, or math.huge if it's bad alignment
function Scan:get_good_alignments_and_scores(tree)
    local transformations = {}
    local nodes = tree:get_all_nodes()
    local numTransformations = 0
    for k,v in pairs(nodes) do
        local transf = v:get_transformation_to_root()
        numTransformations = numTransformations+1
        transformations[v:get_id()] = transf
    end
    local scores=torch.zeros(numTransformations, numTransformations)
    for i=1,numTransformations do
        for j = i+1, numTransformations do
            local score
            if(self:get_sweeppair(i,j).threed_validation_score ~= -1) then
                local success,pair = self:attempt_to_align_pair(self:get_sweeppair(i,j))
                if(not(success)) then
                    score = math.huge
                else
                    score = 1-self:get_sweeppair(i,j):get_3d_validation_score(false,10)[4]
                end
            else
                score = math.huge
            end
            scores[i][j] = score
            collectgarbage()
        end
    end
    return scores
end
