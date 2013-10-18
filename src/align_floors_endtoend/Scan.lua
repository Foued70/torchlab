local path = require 'path'
local pcl = PointCloud.PointCloud

local Sweep = align_floors_endtoend.Sweep
local SweepPair = align_floors_endtoend.SweepPair
local SweepTreeNode = align_floors_endtoend.SweepTreeNode
local SweepTree = align_floors_endtoend.SweepTree
local SweepForest = align_floors_endtoend.SweepForest 

local Scan = Class()

Scan.XYZ = "XYZ"


function Scan:__init(base_dir, complete_loop, first_sweep)
    self.base_dir = base_dir
    self.xyz_dir = path.join(base_dir,Scan.XYZ)
    if(not(util.fs.is_dir(self.xyz_dir))) then
        error("xyz dir given does not exist")
    end
    self.all_files = util.fs.files_only(self.xyz_dir,'.xyz')
    self.complete_loop = complete_loop or false    

    self.numfiles = #self.all_files
    self.scores = {}
    self.first_sweep = first_sweep or 1
end

function Scan:load_and_save_all_sweeps()
    if util.fs.is_dir(self.xyz_dir) then
        for i = 1,self.numfiles do
            local fname = self.all_files[i]
            local bname = path.basename(fname,'.xyz')
            local aSweep = Sweep.newOrLoad(self.base_dir,bname)
            local pc = aSweep:getPC()
        end
        collectgarbage()
    end
end

function Scan:find_forward_and_backward()
    util.fs.mkdir_p(path.join(self.base_dir,"DOWNSAMPLEDXYZ"))
    print('looping through forward and back pairs')
    for i = 1,self.numfiles do
        print('create new sweep pair')
        local success, pair = self:attemptToAlign(i, i+1)
        collectgarbage()
    end
    self:final_registration()

    collectgarbage()
end

function Scan:final_registration()
    --[[]]
    local transfSoFarForward = torch.eye(4) --can get first transformations alignment if want
    local transF = {}
    for i = 1,self.numfiles-1 do
        local pair = self:get_ith_sweeppair(i, true) 
        
        local transf = pair:getTransformation(false, false, false)
        print(transf)
        transfSoFarForward = transfSoFarForward*transf
        print(transfSoFarForward)
        transF[i] = transfSoFarForward
    end

    local transfSoFarBackward = torch.eye(4)
    local transB = {}
    if self.complete_loop then
        error("not yet tested") --this should work, but havent tested yet
        for i = 1,self.numfiles-1 do
            local pair = self:get_ith_sweeppair(i, false) 
            local transb = pair:getTransformation(false, false, true)
            transfSoFarBackward = transfSoFarBackward*transb
            transB[self.numfiles-i] = transfSoFarBackward
        end    
    end    

    local transavg
    for i = 1,self.numfiles-1 do
        local pair = self:get_ith_sweeppair(i, true) 
        print(i)
        print(self.numfiles-i)
        local transavg
        if self.complete_loop then
            local steps_F = i
            local steps_B = self.numfiles - i
            transavg = transF[i]:clone():mul(steps_B):add(transB[i]:clone():mul(steps_F))
            :div(steps_F+steps_B)   
        else
            transavg = transF[i]
        end
        print(transavg)
        local pc = pair:getSweep2():getPC()
        pc:set_pose_from_rotation_matrix(transavg)
        pc:save_to_od(pair:getSweep2().fod)
        pc:save_downsampled_global_to_xyz(0.01, path.join(self.base_dir,"DOWNSAMPLEDXYZ",pair:getSweep2():getName()..'.xyz'))

    end
end
function Scan:save_all_2dcombined()
    local combined_images = path.join(self.base_dir,"combined")
    util.fs.mkdir_p(combined_images)
    for i = 17,self.numfiles-1 do
        for j=i+1,self.numfiles do
            local sweepPair_curr_to_nex = self:get_sweeppair(i,j)
            local name1 = sweepPair_curr_to_nex:getSweep1():getName()
            local name2 = sweepPair_curr_to_nex:getSweep2():getName()
            sweepPair_curr_to_nex:saveCurrent(path.join(combined_images, name1 .. "_" .. name2 .. ".png"))
            collectgarbage()
        end
    end
    collectgarbage()
end

function Scan:find_forward_and_backward_all_pairs()

    util.fs.mkdir_p(path.join(self.base_dir,"DOWNSAMPLEDXYZ"))
    print('looping through forward and back pairs')
    local all_scores = torch.zeros(self.numfiles, self.numfiles)
    local portion_scores = torch.zeros(self.numfiles, self.numfiles)
    local close_scores = torch.zeros(self.numfiles, self.numfiles)
    local nclose_scores = torch.zeros(self.numfiles, self.numfiles)

    for i = 1,self.numfiles-1 do
        for j=i+1,self.numfiles do

            print('create new sweep pair')
            local success, pair = self:attemptToAlign(i,j)
            local score = s_ij:get3dValidationScore()
            all_scores[i][j] = score[1]
            portion_scores[i][j] = score[2]
            close_scores[i][j] = score[3]
            nclose_scores[i][j] = score[4]

            collectgarbage()
        end
    end
    collectgarbage()
    print(all_scores)
    torch.save("woohoo2.dat", {all_scores, portion_scores, close_scores, nclose_scores})
    return all_scores, portion_scores, close_scores
end
--scan = align_floors_endtoend.Scan.new("/Users/stavbraun/Desktop/play/motor-unicorn-0776_play")

function Scan:organize_in_trees()
        util.fs.mkdir_p(path.join(self.base_dir,"DOWNSAMPLEDXYZ"))
    local s_11 = self:get_sweeppair(1,1)
    local s_1 = self:get_ith_sweep(1, true)
    local tree_root = SweepTreeNode.new(s_11:getSweep1():getName(), s_1, s_11, false, nil, 1)
    local tree = SweepTree.new(tree_root)
    local tree_forest = SweepForest.new()
    tree_forest:add(tree)
    local i_list = torch.range(1,self.numfiles-1) --self.numfiles-1
    local j_list = i_list+1
    local used_list = {}
    used_list[1] = 1
    for k = 1,i_list:size(1) do
        i = i_list[k]
        j = j_list[k]
        local curSweepId = k+1
        local prevName = self:get_ith_sweep(i, true):getName()
        local curName = self:get_ith_sweep(j, true):getName()
        local curSweep = self:get_ith_sweep(j, true)
        local tree_containing_previous = tree_forest:findTreeWithNode(prevName)      
        local node = tree_containing_previous:getNode(prevName)
        local successTree = self:alignWithForest(tree_forest, curSweep, curSweepId)
        if successTree then
            used_list[curSweepId] = curSweepId
        else 
            print("creating new tree since couldn't match to any existing")
            local s_jj = self:get_sweeppair(j,j)
            local s_j = self:get_ith_sweep(j, true)
            
            local new_root = SweepTreeNode.new(s_jj:getSweep1():getName(), s_j, s_jj, false, nil, curSweepId)
            local new_tree = SweepTree.new(new_root)
            tree_forest:add(new_tree)

            --attempt to match forwards!
        end
        collectgarbage()
    end
    for k,v in pairs(tree_forest:getTrees()) do
        while(self:mergeForestWithTree(tree_forest, v)) do end --while this tree can be merged with others...
    end
    torch.save("test.dat",tree_forest)
    return tree_forest
end

function Scan:mergeForestWithTree(forest, tree_orig)
    local foundMatch = false
    local maxDepth = 15
    --for fun choose depth less than 20 as how far we go to check
    local function checkWithOtherNodes(curtreenode, node,depth, tree)
        if not(foundMatch) and depth<maxDepth then
            local smallerNode = curtreenode
            local largerNode = node
            local inverse = false
            if(node:getId() < curtreenode:getId()) then
                smallerNode = node
                largerNode = curtreenode
                inverse = true
            end
            local s_ij = SweepPair.newOrLoad(self.base_dir, smallerNode.sweep, largerNode.sweep)
            local success, pair = self:attemptToAlignPair(s_ij)
            if(success) then
                print("merging forests based on nodes" .. curtreenode:getName() .. " and " .. node:getName())
                foundMatch = true
                winningTree = tree
                winningTree:changeRoot(node)
                curtreenode:addChild(node:getName(), pair, inverse, node:getId(), node.children)                 
            end
        end
    end
    forest:traverseBasedOnDistanceFromTree(tree_orig, checkWithOtherNodes, self.numfiles)
    if(foundMatch) then
        forest:remove(winningTree)
    end
    return foundMatch
end

function Scan:alignWithForest(forest, curnode, i)
    local foundMatch = false
    local goodPair
    local maxDepth = 15
    local nodeToMatch
    --for fun choose depth less than 20 as how far we go to check
    local function checkWithOtherNodes(node,depth)
        if not(foundMatch) and depth< maxDepth then
            print("matching with " .. node:getName() .. " at distance " .. depth)
            local s_ij = SweepPair.newOrLoad(self.base_dir, node.sweep,curnode)
            local success, pair = self:attemptToAlignPair(s_ij)
            if(success) then
                foundMatch = true
                goodPair = pair
                nodeToMatch = node
            end
        end
    end
    forest:traverseBasedOnDistanceFrom(i, checkWithOtherNodes, self.numfiles)
    if(foundMatch) then
        nodeToMatch:addChild(goodPair:getSweep2():getName(), goodPair, false, i)
    end
    return foundMatch
end

function Scan:attemptToAlignPair(s_ij)
    print('Attempting to align ' .. s_ij:getSweep1():getName() .. " and " .. s_ij:getSweep2():getName())
    s_ij:getAllTransformations()
    s_ij:setBestDiffTransformation(1)
    score =  s_ij:get3dValidationScore()
    if(score[1]>.8 and score[4]>.3) then
        s_ij:getIcp()
    end
    --[[
    t = s_ij:getTransformation(true, false, false) 
    t2 = s_ij:getTransformation(false, false, false) 
    s_ij:getSweep1():getPC(t):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","first"..s_ij:getSweep1():getName()..'.xyz'))
    s_ij:getSweep2():getPC(t*t2):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","second"..s_ij:getSweep2():getName()..'.xyz'))
    t2 = s_ij:getTransformation(false, true, false) 
    s_ij:getSweep2():getPC(t*t2):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","noicpsecond"..s_ij:getSweep2():getName()..'.xyz'))
    --]]
    score =  s_ij:get3dValidationScore()
    if(score[1]<.85 or score[4]<.3) then
        print("BAD ALIGNMENT---------" .. s_ij:getSweep1():getName() .. " and " .. s_ij:getSweep2():getName())
        return false, s_ij
    else
        return true, s_ij
    end
end

function Scan:attemptToAlign(i, j)
    s_ij = scan:get_sweeppair(i,j)

    return self:attemptToAlignPair(s_ij)
end

--example input: 1,10, true
--example input2: 1,10,false
function Scan:get_ith_sweep(i, forward)
    if forward then
        -- forwards
        numcur = (self.first_sweep + (i-1) + (self.numfiles-1)) % self.numfiles + 1
    else
        --backwards
        numcur = (self.first_sweep - (i-1) + (self.numfiles-1)) % self.numfiles + 1
    end
    local fnamecur = self.all_files[numcur]
    local bnamecur = path.basename(fnamecur,'.xyz')
    return Sweep.newOrLoad(self.base_dir,bnamecur)
end

--always return smaller/larger
function Scan:get_ith_sweeppair(i, forward)
    if(forward) then
        return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i, true), self:get_ith_sweep(i+1, true)) 
    else
        return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i+1, true),self:get_ith_sweep(i, true)) 
    end

end

function Scan:get_sweeppair(i, j)
    local numcur, numnex
        -- forwards
    numcur = i
    numnex = j
                
    return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i, true), self:get_ith_sweep(j, true))
  
end