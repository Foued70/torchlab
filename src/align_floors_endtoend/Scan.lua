local path = require 'path'
local pcl = PointCloud.PointCloud

local Sweep = align_floors_endtoend.Sweep
local SweepPair = align_floors_endtoend.SweepPair
local SweepTreeNode = align_floors_endtoend.SweepTreeNode
local SweepTree = align_floors_endtoend.SweepTree
local SweepForest = align_floors_endtoend.SweepForest 
 
 Scan = Class()

--[[
po_dir = "/Users/stavbraun/Desktop/play/elegant-prize-3149/source/po_scan/a/"
savedir = "/Users/stavbraun/Desktop/play/elegant-prize-3149/source/test2"
scan = align_floors_endtoend.Scan.new(savedir, po_dir)
scan:organize_in_trees()
]]--
function Scan:__init(savedir, po_dir, complete_loop, first_sweep, extname, maxDepth)
    self.base_dir = savedir
    self.all_folders = util.fs.dirs_only(po_dir)
    self.complete_loop = complete_loop or false    

    self.numfiles = table.getn(self.all_folders)
    self.scores = {}
    self.first_sweep = first_sweep or 1
    self.maxDepth = 15 or maxDepth -- a check of diff of ids <= depth is done to determine which pairs will be attempted, so min depth should be 1 if want to check just adjacent pairs
    self.complete_loop_num = self.num_files
    if(not(self.complete_loop)) then
        self.complete_loop_num  = 10^10
    end
end

--do icp with everything, not just pairwise
function Scan:do_icp_final(tree)
    local nodes = tree:getAllNodes()
    local io = require 'io'
    local pts
    for k,v in pairs(nodes) do
        local pair = v:getPair()
        --pair:setFinalICPTransformation(torch.eye(4))
        --pair:getIcp(true)
        --pair:getFinalIcp()
       -- pc = pair:getSweep2():getPC()
       -- pc:save_global_points_to_xyz_H(v:getSweep():getName() .. "yes2.xyz", v:getTransformationToRoot())
    end

    local all_nodes = {}
    local node_distances = {}
    function getNodesAndDistances(v)
        all_nodes[table.getn(all_nodes)+1] = v
        local depth= torch.abs(tree:getRoot():getId()-v:getId())
        node_distances[table.getn(node_distances)+1] = math.min(depth%self.numfiles,(-depth)%self.numfiles)
    end
    SweepTree.traverse(tree:getRoot(), getNodesAndDistances)
    node_distances = torch.Tensor(node_distances)
    local temp,order = torch.sort(node_distances)
    local pointsA, rgbA
    for i=1,order:size(1) do
        v = all_nodes[order[i]]
        d = node_distances[order[i]]
        pair = v:getPair()
        if(not(pointsA)) then
            pointsA, rgbA = pair:getSweep2():getPoints(v:getTransformationToRoot())
        else
            points, rgb = pair:getSweep2():getPoints(v:getTransformationToRoot())
            transf = pair:getICPFromPoints(pointsA, points)
            if(transf) then
                pair:setFinalICPTransformation(transf*pair:getFinalICPTransformation())
            end
            points, rgb = pair:getSweep2():getPoints(v:getTransformationToRoot())
            pointsA = torch.cat(pointsA, points, 1)
            rgbA = torch.cat(rgbA, rgb, 1)
             
        end
    end

    --collectgarbage()
end

--save flattened images to easily visualize
function Scan:save_all_2dcombined()
    local combined_images = path.join(self.base_dir,"combined")
    util.fs.mkdir_p(combined_images)
    for i = 1,self.numfiles do
        for j=i+1,math.min(i+3,self.numfiles) do
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
    return all_scores, portion_scores, close_scores
end

function Scan:organize_in_trees()
        util.fs.mkdir_p(path.join(self.base_dir,"DOWNSAMPLEDXYZ"))
    local s_11 = self:get_sweeppair(1,1)
    local s_1 = self:get_ith_sweep(1, true)
    local tree_root = SweepTreeNode.new(s_11:getSweep1():getName(), s_1, s_11, false, nil, 1)
    local tree = SweepTree.new(tree_root)
    local forest = SweepForest.new()
    forest:add(tree)
    local i_list = torch.range(1,self.numfiles-1) --self.numfiles-1
    local j_list = i_list+1
    for k = 1,i_list:size(1) do
        local i = i_list[k]
        local j = j_list[k]
        
        local prevName = self:get_ith_sweep(i, true):getName()
        local curName = self:get_ith_sweep(j, true):getName()
        local curSweep = self:get_ith_sweep(j, true)


        local tree_containing_previous = forest:findTreeWithNode(prevName)      
        local node = tree_containing_previous:getNode(prevName)
--[[        local successTree = self:alignWithForest(forest, curSweep, curSweepId)
       if not(successTree) then
            print("creating new tree since couldn't match to any existing")
            local s_jj = self:get_sweeppair(j,j)
            local s_j = self:get_ith_sweep(j, true)
            
            local new_root = SweepTreeNode.new(s_jj:getSweep1():getName(), s_j, s_jj, false, nil, j)
            local new_tree = SweepTree.new(new_root)
            forest:add(new_tree)

            --attempt to match forwards!
        end
        ]]--
        local success, pair = self:attemptToAlignPair(self:get_sweeppair(i,j,true))
        if(success) then
            node:addChild(curName, pair, false, j)
        else
            local s_jj = self:get_sweeppair(j,j)
            local s_j = self:get_ith_sweep(j, true)
            print("creating new tree since couldn't match to any existing for" .. s_jj:getSweep1():getName())
            
            local new_root = SweepTreeNode.new(s_jj:getSweep1():getName(), s_j, s_jj, false, nil, j)
            local new_tree = SweepTree.new(new_root)
            forest:add(new_tree)
        end
        collectgarbage()
    end
    for k,v in pairs(forest:getTrees()) do
        while(self:mergeForestWithTree(forest, v)) do end --while this tree can be merged with others...
    end
    return forest
end



function Scan:mergeForestWithTree(forest, tree_orig)
    local foundMatch = false
    local function checkWithOtherNodes(curtreenode, node,depth, tree)
        if not(foundMatch) and depth<=self.maxDepth then
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
                
                curtreenode.children[node:getName()] = node
                node.from = curtreenode
                node.inverse = inverse
                node.pair = pair

                --curtreenode:addChild(node:getName(), pair, inverse, node:getId(), node.children)                 
            end
        end
    end
    forest:traverseBasedOnDistanceFromTree(tree_orig, checkWithOtherNodes, self.complete_loop_num)
    if(foundMatch) then
        forest:remove(winningTree)
    end
    return foundMatch
end

function Scan:alignWithForest(forest, curnode, i)
    local foundMatch = false
    local goodPair
    local nodeToMatch
    --for fun choose depth less than 20 as how far we go to check
    local function checkWithOtherNodes(node,depth)
        if not(foundMatch) and depth<= self.maxDepth then
            print("matching " .. curnode:getName() .. " with " .. node:getName() .. " at distance " .. depth)
            local s_ij = SweepPair.newOrLoad(self.base_dir, node.sweep,curnode)
            local success, pair = self:attemptToAlignPair(s_ij)
            if(success) then
                foundMatch = true
                goodPair = pair
                nodeToMatch = node
            end
        end
    end
    forest:traverseBasedOnDistanceFrom(i, checkWithOtherNodes, self.complete_loop_num)
    if(foundMatch) then
        nodeToMatch:addChild(goodPair:getSweep2():getName(), goodPair, false, i)
    end
    return foundMatch
end

function Scan:attemptToAlignPair(s_ij)
    print('Attempting to align ' .. s_ij:getSweep1():getName() .. " and " .. s_ij:getSweep2():getName())
    s_ij:getAllTransformations()
    local success
 --          success = s_ij:setBest3DDiffTransformation()     
    s_ij:setBestDiffTransformation(1)
    local score =  s_ij:get3dValidationScore(true, 10)
    if(score[1]<.75 or score[4]<.05) then
        return false, s_ij
    end
    s_ij:getIcp()
    
    --[[
    t = s_ij:getTransformation(true, false, false) 
    t2 = s_ij:getTransformation(false, false, false) 
    s_ij:getSweep1():getPC(t):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","first"..s_ij:getSweep1():getName()..'.xyz'))
    s_ij:getSweep2():getPC(t*t2):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","second"..s_ij:getSweep2():getName()..'.xyz'))
    t2 = s_ij:getTransformation(false, true, false) 
    s_ij:getSweep2():getPC(t*t2):write(path.join(self.base_dir,"DOWNSAMPLEDXYZ","noicpsecond"..s_ij:getSweep2():getName()..'.xyz'))
    --]]
    score =  s_ij:get3dValidationScore(false, 10)
    if (score[1]<.85 or score[4]<.15) then --not(success) then --
        print("BAD ALIGNMENT---------" .. s_ij:getSweep1():getName() .. " and " .. s_ij:getSweep2():getName())
        return false, s_ij
    else
        print("GOOD ALIGNMENT---------" .. s_ij:getSweep1():getName() .. " and " .. s_ij:getSweep2():getName())
        return true, s_ij
    end
end

function Scan:attemptToAlign(i, j)
    return self:attemptToAlignPair(scan:get_sweeppair(i,j))
end

--example input: 1,10, true
--example input2: 1,10,false
function Scan:get_ith_sweep(i, forward)
    local numcur
    if forward then
        numcur = (self.first_sweep + (i-1) + (self.numfiles-1)) % self.numfiles + 1
    else
        numcur = (self.first_sweep - (i-1) + (self.numfiles-1)) % self.numfiles + 1
    end
    return Sweep.newOrLoad(self.base_dir,string.format("sweep%03d",self.all_folders[numcur]:sub(-3,-1)), path.join(self.all_folders[numcur],"sweep.xyz"))
end

--always return smaller/larger
function Scan:get_ith_sweeppair(i, forward)
    if(forward) then
        return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i, true), self:get_ith_sweep(i+1, true)) 
    else
        return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i+1, false),self:get_ith_sweep(i, false)) 
    end

end

function Scan:get_sweeppair(i, j)
   return SweepPair.newOrLoad(self.base_dir, self:get_ith_sweep(i, true), self:get_ith_sweep(j, true))  
end

--OLD STUFF
function Scan:getOctTreeForTree(tree,res)
    local nodes = tree:getAllNodes()
    local res = res or .02
    local tree = octomap.Tree.new(res*1000*2.5)
    for k,v in pairs(nodes) do
        local transf = v:getTransformationToRoot()
        local pc = v:getSweep():getPC()
        local pts, tmp = v:getSweep():getPoints(transf)
        local center  = transf:sub(1,3,4,4):squeeze():contiguous()
        --pts = v:getSweep():getZeroAzimuthAndElevation(transf):contiguous()        
        tree:add_points(pts:contiguous(),center, pc:get_max_radius())
           tree:add_points_only_empties(v:getSweep():getZeroAzimuthAndElevation(transf):contiguous(),center, pc:get_max_radius())
        collectgarbage()
    end
    return tree
end

local function saveHelper_xyz(points, rgb, file)
   local tmpt = torch.range(1,points:size(1))                                                                                                                                
   tmpt:apply(function(i) 
      pt = points[i]
      if(rgb) then 
        rgbx = rgb[i]
        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
      else
        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..'\n')        
      end
   end)
end

function Scan:saveGlobal(tree)
    local nodes = tree:getAllNodes()
    local io = require 'io'

    local file = io.open(path.join(self.base_dir,"DOWNSAMPLEDXYZ","global.xyz"), 'w')
    for k,v in pairs(nodes) do
        local transf = v:getTransformationToRoot()
        local points = v:getSweep():getPoints(transf)
        if self.format == 1 then
         points = points/self.meter
        end  
        saveHelper_xyz(points, v:getSweep():getPC():get_rgb(), file)
    end
    file:close()
end

function Scan:saveTransformations(tree,res)
    local nodes = tree:getAllNodes()
    for k,v in pairs(nodes) do
        print(k)
        local transf = v:getTransformationToRoot()
        torch.save("transf_" .. k .. "_" .. tree.root:getName() .. ".dat", transf)
    end
    return tree
end

function Scan:findTransformationsForTree(tree)
    local nodes = tree:getAllNodes()
    local io = require 'io'
    for k,v in pairs(nodes) do
        local transf = v:getTransformationToRoot()
        local pc = v:getSweep():getPC()
        pc:set_pose_from_rotation_matrix(transf)
        pc:write(path.join(self.base_dir,"DOWNSAMPLEDXYZ",v:getSweep():getName() .. ".xyz"))
    end
end

function Scan:find_forward_and_backward()
    util.fs.mkdir_p(path.join(self.base_dir,"DOWNSAMPLEDXYZ"))
    print('looping through forward pairs')
    for i = 1,self.numfiles do
        print('create new sweep pair' .. i .. " and " .. (i+1))
        local success, pair = self:attemptToAlign(i, i+1)
        collectgarbage()
    end
    self:final_registration()

    --collectgarbage()
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
        pc:save_global_points_to_xyz(path.join(self.base_dir,"DOWNSAMPLEDXYZ",pair:getSweep2():getName()..".xyz"))
    end
end
