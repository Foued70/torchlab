local intersect = geom.intersect
local Ray       = geom.Ray

local BIHTree = Class()

-- sv : split_verts.    Used to split the faces (obj.face_centers)
-- bb : bounding_boxes. Around the object centered in each bbox
local function recurse_build(tree,sv,bb,noffset,absindex,debug)
   local debug = false
   local nverts = sv:size(1)
   local rmin   = sv:min(1):squeeze()
   local rmax   = sv:max(1):squeeze()
   local range  = rmax - rmin

   if not noffset then
      noffset = 1
   end

   -- 1) stop recursion make a leaf
   if sv:size(1) <= tree.min_for_leaf then
      local node      = tree.nodes[noffset]
      local cindex    = tree.child_index[noffset]
      local newindex  = tree.leafindex + absindex:size(1) - 1

      tree.leaf_fid[{{tree.leafindex,newindex}}] = absindex

      node[1]   = 0 -- no dim for leaf node
      cindex[1] = tree.leafindex
      cindex[2] = newindex

      tree.leafindex = newindex + 1 -- global
      if debug then log.trace("made a leaf at", noffset) end

      return noffset + 1
   end

   -- 2) else find largest dim, split, record and recurse
   --  a) find largest dim
   local _,dim = range:max(1)
   dim   = dim:squeeze()

   --  b) sort the faces in this dimension
   --     FIXME could do a single sort at the start
   local vals,indexes = sv:select(2,dim):sort()

   if debug then
      log.trace("dim:", dim)
      log.trace("vals size:", vals:size(1),"min:", vals:min(), " max:", vals:max())
      --       print(vals)
   end

   -- bookeeping : keep track of the absolute indices
   if absindex then
      absindex = absindex[indexes]
   else
      absindex = indexes:clone()
   end

   -- b) split: on grid as in BIH paper.
   -- 
   -- Carsten Wächter & Alexander Keller (2006)
   -- Instant Ray Tracing: The Bounding Interval Hierarchy
   -- 
   -- We want to create empty space between intervals and not use the
   -- faces themselves to pick the split plane.  Intervals will fit
   -- the objects and objects with equal values will go to the same
   -- child.

   -- b1) choose split plane in the middle of the range
   -- 
   -- FIXME could do an SAH (with count of emptiness) here and
   -- optimize traversal
   
   local splitval  = rmin[dim] + range[dim]*0.5

   if debug then
      log.trace("splitval:", splitval)
   end

   -- b2) get index into sorted vals s.t.
   --      - all vals from index and below are < splitval
   --      - all vals above index are >= splitval
   local splitleft  = vals:lt(splitval):sum()
   local splitright = splitleft + 1
   
   -- book keeping
   --  - keep track of absolute indices in to face array
   local absindex1 = absindex[{{1,splitleft}}]
   local absindex2 = absindex[{{splitright,nverts}}]

   --  - keep track of relative indices
   local elems1    = indexes[{{1,splitleft}}]
   local elems2    = indexes[{{splitright,nverts}}]

   -- b3) find new points lists
   local sv1       = sv[elems1] 
   local sv2       = sv[elems2] 
   local bb1       = bb[elems1] 
   local bb2       = bb[elems2] 

   -- c) record: this node is a split node
   local node      = tree.nodes[noffset]
   local cindex    = tree.child_index[noffset]

   local eps = 1e-4

   node[1] = dim          -- dimension on which we split
   node[2] = bb1:select(2,dim+3):max() + eps -- max of left child bbox max
   node[3] = bb2:select(2,dim):min() - eps -- min of right child bbox min
   
   if debug then
      log.trace("c1max:", node[2])
      log.trace("c2min:", node[3])
   end

   noffset = noffset + 1

   -- d) recurse
   cindex[1] = noffset -- left child
   noffset = recurse_build(tree,sv1,bb1,noffset,absindex1)

   cindex[2] = noffset -- right child
   noffset = recurse_build(tree,sv2,bb2,noffset,absindex2)

   return noffset
end

-- This is a BIH-tree for fast lookups of bounding volumes.  Written in torch.
function BIHTree:__init(obj,debug)
   -- the tree is two torch tensors

   -- bulky node struct (FIXME time with nice ffi struct)
   -- [1] dim
   -- [2] bounding interval min
   -- [3] bounding interval max
   self.nodes   = torch.Tensor(obj.n_faces,3):fill(0)

   -- inner node
   -- [1] index left child
   -- [2] index right child
   -- leaf node
   -- [1] start index in leaf_fid matrix
   -- [2] stop index in leaf_fid matrix
   self.child_index = torch.LongTensor(obj.n_faces,2):fill(0)

   -- leaves are indices to the faces
   self.leaf_fid = torch.LongTensor(obj.n_faces):fill(0)


   self.leafindex = 1 -- Global offset at which to write the next leaf
   self.min_for_leaf = 4

   local noffset = recurse_build(self,obj.face_centers,obj.face_bboxes)

   -- clean up
   self.child_index = self.child_index:narrow(1,1,noffset-1)
   self.nodes       = self.nodes:narrow(1,1,noffset-1)
   self.bbox        = obj.bbox

   if debug then p(self.nodes) end

end



-- write a simple obj file to visualize the tree partitioning: each
-- leaf as separate objects.

function BIHTree:dump(obj)
   local objfilename = "tree_dump.obj"
   local objf = assert(io.open(objfilename, "w"))

   -- print vertices
   local verts = obj.verts
   for vid = 1,verts:size(1) do
      local v = verts[vid]
      objf:write(string.format("v %f %f %f %f\n",v[1],v[2],v[3],v[4]))
   end
   objf:write("\n")

   local nodes       = self.nodes
   local leaf_offset = self.child_index
   local leaf_fid    = self.leaf_fid

   local nvpf  = obj.n_verts_per_face
   local faces = obj.faces
   
   for i=1, nodes:size(1) do
     if (nodes[i][1]==0) then
       local loff = leaf_offset[i]
       local fids = leaf_fid[{{loff[1],loff[2]},{}}]
       
       objf:write("\n")
       objf:write(string.format("o node_%05d\n",i))
       for j = 1,fids:size(1) do
         local fid = fids[j]
         str = "f "
         for vid = 1,nvpf[fid] do
           str = str .. string.format("%d ",faces[fid][vid][2])
         end
        objf:write(str .. "\n")
       end
     end
   end

   objf:close()
   log.trace("Wrote", objfilename)
end

-- update mint,maxt based on the ray segment intersecting with the
-- bounding interval
local function recurse_traverse (tree,node_id,obj)
   log.trace("Traversing:", node_id)
   local node    = tree.nodes[node_id]
   local nidx    = tree.child_index[node_id]
   local dim     = node[1]
   -- not leaf
   if (dim == 0) then
      log.trace(" + leaf node: range:",  nidx[1], ",", nidx[2])
      for i = 0,nidx[2]-nidx[1] do
         local fid = tree.leaf_fid[nidx[1] + i]
         log.trace("  - face_id: ", fid, " @", nidx[1] + i)
         if obj then
            local bbx = obj.face_bboxes[fid]
            log.trace("min: ", bbx[1], bbx[2], bbx[3], " max:", bbx[4],bbx[5],bbx[6])
         end
      end
   else
      log.trace(" + inner node: children:",  nidx[1], ",", nidx[2])
      log.trace(" - dim: ", dim, " c1max: ", node[2], " c2min:", node[3])

      recurse_traverse(tree,nidx[1],obj)

      recurse_traverse(tree,nidx[2],obj)
   end
end


-- recursive traversal of tree useful for debugging or perhaps to
-- flatten the tree.
function BIHTree:walk(obj)
   local node_id    = 1
   -- start at parent
   self:recurse_traverse(node_id,obj)
end

-- return depth of closest intersection.
function BIHTree:traverse(obj,ray,debug)
   -- stack used to keep track of nodes left to visit
   local todolist  = torch.LongTensor(64)
   local todominmax = torch.Tensor(64,2)
   local todoindex = 0
   local node_id   = 1 -- start at parent
   local nodes     = self.nodes
   local children  = self.child_index
   local fids      = self.leaf_fid
   local rsign     = ray.sign

   -- In this BIH tree the node holds the max and min for the
   -- children. The initial test whether the ray intersects the whole
   -- tree needs to be run first. This is an uglier loop than I would like...
      
   -- intersect w/ bbox of whole object
   local process_tree, ray_mint, ray_maxt = intersect.ray_bbox(ray,obj.bbox)

   local mindepth = math.huge
   local face_id  = 0

   while (process_tree) do
      -- FIXME rewrite to CHECK intersect first
      local node  = nodes[node_id]
      local cids  = children[node_id]
      local dim   = node[1]
      local c1max = node[2]
      local c2min = node[3]
      local process_stack = true
      
      if (dim == 0) then
         -- 1) leaf node process primitives
         if debug then
            log.trace("Leaf node:", node_id)
         end
         local start_id = cids[1]
         local   end_id = cids[2]
         local      ids = fids[{{start_id,end_id}}]
         for i = 1,ids:size(1) do
            local fid = ids[i]
            if debug then
               log.trace("  - fid:", fid)
            end
                        
            local dp = intersect.ray_polygon(ray,obj,fid)
            if debug then print(dp) end
            if dp and dp < mindepth then
               if debug then
                  log.trace("new d:", dp)
               end
               mindepth = dp
               ray_maxt = dp
               face_id  = fid
            end
         end

      else
         if debug then
            log.trace("node: ", node_id, " dim:", dim, "c1max:",c1max, "c2min:", c2min)
            log.trace("ray: o: ", ray.origin[dim], " d:", ray.dir[dim], "mint:", ray_mint, "maxt:", ray_maxt)
         end
         -- 2) inner node evaluate children
         local dsign = (rsign[dim] == 1)
         if (dsign) then
            local intersectp, new_min, new_max =
               intersect.ray_boundary(ray,ray_mint,ray_maxt,dim,c2min)
            if debug then
               log.trace("> min child[", cids[2], "] intersect: ", intersectp, " dim: ", dim," min:", c2min)
            end
            -- if intersection, add child2 to the todo list
            if intersectp then
               if debug then
                  log.trace("  - new_min:", new_min, " new_max:", new_max)
                  log.trace("   - add to todo")
               end
               todoindex = todoindex + 1
               todolist[todoindex]      = cids[2]
               todominmax[todoindex][1] = new_min
               todominmax[todoindex][2] = new_max
            end
            -- process child 1
            local intersectp, new_min, new_max =
               intersect.ray_boundary(ray,ray_mint,ray_maxt,dim,c1max,true)
            if debug then
               log.trace("< max child[", cids[1], "] intersect: ", intersectp, " dim: ", dim, " max:", c1max)
            end
            if intersectp then
               if debug then
                  log.trace("  - new_min: ", new_min, " new_max:", new_max)
                  log.trace( "  - processing")
               end
               -- process child
               ray_mint = new_min
               ray_maxt = new_max
               node_id  = cids[1]
               process_stack = false
            end
         else
            -- check child1
            local intersectp, new_min, new_max =
               intersect.ray_boundary(ray,ray_mint,ray_maxt,dim,c1max,true)
            if debug then
               log.trace("> max child[", cids[1], "] intersect: ", intersectp, " dim: ", dim, " max:", c1max)
            end
            if intersectp then
               if debug then
                  log.trace("  - new_min: ", new_min, " new_max:", new_max)
                  log.trace("  - add to todo")
               end
               -- add child1 to the todo list
               todoindex = todoindex + 1
               todolist[todoindex]      = cids[1]
               todominmax[todoindex][1] = new_min
               todominmax[todoindex][2] = new_max
            end
            -- process child 2
            local intersectp, new_min, new_max =
               intersect.ray_boundary(ray,ray_mint,ray_maxt,dim,c2min)
            if debug then
               log.trace("> max child[", cids[2], "] intersect: ", intersectp, " dim: ", dim, " min:", c2min)
            end
            if intersectp then
               if debug then
                  log.trace("  - new_min: ", new_min, " new_max:", new_max)
                  log.trace("  - processing")
               end
               -- process child
               ray_mint = new_min
               ray_maxt = new_max
               node_id  = cids[2]
               process_stack = false
            end
         end
      end
      -- process stack
      if process_stack then
         if (todoindex > 0) then
            node_id  = todolist[todoindex]
            ray_mint = todominmax[todoindex][1]
            ray_maxt = todominmax[todoindex][2]
            todoindex = todoindex - 1
         else
            break -- we're done.
         end
      end
   end
   return mindepth,face_id
end



-- compare aggregate to and exhaustive search through all polygons.
-- FIXME write this test case.  Return get_occlusions from icebox.
function BIHTree:test_traverse(obj,ray)
   log.tic()
   local tree_d, tree_fid = self:traverse(obj,ray)
   local tree_duration = log.toc()

   log.tic()
   local slow_d,slow_fid = retex.Occlusions.get_occlusion_slow(ray,obj)
   local brute_duration = log.toc()

   log.trace("tree: ", tree_d, ",", tree_fid, ",", tree_duration,"   exhaustive: ", slow_d, ",", slow_fid, "   Speedup:", (brute_duration/tree_duration))
end