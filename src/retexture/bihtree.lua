require 'torch'
require 'sys'
require 'paths'
require 'math'

local util = require 'util'

local intersect = util.intersect

local Ray = util.Ray

-- This is a BIH-tree for fast lookups of bounding volumes.  Written in torch.



-- sv : split_verts.    Used to split the faces (obj.centers)
-- bb : bounding_boxes. Around the object centered in each bbox
function recurse_tree(tree,sv,bb,noffset,absindex)
   local debug  = false
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
      printf("dim: %d", dim)
      print(vals)
   end
   
   -- bookeeping : keep track of the absolute indices
   if absindex then
      local idx = indexes:clone()
      -- replace indexes with the absolute face index values
      idx:apply(function(x) return absindex[x] end)
      absindex:copy(idx)
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
   -- FIXME could do an SAH (with count of emptiness) here and optimize traversal
   local splitval  = rmin[dim] + range[dim]*0.5 
   
   if debug then
      printf("splitval: %f", splitval)
   end
   
   -- b2) get index into sorted vals s.t. 
   --      - all vals from index and below are < splitval
   --      - all vals above index are >= splitval
   local splitidx  = util.get_index_lt_val(vals,splitval)

   -- book keeping
   --  - keep track of absolute indices in to face array
   local absindex1 = absindex[{{1,splitidx}}]
   local absindex2 = absindex[{{splitidx+1,nverts}}]
   --  - keep track of relative indices
   local elems1    = indexes[{{1,splitidx}}]
   local elems2    = indexes[{{splitidx+1,nverts}}]

   -- b3) find new points lists
   local sv1       = util.select_by_index(elems1,sv)
   local sv2       = util.select_by_index(elems2,sv)
   local bb1       = util.select_by_index(elems1,bb)
   local bb2       = util.select_by_index(elems2,bb)

   -- c) record: this node is a split node
   local node      = tree.nodes[noffset]
   local cindex    = tree.child_index[noffset]

   local eps = 1e-4

   node[1] = dim          -- dimension on which we split
   node[2] = bb1:select(2,dim+3):max() + eps -- max of left child bbox max
   node[3] = bb2:select(2,dim):min() - eps -- min of right child bbox min

   if debug then
      printf("c1max: %f",node[2])
      printf("c2min: %f",node[3])
   end

   noffset = noffset + 1
   
   -- d) recurse

   cindex[1] = noffset -- left child
   noffset = recurse_tree(tree,sv1,bb1,noffset,absindex1)
   
   cindex[2] = noffset -- right child
   noffset = recurse_tree(tree,sv2,bb2,noffset,absindex2)
   
   return noffset
end

function build_tree(obj,debug)
   
   -- the tree is two torch tensors
   
   local tree = {}
   -- bulky node struct (FIXME time with nice ffi struct)
   -- [1] dim
   -- [2] bounding interval min
   -- [3] bounding interval max
   tree.nodes   = torch.Tensor(obj.nfaces,3):fill(0)
   
   -- inner node
   -- [1] index left child
   -- [2] index right child
   -- leaf node
   -- [1] start index in leaf_fid matrix
   -- [2] stop index in leaf_fid matrix
   tree.child_index = torch.LongTensor(obj.nfaces,2):fill(0)
   
   -- leaves are indices to the faces 
   tree.leaf_fid = torch.LongTensor(obj.nfaces):fill(0)
   

   tree.leafindex = 1 -- Global offset at which to write the next leaf
   tree.min_for_leaf = 4 

   local noffset = recurse_tree(tree,obj.centers,obj.face_bboxes)

   -- clean up 
   tree.child_index = tree.child_index:narrow(1,1,noffset-1)
   tree.nodes       = tree.nodes:narrow(1,1,noffset-1)
   tree.bbox        = obj.bbox
   return tree

end

-- write a simple obj file to visualize the tree partitioning: each
-- leaf as separate objects.

function dump_tree(tree,obj)
   local objfilename = "tree_dump.obj"
   local objf = assert(io.open(objfilename, "w"))

   -- print vertices
   local verts = obj.verts
   for vid = 1,verts:size(1) do 
      local v = verts[vid]
      objf:write(string.format("v %f %f %f %f\n",v[1],v[2],v[3],v[4]))
   end
   objf:write("\n")

   local nodes       = tree.nodes
   local leaf_offset = tree.child_index
   local leaf_fid    = tree.leaf_fid

   local nvpf  = obj.nverts_per_face
   local faces = obj.faces

   for i = 1,nodes:size(1) do 
      if (nodes[i][1] == 0) then
         local loff = leaf_offset[i]
         local fids = leaf_fid[{{loff[1],loff[2]},{}}]
         objf:write("\n") 
         objf:write(string.format("o node_%05d\n",i))
         for j = 1,fids:size(1) do 
            local fid = fids[j]
            str = "f "
            for vid = 1,nvpf[fid] do 
               str = str .. string.format("%d ",faces[fid][vid])
            end
            objf:write(str .. "\n") 
         end
            
      end
   end
   objf:close()
   printf("Wrote %s",objfilename)
end

-- update mint,maxt based on the ray segment intersecting with the
-- bounding interval
function recurse_traverse (tree,node_id,obj)
   printf("Traversing: %d",node_id)
   local node    = tree.nodes[node_id]
   local nidx    = tree.child_index[node_id]
   local dim     = node[1]
   -- not leaf
   if (dim == 0) then
      printf(" + leaf node: range: %d,%d",  nidx[1],nidx[2])
      for i = 0,nidx[2]-nidx[1] do 
         local fid = tree.leaf_fid[nidx[1] + i]
         printf("  - face_id: %d @ %d", fid , nidx[1] + i)
         if obj then
            local bbx = obj.face_bboxes[fid]
            printf("min: %f %f %f max: %f %f %f",
                   bbx[1],bbx[2],bbx[3],bbx[4],bbx[5],bbx[6])
         end
      end
   else
      printf(" + inner node: children: %d,%d",  nidx[1],nidx[2])
      printf(" - dim: %d c1max: %f c2min: %f", dim, node[2], node[3])
 
      recurse_traverse(tree,nidx[1],obj)

      recurse_traverse(tree,nidx[2],obj)
   end
end


-- recursive traversal of tree useful for debugging or perhaps to flatten the tree.
function walk_tree(tree,obj)
   local node_id    = 1
   -- start at parent
   recurse_traverse(tree,node_id,obj)
end

-- return depth of closest intersection.
function traverse_tree(tree,obj,ray,debug)
   -- stack used to keep track of nodes left to visit
   local todolist  = torch.LongTensor(64)
   local todominmax = torch.Tensor(64,2)
   local todoindex = 0
   local node_id   = 1 -- start at parent 
   local nodes     = tree.nodes
   local children  = tree.child_index
   local fids      = tree.leaf_fid
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
            printf("Leaf node: %d", node_id)
         end
         local start_id = cids[1]
         local   end_id = cids[2]
         local      ids = fids[{{start_id,end_id}}]
         for i = 1,ids:size(1) do 
            local fid = ids[i]
            if debug then
               printf("  - fid: %d",fid)
            end
            local dp = intersect.ray_polygon(ray,obj,fid)
            if debug then print(dp) end
            if dp and dp < mindepth then
               if debug then
                  printf("new d: %f", dp)
               end
               mindepth = dp
               ray_maxt = dp
               face_id  = fid
            end
         end
         
      else
         if debug then
            printf("node: %d dim: %d c1max: %f c2min: %f",
                   node_id, dim, c1max, c2min) 
            printf("ray: o: %f d: %f mint: %f maxt: %f", 
                   ray.origin[dim], ray.dir[dim], ray_mint, ray_maxt)
         end
         -- 2) inner node evaluate children
         local dsign = (rsign[dim] == 1)
         if (dsign) then
            local intersectp, new_min, new_max = 
               intersect.ray_boundary(ray,ray_mint,ray_maxt,dim,c2min)
            if debug then
               printf("> min child[%d] intersect: %s dim: %d min: %f",
                      cids[2], intersectp, dim, c2min) 
            end
            -- if intersection, add child2 to the todo list
            if intersectp then
               if debug then 
                  printf("  - new_min: %f new_max: %f", new_min, new_max)
                  print("   - add to todo")
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
               printf("< max child[%d] intersect: %s dim: %d max: %f ",
                      cids[1], intersectp, dim, c1max)
            end
            if intersectp then
               if debug then 
                  printf("  - new_min: %f new_max: %f", new_min, new_max)
                  print( "  - processing")
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
               printf("> max child[%d] intersect: %s dim: %d max: %f",
                      cids[1], intersectp, dim, c1max) 
            end
            if intersectp then
               if debug then 
                  printf("  - new_min: %f new_max: %f", new_min, new_max)
                  print ("  - add to todo")
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
               printf("> min child[%d] intersect: %s dim: %d min: %f",
                      cids[2], intersectp, dim, c2min)
            end
            if intersectp then
               if debug then
                  printf("  - new_min: %f new_max: %f", new_min, new_max)
                  print( "  - processing")
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
function test_traverse()
   local tree = build_tree(target)
   local obj  = target
   local ray  = Ray(torch.Tensor({0,0,0}),torch.Tensor({1,1,1}))
   
   local d    = traverse_tree(tree,obj,ray)
   
   local slow_d,slow_fid = get_occlusions(ray.origin,ray.dir,obj)
   printf("tree: %f exhaustive: %f, %d",d,slow_d,slow_fid)
end
