require 'torch'
require 'sys'
require 'paths'
require 'math'
require 'util'

-- This is a BIH-tree for fast lookups of bounding volumes.  Written in torch.

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
cmd:text('Options')
cmd:option('-targetfile',
           "rivercourt_3307_scan/rivercourt_3307.obj",
           'target obj with new geometry')

cmd:text()
 
-- parse input params
params = cmd:parse(arg)

local axes = torch.eye(3)

targetfile = params.targetfile

cachedir = "cache/"
sys.execute("mkdir -p " .. cachedir)

targetcache = cachedir .. targetfile:gsub("/","_") .. ".t7"

function loadcache (objfile,cachefile,loader,args)
   local object = nil
   -- Process or load the poses
   if (paths.filep(cachefile)) then
      sys.tic()
      object = torch.load(cachefile)
      printf("Loaded %s from %s in %2.2fs", objfile, cachefile, sys.toc())
   else
      object = loader(objfile,args)
      torch.save(cachefile,object)
      printf("Saving %s to %s", objfile, cachefile)
   end
   return object
end

if not target then
   target = loadcache(targetfile,targetcache,util.obj.load,8)
end


-- FIXME add next funcs to utils

-- need C code for this

function select_by_index(ind,mat)
   local nelem = ind:size(1)
   -- allow for any dimension as long as we index by the first
   local shape = mat:size()
   shape[1] = nelem

   local s = torch.Tensor(shape)
   for i = 1,nelem do 
      s[i] = mat[ind[i]]
   end
   return s
end

-- <vals> sorted list of values
-- 
-- <val> value for which we want pointer into list where everything
--       before pointer is < val and after >= val

function get_index_lt_val(vals,val)
   local idx = vals:size(1)*0.5
   local cval = vals[idx]
   local nval = vals[idx+1]
   local step = idx*0.5
   local count = 0
   while true do 
      count = count + 1
      if (cval >= val) then
         idx = idx - step
      elseif (nval < val) then
         idx = idx + step
      else
         break
      end
      step = math.floor(0.5 + (step * 0.5))
      if (idx <= 0) or (idx >= vals:size(1)) or (step < 1) then
         break
      end
      cval = vals[idx]
      nval = vals[idx+1]
   end
   if (idx < 1) or (idx >= vals:size(1)) then
      return -1,count
   end

   return math.floor(idx),count
end

function test_get_index_lt_val()
   local r = torch.sort(torch.randn(1000))
   local nr = r:size(1)
   
   function eval_test(r,val,verbose)
      local idx,count = get_index_lt_val(r,val)
      local err = 0
      if (idx > 0) then
         local cval = r[idx]
         local nval = r[idx+1]
         if (cval < val) and (nval >= val) then
            if verbose then
               printf("  OK idx: %d val: %f count: %d", idx, val, count)
            end
         else
            if verbose then
               printf("  ERROR idx: %d val: %f cval: %f nval: %f", idx, val, cval, nval)
            end
            errs = 1
         end
      else
         if (val <= r[1]) or (val > r[-1]) then
            if verbose then
               printf("  OK val: %f out of bounds",val)
            end
         else
            if verbose then
               printf("  ERROR idx: %d val: %f minr: %f maxr: %f", idx, val, r[1],r[nval])
            end
            errs = 1
         end
      end
      return err,count
   end

   print("Test 1 values in set")
   local steps = 0
   local errs  = 0
   
   for i = 1,nr do 
      local val = r[i]
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 2 jittered values")
   steps = 0
   errs  = 0
   
   for i = 1,nr do 
      local val = r[i] + torch.randn(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 3 bounds")
   steps = 0
   errs  = 0
   local rmin = r[1]
   local rmax = r[-1]
   for i = 1,nr*0.5 do 
      local val = rmin - torch.rand(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   for i = 1,nr*0.5 do 
      local val = rmax + torch.rand(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f/%d",errs,nr,steps/nr,nr)

   print("Test 4 duplicate + jittered values")
   steps = 0
   errs  = 0
   local uf = r:unfold(1,3,3)
   for i = 1,uf:size(1) do 
      uf[i]:fill(torch.randn(1)[1])
   end
   r = torch.sort(r)
   for i = 1,nr do 
      local val = r[i] + torch.randn(1)[1]*1e-5
      local err,count = eval_test(r,val)
      steps = steps + count
      errs = errs + err
   end
   printf("-- %d/%d Errors average steps: %2.2f",errs,nr,steps/nr,nr)

   
end

-- sv : split_verts.    Used to split the faces (obj.centers)
-- bb : bounding_boxes. Around the object centered in each bbox
function recurse_tree(tree,sv,bb,noffset,absindex)
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
      node[2]   = 
      node[3]   
      cindex[1] = tree.leafindex
      cindex[2] = newindex
      
      tree.leafindex = newindex + 1 -- global

      return noffset + 1
   end
   
   -- 2) else find largest dim, split, record and recurse
   --  a) find largest dim
   local _,dim = range:max(1)
   dim   = dim:squeeze()
   
   --  b) sort the faces in this dimension (FIXME should do at the start)
   local vals,indexes = sv:select(2,dim):sort()
   
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
   local splitval  = rmin[dim] + range[dim]*0.5 

   -- b2) get index into sorted vals s.t. 
   --      - all vals from index and below are < splitval
   --      - all vals above index are >= splitval
   local splitidx  = get_index_lt_val(vals,splitval)

   -- book keeping
   --  - keep track of absolute indices in to face array
   local absindex1 = absindex[{{1,splitidx}}]
   local absindex2 = absindex[{{splitidx+1,nverts}}]
   --  - keep track of relative indices
   local elems1    = indexes[{{1,splitidx}}]
   local elems2    = indexes[{{splitidx+1,nverts}}]

   -- b3) find new points lists
   local sv1       = select_by_index(elems1,sv)
   local sv2       = select_by_index(elems2,sv)
   local bb1       = select_by_index(elems1,bb)
   local bb2       = select_by_index(elems2,bb)

   -- c) record: this node is a split node
   local node      = tree.nodes[noffset]
   local cindex    = tree.child_index[noffset]

   node[1] = dim          -- dimension on which we split
   node[2] = bb:min(1):squeeze()[dim] -- min 
   node[3] = bb:max(1):squeeze()[dim] -- max
   noffset = noffset + 1
   
   -- d) recurse
   noffset = recurse_tree(tree,sv1,bb1,noffset,absindex1)
   cindex[1] = noffset -- left child
   
   noffset = recurse_tree(tree,sv2,bb2,noffset,absindex2)
   cindex[2] = noffset -- right child

   return noffset
end

function build_tree(obj)
   
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

   local noffset = recurse_tree(tree,obj.centers,obj.bbox)

   tree.child_index = tree.child_index:narrow(1,1,noffset-1)
   tree.nodes       = tree.nodes:narrow(1,1,noffset-1)

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
function recurse_traverse (tree,node_id,ray_origin,ray_invdir,ray_mint,ray_maxt)

   -- keep splitting ray
   local node = tree.nodes[node_id]
   local dim  = node[1]
   if (dim > 0) then
      local minv = node[2]
      local maxv = node[3]
      -- not leaf
      printf("node: dim: %f min: %f max: %f",dim,minv,maxv)
      
      -- intersection with Axis-aligned bbox (see pbrt book p.194)
      local tmin = (minv - ray_origin[dim]) * ray_invdir[dim]
      local tmax = (maxv - ray_origin[dim]) * ray_invdir[dim]

      printf("tmin: %f tmax: %f", tmin, tmax)
      
   end
end

function traverse_tree(tree,ray_origin,ray_dir)
   local ray_invdir = ray_dir:pow(-1)
   local ray_mint   = 0
   local ray_maxt   = math.huge
   local node_id    = 1
   local todo       = torch.LongTensor(64)
   local todopos    = 1
   for node_id = 1,tree.nodes:size(1) do 
      recurse_traverse(tree,node_id,ray_origin,ray_invdir,ray_mint,ray_maxt)
   end
end

pt = torch.Tensor(3):fill(1)
dir = util.geom.normalize(torch.Tensor(3):fill(1))

tree = build_tree(target)
-- dump_tree(tree,target)

traverse_tree(tree,pt,dir)
