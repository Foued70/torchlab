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
   
   -- b) split: simple median split (replace with SAH)
   local split     = math.floor(nverts/2)
   local absindex1 = absindex[{{1,split}}]
   local absindex2 = absindex[{{split+1,nverts}}]
   local elems1    = indexes[{{1,split}}]
   local elems2    = indexes[{{split+1,nverts}}]
   local sv1       = select_by_index(elems1,sv)
   local sv2       = select_by_index(elems2,sv)
   local bb1       = select_by_index(elems1,bb)
   local bb2       = select_by_index(elems2,bb)

   -- c) record: this node is a split 
   local node      = tree.nodes[noffset]
   local cindex    = tree.child_index[noffset]

   node[1] = dim          -- dimension on which we split
   node[2] = bb:min(1):squeeze()[dim]
   node[3] = bb:max(1):squeeze()[dim]
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
   
   -- tree is the indices to the faces 
   tree.leaf_fid = torch.LongTensor(obj.nfaces):fill(0)
   

   tree.leafindex = 1 -- Global offset at which to write the next leaf
   tree.min_for_leaf = 4 

   local noffset = recurse_tree(tree,obj.centers,obj.bbox)

   tree.child_index = tree.child_index:narrow(1,1,noffset-1)
   tree.nodes       = tree.nodes:narrow(1,1,noffset-1)

   return tree

end

tree = build_tree(target)

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

traverse_tree(tree,pt,dir)
