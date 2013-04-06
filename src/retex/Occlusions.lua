-- takes a scan and writes a .t7 to filesystem with depth map for each photo
-- example usage:
-- occlusions = retex.Occlusions.new(scan, scale, packetsize)
-- occlusions:calc()

require 'torch'
require 'sys'
require 'paths'
require 'math'

local loader = require 'util.loader'
local Ray = require 'util.Ray'
local bihtree = require 'util.bihtree'
local interpolate = require 'util.interpolate'
local intersect = require 'util.intersect'

local Occlusions = Class()

function Occlusions:__init(scan, scale, packetsize)
  if not scan then error('arguments invalid') end

  self.scan = scan
  self.output_dir = paths.concat(paths.dirname(scan.path), 'occlusions')
  sys.execute("mkdir -p " .. self.output_dir)
  
  self.scale = scale or 4
  self.packetsize = packetsize
  if packetsize and packetsize < 1 then self.packetsize = nil end
end

-- get occlusions for each photo, trying to load occlusions from torch file
-- if occlusions can't be found for the pose, set to nil
function Occlusions:get()
  if not self.occlusions then
    local occlusions = {}
    for i=1, #self.scan.sweeps do
      occlusions[i] = {}
      for j=1, #self.scan.sweeps[i].photos do
        local photo = self.scan.sweeps[i].photos[j]
        if paths.filep(self:file(photo)) then
          occlusions[i][j] = torch.load(self:file(photo))
          log.trace('Loaded occlusions for photo', photo.name)        
        else
          occlusions[i][j] = nil
          log.trace('No occlusions found for photo', photo.name)
        end
      end
    end
  end
  
  return self.occlusions
end

-- filename to use when saving and loading occlusions for a photo
function Occlusions:file(photo)
  local occ_file = string.format('%s-%s-s%s-depth.t7', paths.basename(self.scan.path), photo.name, self.scale)
  return paths.concat(self.output_dir, occ_file)
end

-- calculate occlusions for all photos
function Occlusions:calc()  
  local target = self.scan:get_model_data()
  local occlusions = {}
  
  sys.tic()
  local tree = bihtree.build(target)
  log.trace("Built tree in", sys.toc())
  
  for i=1, #self.scan.sweeps do
    occlusions[i] = {}
    for j=1, #self.scan.sweeps[i].photos do
      local photo     = self.scan.sweeps[i].photos[j]
      local dirs      = photo:get_dirs(self.scale,self.packetsize)
      
      local out_tree  = torch.Tensor(dirs:size(1),dirs:size(2))
      local fid_tree  = torch.LongTensor(dirs:size(1),dirs:size(2))
      
      sys.tic()
      log.trace("Computing depth map for photo", photo.name, '('..i..'/'..j..') at scale 1/'..self.scale)
      
      local tot = 0
      local totmiss = 0
      local position = photo.position
      
      for ri = 1,dirs:size(1) do
        for ci = 1,dirs:size(2) do
          local ray = Ray.new(position,dirs[ri][ci])
          local tree_d, tree_fid = bihtree.traverse(tree,target,ray) -- turned off debugging
          -- bihtree.test_traverse(tree,target,ray)
          -- local tree_d, tree_fid = self:get_occlusion_slow(ray,target)
          -- log.trace("traversed tree in", sys.toc())

          tot = tot + 1
          out_tree[ri][ci] = tree_d
          fid_tree[ri][ci] = tree_fid
          if (tree_d == math.huge) then
            totmiss = totmiss + 1
          end
        end
      end
      
      log.trace("Computing depth map for photo", photo.name, '('..i..'/'..j..') in', sys.toc())      
      log.trace("Interpolating for", totmiss, "missed edges out of", tot)

      sys.tic()
      interpolate.math_huge(out_tree)
      log.trace("Interpolation done", sys.toc())

      image.display{image={out_tree},min=0,max=10}

      local output_file = self:file(photo)
      log.trace("Saving depth map:", output_file)
      torch.save(output_file, out_tree)

      occlusions[i][j] = out_tree
    end
  end
  
  self.occlusions = occlusions
end

-- Intentionally very slow.  Checks _all_ the faces. Returns closest
-- intersection. Used for debugging the aggregates.
function Occlusions:get_occlusion_slow(ray,obj,debug)
   local mindepth        = math.huge
   local fid             = 0
   local nverts_per_face = obj.n_verts_per_face
   local face_verts      = obj.face_verts
   local normals         = obj.face_normals
   local ds              = obj.face_center_dists
   -- exhausting loop through all faces
   for fi = 1,obj.n_faces do
      local nverts = nverts_per_face[fi]
      local verts  = face_verts[fi]:narrow(1,1,nverts)      
      local normal = normals[fi]
      local d      = ds[fi]

      
      local testd = intersect.ray_polygon(ray,obj,fi,debug)
      local bstr  = " "
      if testd and (testd < mindepth) then
         bstr = "*"
         mindepth = testd
         fid = fi
      end
      if debug then 
         if not testd then testd = math.huge end
         printf("%s[%05d] : %f", bstr,fi,testd)
      end
      
   end
   return mindepth,fid
end