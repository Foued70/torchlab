-- takes a scan and writes a .t7 to filesystem with depth map for each photo
-- example usage:
-- occlusions = retex.Occlusions.new(scan, scale, packetsize)
-- occlusions:calc()

require 'torch'
require 'sys'
require 'paths'
require 'math'

local loader = require 'util.loader'
local Ray = util.Ray
local intersect = util.intersect
local bihtree = util.bihtree
local interpolate = util.interpolate


local Occlusions = Class()

function Occlusions:__init(scan, scale, packetsize)
  if not scan then error('arguments invalid') end

  self.scan = scan
  self.output_dir = paths.concat(scan.path, 'occlusions')
  sys.execute("mkdir -p " .. self.output_dir)
  
  self.scale = scale or 4
  self.packetsize = packetsize
  if packetsize and packetsize < 1 then self.packetsize = nil end
end

-- get occlusions for each photo, trying to load occlusions from torch file
-- if occlusions can't be found for the pose, set to nil or calc them if flag to calc is set
function Occlusions:get(force_calc)
  if not self.occlusions then
    local occlusions = {}    
    local photos = self.scan:get_photos()
    for i=1, #photos do
      local photo = photos[i]      
      if paths.filep(self:file(photo)) then
        table.insert(occlusions, torch.load(self:file(photo)))
        log.trace('Loaded occlusions for photo', photo.name)        
      else
        if force_calc then
          table.insert(occlusions, self:calc_for_photo(photo))
          log.trace('Calculating occlusions for', photo.name)
        else
          table.insert(occlusions, nil)
          log.trace('No occlusions found for photo', photo.name)        
        end
      end
    end
    self.occlusions = occlusions
  end
  
  return self.occlusions
end

-- filename to use when saving and loading occlusions for a photo
function Occlusions:file(photo)
  local occ_file = string.format('%s-%s-%s-s%s-depth.t7', 
    paths.basename(self.scan.model_file), paths.basename(self.scan.pose_file), photo.name, self.scale)
  return paths.concat(self.output_dir, occ_file)
end

function Occlusions:get_tree()
  if not self.tree then
    sys.tic()
    self.tree = bihtree.build(self:get_target())
    log.trace('built tree in', sys.toc())
  end
  
  return self.tree
end

function Occlusions:get_target()
  if not self.target then
    self.target = self.scan:get_model_data()
  end
  return self.target
end

function Occlusions:calc_for_photo(photo)
  local target    = self:get_target()
  local tree      = self:get_tree()
  
  local dirs      = photo:get_dirs(self.scale,self.packetsize)
  local out_tree  = torch.Tensor(dirs:size(1),dirs:size(2))
  local fid_tree  = torch.LongTensor(dirs:size(1),dirs:size(2))
  
  sys.tic()
  log.trace("Computing depth map for photo", photo.name, 'at scale 1/'..self.scale)
  
  local tot = 0
  local totmiss = 0
  local position = photo.position

  local percent_complete = 0
  local dirs_dim1 = dirs:size(1)
  local dirs_dim2 = dirs:size(2)
  local total_dirs = dirs_dim1 * dirs_dim2
  
  for ri = 1,dirs_dim1 do
    for ci = 1,dirs_dim2 do      
      local current_progress = math.floor(((ri-1)*dirs_dim2 + ci) / (total_dirs) * 100)

      if current_progress > percent_complete then
        percent_complete = current_progress 
        log.trace("Computing depth map for photo", photo.name, percent_complete.."%")
      end

      local ray = Ray.new(position,dirs[ri][ci])
      local tree_d, tree_fid = bihtree.traverse(tree,target,ray) 

      tot = tot + 1
      out_tree[ri][ci] = tree_d
      fid_tree[ri][ci] = tree_fid
      if (tree_d == math.huge) then
        totmiss = totmiss + 1
      end
    end
  end
  
  log.trace("Done computing depth map for photo", photo.name, sys.toc())
  
  log.trace("Interpolating for", totmiss, "missed edges out of", tot)
  sys.tic()
  interpolate.math_huge(out_tree)
  log.trace("Interpolation done", sys.toc())

  image.display{image={out_tree},min=0,max=10,legend=photo.name}

  local output_file = self:file(photo)
  log.trace("Saving depth map:", output_file)
  torch.save(output_file, out_tree)  
  return out_tree
end

function Occlusions:show(force_calc)
  local occlusions = self:get()
  local photos = self.scan:get_photos()
    
  for i=1, #photos do
    if occlusions[i] then
      image.display{image={occlusions[i]}, min=0, max=10, legend=photos[i].name}
    elseif force_calc then      
      self:calc_for_photo(photos[i])
    end
  end
end

-- calculate occlusions for all poses
function Occlusions:calc()
  local occlusions = {}  
  local photos = self.scan:get_photos()
  
  for i=1, #photos do
    occlusions[i] = self:calc_for_photo(photos[i])
  end
  
  self.occlusions = occlusions
end

-- Intentionally very slow.  Checks _all_ the faces. Returns closest
-- intersection. Used for debugging the aggregates.
function Occlusions:get_occlusion_slow(ray,debug)
  local obj = self:get_target()
  
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