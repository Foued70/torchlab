require 'torch'

local paths      = require 'paths'
local config     = require 'model.config'
local loader     = require 'data.loader'

local fs         = util.fs
local Obj        = data.Obj
local Sweep      = model.Sweep
local intersect  = geom.intersect
local bihtree    = model.bihtree

local LensSensor = projection.LensSensor

local Scan       = Class()

local MODEL_FILE_EXTENSION = '.obj'

function Scan:__write_keys()
  return {'path', 'camera_id', 'sweeps', 'poses', 'model_file', 'pose_file'}
end

function Scan:__after_read()
  for i=1, #self.sweeps do
    self.sweeps[i].scan = self
  end
end

-- scan_path: a dir (required).
-- pose_file: .txt file (optional). When nil, will try to use scan_path to find pose file.
-- obj_file: .obj file (optional). When nil, will try to use pose_file or scan_path to find model
function Scan:__init(scan_path, pose_file, obj_file)
  if not scan_path then error('arg #1 invalid, cannot be nil') end
  
  if paths.dirp(scan_path) then    
    self.path = scan_path

    self:set_sweeps()

    log.trace("pose_file", pose_file)
    self:set_poses(pose_file)

    self:set_model_file(obj_file or pose_file)
  else
    error('arg #1 must be valid directory')
  end  
end

-- file_path: any file path (optional)
function Scan:set_model_file(file_path)
  -- if there's a file_path and it has the right extension, use it. 
  -- otherwise try looking in the file_path's folder, then in scan_path folder
    
  if file_path and paths.filep(file_path) then
    if fs.extname(file_path) == MODEL_FILE_EXTENSION then 
      self.model_file = file_path
      return
    else
      local model_files = fs.files_only(paths.dirname(file_path), MODEL_FILE_EXTENSION)
      if model_files and #model_files > 0 then
        self.model_file = model_files[1]
        return
      end
    end
  end
  
  local model_files = fs.files_only(self.path, MODEL_FILE_EXTENSION)
  if model_files and #model_files > 0 then
    self.model_file = model_files[1]
    return 
  end
  
  log.trace("no model file found")
end

function Scan:get_model_data()
  if not self.model_file or not paths.filep(self.model_file) then 
    log.trace('Could not get model data. No file found.')
    return nil 
  end
  
  if not self.model_data then 
    sys.tic()
    log.trace("Loading model data from", self.model_file)    
    self.model_data = loader(self.model_file, Obj.new)
    log.trace('Model loaded in', sys.toc())
  end
  
  return self.model_data
end

function Scan:flush_model_data()
  self.model_data = nil
  collectgarbage()
end

function Scan:get_bihtree()
  if not self.bihtree then
    sys.tic()
    self.bihtree = bihtree.build(self:get_model_data())
    log.trace('built tree in', sys.toc())
  end

  return self.bihtree
end

function Scan:set_sweeps()  
  local sweeps_dirs = fs.dirs_only(self.path, config.sweep_folder_prefix)
  if not sweeps_dirs or #sweeps_dirs == 0 then log.trace('no sweeps dirs') return end
  self.sweeps = {}    
  for i, v in ipairs(sweeps_dirs) do
    -- make a sweep for the img dir even if there are no imgs in it b.c. assumption is that sweep idx = pose idx
    table.insert(self.sweeps, Sweep.new(self, v))
  end
  
  self:init_sweeps_poses()    
end

function Scan:set_poses(pose_file)
  -- guess the pose file based on scan path if pose file path not provided or isn't a file
  if not pose_file or not paths.filep(pose_file) then
    local txt_files = fs.files_only(self.path, '.txt')
    if txt_files and #txt_files > 0 then 
      pose_file = txt_files[1] 
    else
      log.trace('no pose file found')
      return
    end
  end
  self.pose_file = pose_file
  self.poses = model.mp.poses(pose_file)  
  self:init_sweeps_poses()
end

function Scan:init_sweeps_poses()
  if not self.poses or #self.poses == 0 or not self.sweeps or #self.sweeps == 0 then return end
    
  for i, sweep in ipairs(self.sweeps) do
    local pose = self:get_pose(i)
    sweep:set_pose(pose)
  end
end


function Scan:get_photos()
  if not self.photos then
    local photos = {}
    for i=1, #self.sweeps do
      for j=1, #self.sweeps[i].photos do
        table.insert(photos, self.sweeps[i].photos[j])
      end
    end
    self.photos = photos
  end
  
  return self.photos
end

function Scan:get_depth_maps(scale, packetsize, only_cached)
  if not self.depth_maps then
    photos = self:get_photos()
    local depth_maps = {}
    for i=1, #photos do
      table.insert(depth_maps, photos[i]:get_depth_map(scale, packetsize, only_cached))
    end
    self.depth_maps = depth_maps
  end
  return self.depth_maps
end

function Scan:show_depth_maps(scale, packetsize, only_cached)  
  local photos = self:get_photos()
  for i=1, #photos do
    local map = photos[i]:get_depth_map(scale, packetsize, only_cached)
    if map then image.display{image={map}, min=0, max=10, legend=photos[i].name} end
  end
end

function Scan:get_pose(idx)
  -- try to get the pose at the idx but if that pose does not exist, try each idx-1 until arriving at the first pose
  if idx == 1 then return self.poses[1] end  
  if self.poses[idx] then return self.poses[idx] end
  return self:get_pose(idx-1)  
end

-- Intentionally very slow.  Checks _all_ the faces. Returns closest
-- intersection. Used for debugging the aggregates.
function Scan:get_occlusions_slow(ray,debug)
  local obj = self:get_model_data()
  
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

-- file_path: save in a certain location or with certain file name (optional)
function Scan:save(file_path)
  local default_filename = 'scan.lua'
  if file_path then 
    if paths.dirp(file_path) then file_path = paths.concat(file_path, default_filename) end
  else 
    file_path = paths.concat(self.path, default_filename)
  end
  
  torch.save(file_path, self)
end