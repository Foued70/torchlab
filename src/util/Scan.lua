require 'torch'

local libui = require 'libui'
local paths = require 'paths'
local config = require 'util.config'
local loader = require 'util.loader'

local fs = util.fs
local Obj = util.Obj
local Sweep = util.Sweep
local LensSensor = util.LensSensor

local Scan = Class()

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
    self.camera_id = 'nikon_10p5mm_r2t_full' -- hardcoded for now, can figure it from exif data maybe?
    self.lens_luts = {}
    
    self:set_sweeps()

    log.trace("pose_file", pose_file)
    self:set_poses(pose_file)

    self:set_model_file(obj_file or pose_file)
  else
    error('arg #1 must be valid directory')
  end  
end

function Scan:get_lens(image_data)
  local img_data_ptr, img_data_size = libui.double_storage_info(image_data)

  if not self.lens_luts then self.lens_luts = {} end
      
  if self.lens_luts[img_data_size] == nil then
    local lens_sensor = LensSensor.new(self.camera_id, image_data)
    local rectilinear_lut = lens_sensor:make_projection_map("rectilinear")
    local spherical_lut = lens_sensor:make_projection_map("spherical")
    self.lens_luts[img_data_size] = {sensor = lens_sensor, rectilinear = rectilinear_lut, spherical = spherical_lut}
  end

  return self.lens_luts[img_data_size]
end

function Scan:set_camera_id()
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
  self.poses = util.mp.poses(pose_file)  
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

function Scan:get_pose(idx)
  -- try to get the pose at the idx but if that pose does not exist, try each idx-1 until arriving at the first pose
  if idx == 1 then return self.poses[1] end  
  if self.poses[idx] then return self.poses[idx] end
  return self:get_pose(idx-1)  
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