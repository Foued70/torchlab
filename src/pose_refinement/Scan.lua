require 'torch'

local paths = require 'paths'
local config = require 'config'
local Pose = require 'PoseSlim'
local fs = require 'util/fs'
local Sweep = require 'Sweep'
local Scan = torch.class('Scan')

local MODEL_FILE_EXTENSION = '.obj'

-- scan_path is a .lua file or a dir 
-- pose_file is a .txt file or nil
-- pose_file is optional, if not included, will try to use scan_path to find model and pose
function Scan:__init(scan_path, pose_file)
  if scan_path and paths.filep(scan_path) and fs.extname(scan_path) == '.lua' then
    self:init_from_file(scan_path)
  else
    self.lenses = {}  

    self:create_lens(config.lens)
    self.sweep_lens = 1    
    
    self.path = scan_path
    self:set_sweeps()
    log.trace("pose_file", pose_file)
    self:set_poses(pose_file)
    self:set_model_file(pose_file)
  end  
end

function Scan:init_from_file(lua_file)
  local scan = torch.load(lua_file)
  self.path = paths.dirname(lua_file)
  self.sweeps = scan.sweeps
  self.poses = scan.poses
  self.lenses = scan.lenses
  self.model_path = scan.model_path    
end

-- file_path is model's filepath or pose's filepath or nil
function Scan:set_model_file(file_path)
  -- 1. if there's a file_path and it has the right extension, use it. 
  -- 2. if file_path has wrong extension, look in that folder. 
  -- 3. if no file_path and scan has a path, try looking there for the model file
  
  local model_dir = self.path
  
  if file_path and paths.filep(file_path) then
    if fs.extname(file_path) == MODEL_FILE_EXTENSION then 
      self.model_file = file_path
      return
    else
      model_dir = paths.dirname(file_path)
    end
  end
  
  if not model_dir then log.trace('No directory to look for model') return end
  
  local model_files = fs.files_only(model_dir, MODEL_FILE_EXTENSION)
  if model_files and #model_files > 0 then
    self.model_file = model_files[1]
  else
    log.trace('No model file found') 
  end
end

function Scan:load_model_data()
  if self.model_file ~= nil then
    self.model_data = require('util').obj2.new(paths.concat(self.path, self.model_file))
    return true
  end

  log.trace("Failed to load model data. No model file set")
  return false
end

function Scan:flush_model_data()
  self.model_data = nil
  collectgarbage()
end

function Scan:create_lens(lens)
  table.insert(self.lenses, lens)
end

function Scan:set_sweeps()
  if not self.path or not paths.dirp(self.path) then log.trace('no scan path') return end  
  local sweeps_dirs = fs.dirs_only(self.path, config.sweep_folder_prefix)
  if not sweeps_dirs or #sweeps_dirs == 0 then log.trace('no sweeps dirs') return end
  self.sweeps = {}  
  for i, v in ipairs(sweeps_dirs) do
    -- make a sweep for the img dir even if there are no imgs in it b.c. assumption is that sweep idx = pose idx
    table.insert(self.sweeps, Sweep.new(self.sweep_lens, v))
  end
  
  self:init_sweeps_poses()
end


function Scan:set_poses(pose_file)
  -- guess the pose file based on scan path if pose file path not provided
  if not pose_file or not paths.filep(pose_file) then
    if self.path and paths.dirp(self.path) then
      local txt_files = fs.files_only(self.path, '.txt')
      pose_file = txt_files[1]
    else
      log.trace('no pose file')
      return
    end
  end

  local f, err = io.open(pose_file, "r")
  if err then log.trace('error opening pose file', err) return end
  
  self.poses = {}
  local t = f:read("*all")
  for line in string.gmatch(t, "[^\r\n]+") do
    table.insert(self.poses, Pose.new(line))
  end
  f:close()
    
  self:init_sweeps_poses()
end

function Scan:init_sweeps_poses()
  if not self.poses or #self.poses == 0 or not self.sweeps or #self.sweeps == 0 then return end
    
  for i, sweep in ipairs(self.sweeps) do
    local pose = self:get_pose(i)
    sweep:set_pose(pose)
  end
end

function Scan:get_pose(idx)
  -- try to get the pose at the idx but if that pose does not exist, try each idx-1 until arriving at the first pose
  if idx == 1 then return self.poses[1] end  
  if self.poses[idx] then return self.poses[idx] end
  return self:get_pose(idx-1)  
end

-- optional file_path to save in a certain location or with certain file name.
function Scan:save(file_path)  
  -- don't save the model data
  self:flush_model_data()
  
  -- don't save the image data in a SweepCamera
  for i, sweep in ipairs(self.sweeps) do
    for j, sweep_cam in ipairs(sweep.cameras) do 
      sweep_cam:flush_image()
    end
  end
  
  local default_filename = 'scan.lua'  
  if file_path then 
    if paths.dirp(file_path) then file_path = paths.concat(file_path, default_filename) end
  else 
    file_path = paths.concat(self.path, default_filename)
  end
  
  torch.save(file_path, self)
end

return Scan
