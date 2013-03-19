require 'torch'
require 'paths'
local geom = require 'util.geom'
local Sweep = require 'Sweep'
local pose_data_handler = require 'PoseDataHandler'

local Scan = torch.class('Scan')

--[[
ex_config_table = {}
ex_config_table.scan_folder = 'Perfect_Scan'
ex_config_table.sweep_folder_prefix = 'sweep_'
ex_config_table.processed_pics_folder = 'JPG'
ex_config_table.pic_extensions = 'jpg'
ex_config_table.lens = {} -- lookup table for lens transform
ex_config_table.pose_file_path = 'Perfect_Scan_Pose_Data.txt'
]]--

local MODEL_FILE_EXTENSION = 'obj'

function Scan:__init(config_table, scan_path, pose_file)
  self.config = config_table
  self.scan_path = scan_path


  self.poses = read_pose_data(paths.concat(self.scan_path, pose_file))
  self.lenses = {}
  self.sweeps = {}

  self.model_file = nil
  self.model_data = nil
  self:find_model_file()

  self:create_lens(self.config.lens)
  self.sweep_lens = 1

  self:setup_sweeps()
end

function Scan:find_model_file()
  local model_files = {}
  for file in paths.files(self.scan_path) do
    local file_extension = file:sub(#file-#MODEL_FILE_EXTENSION+1,#file)
    if file_extension == MODEL_FILE_EXTENSION then
        log.trace("Found model: "..file)
        table.insert(model_files, file)
    end
  end
  if #model_files < 1 then
    log.trace("No model file found in directory:", self.scan_path)
    self.model_file = nil
    return false
  elseif #model_files > 1 then
    log.trace("Multiple models found in directory:", "\""..self.scan_path.."\"", "Using first one:", "\""..model_files[1].."\"")
    self.model_file = model_files[1]
    return true
  else
    log.trace("Found model:", "\""..model_files[1].."\"")
    self.model_file = model_files[1]
    return true
  end
end

function Scan:load_model_data()
  if self.model_file ~= nil then
    self.model_data = require('util.obj2').new(paths.concat(self.scan_path, self.model_file))
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

function Scan:setup_sweeps()
  local files = {}
  local sweep_index = 1
  local sweep_directory_path = paths.concat(self.scan_path, self.config.sweep_folder_prefix..sweep_index)

  --FIXME: This assumes 'sweep_1' through 'sweep_n' exist. 
  --If there is a missing sweep in the middle, this will stop there, and not pick up following sweeps.
  while paths.dirp(sweep_directory_path) do
    log.trace("Found sweep folder: "..sweep_directory_path)

    self:create_sweep(sweep_index, sweep_directory_path)

    sweep_index = sweep_index + 1
    sweep_directory_path = paths.concat(self.scan_path, self.config.sweep_folder_prefix..sweep_index)
  end
end

function Scan:create_sweep(sweep_number, sweep_folder_path)
  local image_directory_path = paths.concat(sweep_folder_path, self.config.processed_pics_folder)

  if paths.dirp(image_directory_path) == false then
    log.trace("Image path ".."\""..image_directory_path.."\" ".."not found. No sweep created.")
    return false
  end

  local image_paths = {}
  for file in paths.files(image_directory_path) do
    local file_path = paths.concat(image_directory_path, file)

    local file_extension = file_path:sub(#file_path-#self.config.pic_extensions,#file_path)
    for i=1, #self.config.pic_extensions do
      if file_extension == self.config.pic_extensions[i] then
        log.trace("Found image: "..file_path)
        table.insert(image_paths, file_path)
        break
      end
    end
  end

  log.trace(image_paths)
  log.trace(#image_paths)
  if #image_paths < 1 then
    log.trace("No images found in directory: ".."\""..image_directory_path.."\"".." No sweep created.")
    return false
  end

  --Sort image paths alphabetically. TODO: Might need a better way to sort images sequentially
  table.sort(image_paths, function (a,b) return a < b end)

  if self.poses[sweep_number] == nil then
    log.trace("No pose data found for sweep "..sweep_number..". No sweep created.")
    return false
  end

  local sweep = Sweep.new(self.sweep_lens, self.poses[sweep_number], image_paths)
  table.insert(self.sweeps, sweep)
  return true
end

return Scan
