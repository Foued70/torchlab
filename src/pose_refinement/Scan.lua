require 'torch'
local geom = require 'util.geom'
local Sweep = require 'Sweep'
local Poses = require 'Poses'
local Pose = require 'Pose'
local PoseDataHandler = require 'PoseDataHandler'

local Scan = torch.class('Scan')

--[[
ex_config_table = {}
ex_config_table.scan_folder = 'Perfect_Scan'
ex_config_table.sweep_folder_prefix = 'sweep_'
ex_config_table.image_folder = 'JPG'
ex_config_table.image_extension = 'jpg'
ex_config_table.lens = {} -- lookup table for lens transform
ex_config_table.pose_file_path = 'Perfect_Scan_Pose_Data.txt'
]]--

function Scan:__init(config_table)
  self.config = config_table
  self.poses = PoseDataHandler.read_pose_data(self.config.pose_file_path)
  self.lenses = {}
  self.sweeps = {}

  self:create_lens(self.config.lens)
  self.sweep_lens = 1

  self:setup_sweeps(self.config.scan_folder)
end

function Scan:create_lens(lens)
  table.insert(self.lenses, lens)
end

function Scan:setup_sweeps(scan_folder_path)
  local files = {}
  local sweep_index = 1
  local sweep_directory_path = paths.concat(scan_folder_path, self.config.sweep_folder_prefix..sweep_index)

  --FIXME: This assumes 'sweep_1' through 'sweep_n' exist. 
  --If there is a missing sweep in the middle, this will stop there, and not pick up following sweeps.
  while paths.dirp(sweep_directory_path) do
    log.trace("Found sweep folder: "..sweep_directory_path)

    self:create_sweep(sweep_index, sweep_directory_path)

    sweep_index = sweep_index + 1
    sweep_directory_path = paths.concat(scan_folder_path, self.config.sweep_folder_prefix..sweep_index)
  end
end

function Scan:create_sweep(sweep_number, sweep_folder_path)
  local image_directory_path = paths.concat(sweep_folder_path, self.config.image_folder)

  if paths.dirp(image_directory_path) == false then
    log.trace("Image path ".."\""..image_directory_path.."\" ".."not found. No sweep created.")
    return false
  end

  local image_paths = {}
  for file in paths.files(image_directory_path) do
    local file_path = paths.concat(image_directory_path, file)

    if (file_path:sub(#file_path-#self.config.image_extension+1,#file_path) == self.config.image_extension) then
      log.trace("Found image: "..file_path)
      table.insert(image_paths, file_path)
    end
  end

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

  local sweep = Sweep.new(self.sweep_lens, self.poses[sweep_number].xyz, self.poses[sweep_number].quat, image_paths)
  table.insert(self.sweeps, sweep)
end

return Scan
