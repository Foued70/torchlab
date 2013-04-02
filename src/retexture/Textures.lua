require 'torch'
require 'sys'
require 'paths'
require 'math'

local Poses = require 'Poses'

local loader = require 'util.loader'
local ObjData = require 'util.obj'

local output_dir = paths.concat(paths.dirname(paths.thisfile()), 'output')
sys.execute("mkdir -p " .. output_dir)

local Textures = torch.class('Textures')

function Textures:__init(posefile, targetfile, scale, maskdir)
  if not posefile or not targetfile then error('arguments invalid') end
  if not paths.filep(posefile) or not paths.filep(targetfile) then 
    error('pose file or target file does not exist') 
  end  
  
  self.poses = loader(posefile, Poses.new)
  self.target = loader(targetfile, ObjData.new)
  self.scale = scale or 4  
end


function Textures:load_occlusions()
  -- load precomputed pose occlusions
  log.trace("Looking for occlusion info in", output_dir)
  local poses = self.poses
  
  for pose_idx = 1, poses.nposes do
    local pose = poses[pose_idx]
    local depth_map_file = paths.concat(output_dir, pose.depth_map_filename)
    if paths.filep(depth_map_file) then 
      log.trace('Loading occlusions for pose', pose_idx)
      pose.occlusions = torch.load(depth_map_file)
    end    
  end  
end

function Textures:show()
end

return Textures