require 'paths'

local Scan = require 'util.Scan'
local Sweep = require 'util.Sweep'
local Photo = require 'util.Photo'
local loader = require 'util.loader'

Class()

function scan_name(posefile, objfile)
  return string.format('%s-%s-mp-scan.t7', paths.basename(posefile), 
                    paths.basename(objfile))
end

-- takes a posefile path and obj path and return a scan object
-- images must be in the same folder as the posefile
-- useful for testing retexturing on mp data
-- for example: 
-- scan = util.mp.scan(posefile, objfile)
-- tex = retex.Textures(scan)
function scan(posefile, objfile)
  return loader(scan_name(posefile, objfile), load_scan, posefile, objfile)
end

function load_scan(posefile, objfile)  
  local scan_folder = paths.dirname(posefile)
  local scan = Scan.new(scan_folder, posefile, objfile)  
  local mp_poses = loader(posefile, poses)
  -- each pose in the posefile corresponds to 1 sweep with 1 photo
  local sweeps = {}
  for i=1, #mp_poses do
    local pose = mp_poses[i]
    local sweep = Sweep.new(scan, scan_folder)
    sweep.photos = {}
    local photo = Photo.new(sweep, paths.concat(scan_folder, pose.name))
    table.insert(sweep.photos, photo)
    table.insert(sweeps, sweep)
  end
  scan.sweeps = sweeps
  scan:init_sweeps_poses()
  
  return scan
end

-- returns a table with data from the posefile
function poses(posefile)
  return loader(posefile, load_poses)
end

-- <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
-- <center v> <degrees per px x> <degrees per px y> 
function load_poses(posefile)
  if not paths.filep(posefile) then return nil end
  
  log.trace('Loading poses from', posefile)
  local mp_poses = {}
  
  for line in io.lines(posefile) do
    local pose_values = {}
    
    for value in line:gmatch("%S+") do    
      table.insert(pose_values, value)
    end

    local pose = {}
    pose.name = pose_values[1]
    pose.rotation = torch.Tensor({pose_values[2], pose_values[3], pose_values[4], pose_values[5]})
    pose.position = torch.Tensor({pose_values[6], pose_values[7], pose_values[8]})
    pose.center_u = pose_values[9]
    pose.center_v = pose_values[10]
    pose.degrees_per_px_x = pose_values[11]
    pose.degrees_per_px_y = pose_values[12]        
    table.insert(mp_poses, pose)
  end
  log.trace(#mp_poses, 'poses loaded.')
  
  return mp_poses
end