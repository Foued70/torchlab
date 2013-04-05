-- this is used for testing retexturing when we don't have our own photos and only have mp posefile + remesh
-- takes a posefile path and obj path and return a scan object
-- images must be in the same folder as the posefile
-- to use: 
-- scan = retex.mp.scan(posefile, objfile)
-- tex = retex.Textures(scan)

require 'paths'

local Scan = require 'util.Scan'
local Photo = require 'util.Photo'
local loader = require 'util.loader'
local Poses = require 'retex.Poses'

Class()

function scan_name(posefile, objfile)
  return string.format('%s-%s-mp-scan.t7', paths.basename(posefile), 
                    paths.basename(objfile))
end

function scan(posefile, objfile)
  return loader(scan_name(posefile, objfile), load_scan, posefile, objfile)
end

function load_scan(posefile, objfile)  
  local scan_folder = paths.dirname(posefile)
  local scan = Scan.new(scan_folder, posefile, objfile)  
  local poses = loader(posefile, Poses)
  -- each pose in the posefile corresponds to 1 sweep with 1 photo
  local sweeps = {}
  for i=1, poses.nposes do
    local pose = poses[i]
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
