require 'paths'

local Scan = util.Scan
local Sweep = util.Sweep
local Photo = util.Photo
local LensSensor = util.LensSensor
local loader = require 'util.loader'

Class()

function scan_name(posefile, objfile)
  local sn = ""
  if objfile then
    sn = paths.basename(objfile)
  end
  return string.format("%s-%s-mp-scan.t7", sn, paths.basename(posefile))
end

-- takes a posefile or scan folder path and obj path and return a scan object
-- images must be in the same folder as the posefile
-- useful for testing retexturing on mp data
-- for example: 
-- scan = util.mp.scan(posefile, objfile)
-- tex = retex.Textures(scan)
function scan(file_or_dirpath, objfile)  
  -- return load_scan(file_or_dirpath, objfile)
  return loader(scan_name(file_or_dirpath, objfile), load_scan, file_or_dirpath, objfile)  
end

function load_scan(file_or_dirpath, objfile)
  local scan_folder = file_or_dirpath
  local posefile = nil
  
  if paths.filep(file_or_dirpath) then
    scan_folder = paths.dirname(file_or_dirpath)
    posefile = file_or_dirpath
  end

  local scan = Scan.new(scan_folder, posefile, objfile)    
  -- each pose in the posefile corresponds to 1 sweep with 1 photo
  local sweeps = {}
  for i=1, #scan.poses do
    local pose = scan.poses[i]
    local sweep = Sweep.new(scan, scan_folder)
    sweep.photos = {}
    local photo = Photo.new(sweep, paths.concat(scan_folder, pose.name))
    local sensor = LensSensor.new('matterport')
    local img = photo:get_image()
   
    sensor.image_w = img:size(3)
    sensor.image_h = img:size(2)
    sensor.inv_image_w = 1/sensor.image_w
    sensor.inv_image_h = 1/sensor.image_h
 
    sensor.center_x = sensor.image_w * pose.center_u -- px
    sensor.center_y = sensor.image_h * (1-pose.center_v) -- px
     
    sensor.hfov = pose.degrees_per_px_x
    sensor.vfov = pose.degrees_per_px_y
    sensor.inv_hfov = 1/sensor.hfov
    sensor.inv_vfov = 1/sensor.vfov
    
    photo.lens = {sensor = sensor}
    -- position and rotation of the photo is exactly what the posefile says it is
    photo.position = pose.position
    photo.rotation = pose.rotation    
    photo.rotation_r = torch.Tensor(4):copy(photo.rotation)
    photo.rotation_r:narrow(1,1,3):mul(-1)
    
    table.insert(sweep.photos, photo)
    table.insert(sweeps, sweep)
  end
  scan.sweeps = sweeps
  log.trace('scan loaded')
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