local Pose = require('PoseSlim')

function read_pose_data(pose_file_path)
  if not pose_file_path or string.len(pose_file_path) == 0 or not paths.filep(pose_file_path) then
    log.trace("No pose file set. No poses read.")
    return
  end

  local file, errors = io.open(pose_file_path, 'r')
  if errors then 
    log.trace("Error(s) reading pose data:", errors)
    return
  end

  local poses = {}
  local file_data = file:read('*all')
  for line in string.gmatch(file_data, "[^\r\n]+") do
    table.insert(poses, Pose.new(line))
  end
  file:close()
  return poses
end