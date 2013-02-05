require 'paths'

local project_dir = "@project_dir@"
local install_root = "@install_root@"
local current_dir = paths.cwd()
local current_dir_name = paths.basename(current_dir)

local build_dir = paths.concat(install_root, 'build', 'cloudlab', current_dir_name)

local cmake = 'cmake '..current_dir

local exec = os.execute
-- local exec = print

if not paths.filep(paths.concat(build_dir, 'CMakeCache.txt')) then
  exec('mkdir -p '..build_dir..'; cd '..build_dir..'; ' .. cmake)
end

exec('cd '..build_dir..'; make install')
