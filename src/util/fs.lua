local paths = require "paths"
local fs = {} 

local function dirCheck(fname, include_hidden)
  if fname == '.' or fname == '..' then return end
  if not include_hidden and fname:find("^%..+") then return end
  return true
end

function fs.dirs_only(dirPath, include_hidden)
  if not paths.dirp(dirPath) then return end
  local dirs = {}
  for f in paths.files(dirPath) do
    if dirCheck(f, include_hidden) then
      local dir = paths.concat(dirPath, f)      
      if paths.dirp(dir) then table.insert(dirs, dir) end      
    end
  end
  
  return dirs
end

function fs.files_only(dirPath, ...)
  if not paths.dirp(dirPath) then return end
  
  local files = {}
  for f in paths.files(dirPath) do
    local filePath = paths.concat(dirPath, f)
    if paths.filep(filePath) then
      local typeMatch = false
      
      local ext = fs.extname(filePath)  
      for i, v in ipairs{...} do
        if v == ext then
          typeMatch = true
          break
        end
      end
      
      if typeMatch or #{...} == 0 then table.insert(files, filePath) end
    end
  end
  
  return files
end

function fs.extname(filePath)
  return filePath:match("%..+$")
end

return fs