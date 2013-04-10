local paths = require "paths"
Class()

local function dirCheck(fname, prefix, show_hidden)
  if fname == '.' or fname == '..' then return end
  if prefix and not fname:find("^"..prefix) then return end
  if not show_hidden and fname:find("^%..+") then return end
  return true
end

function dirs_only(dir_path, prefix, show_hidden)  
  if not dir_path or not paths.dirp(dir_path) then return end
  local dirs = {}
  for f in paths.files(dir_path) do
    if dirCheck(f, prefix, show_hidden) then
      local dir = paths.concat(dir_path, f)      
      if paths.dirp(dir) then table.insert(dirs, dir) end      
    end
  end
  
  return dirs
end

-- find all the files in a directory
-- optionally pass file extension if you only want files of that type
function files_only(dir_path, ...)
  if not paths.dirp(dir_path) then return end
  
  local files = {}
  for f in paths.files(dir_path) do
    local file_path = paths.concat(dir_path, f)
    if paths.filep(file_path) then
      local typeMatch = false
      
      local ext = extname(file_path)  
      for i, v in ipairs{...} do
        if v == ext then
          typeMatch = true
          break
        end
      end
      
      if typeMatch or #{...} == 0 then table.insert(files, file_path) end
    end
  end
  
  return files
end

function extname(file_path)
  return file_path:match("%..+$")
end