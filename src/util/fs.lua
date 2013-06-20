local fs = require 'fs'
local path = require 'path'

Class()


function is_file(file_path)
  if not file_path then return false end
  local stats = fs.statSync(file_path)
  return stats.is_file
end


function is_dir(dir_path)
  if not dir_path then return false end
  local stats = fs.statSync(dir_path)
  return stats.is_directory
end


-- Really need a FILE GLOB...
function glob(dir, match, out) 
   if not is_dir(dir) then return nil end
   out = out or {}
   for i, f in ipairs(fs.readdirSync(dir)) do
      if not match then
        table.insert(out, dir .. "/" .. f)
      elseif type(match) == 'table' then
         for _,m in pairs(match) do 
            if f:gmatch(m)() then
               table.insert(out, dir .. "/" .. f) 
            end 
         end
      else -- match is a string
         if f:gmatch(match)() then
            table.insert(out, dir .. "/" .. f) 
         end 
      end
   end 
   return out
end


function dirs_only(dir_path, prefix)
  local all = glob(dir_path, prefix)
  local dirs = {}

  for i, f in ipairs(all) do
    if is_dir(f) then table.insert(dirs, f) end
  end

  return dirs
end

-- find all the files in a directory
-- optionally pass file extension if you only want files of that type
function files_only(dir_path, extension)
  local match = extension and extension..'$' or nil
  local all = glob(dir_path, match)
  local files = {}

  for i, f in ipairs(all) do
    if is_file(f) then table.insert(files, f) end
  end

  return files
end

function extname(file_path)
  return path.extname(file_path)
end

