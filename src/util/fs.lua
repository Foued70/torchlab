local fs = require 'fs'
local path = require 'path'

Class()


function is_file(file_path)
   if file_path and fs.existsSync(file_path) then
      local stats = fs.statSync(file_path)
      if stats then 
         return stats.is_file
      end
   end
   return false
end

function is_dir(dir_path)
   if dir_path and fs.existsSync(dir_path) then 
      local stats = fs.statSync(dir_path)
      if stats then 
         return stats.is_directory
      end
   end
   return false
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


function mkdir_p(dir_path, mode)
  mode = mode or '0777'
  local parent_path = path.dirname(dir_path)
  if not fs.existsSync(parent_path) then mkdir_p(parent_path, mode) end
  if not fs.existsSync(dir_path) then fs.mkdirSync(dir_path, mode) end
end

--note that this spawns off a child process and performs op in that, so things like cd will not work
function exec(cmd) 
  local io = require 'io'
  h = io.popen(cmd) 
  line = h:read("*a") 
  h:close() 
  print(line) 
  return line 
end

