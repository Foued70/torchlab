local fs = require'fs'
local path = require 'path'
local debug = require 'debug'
local Watcher = require('uv').Watcher


local loaded_classes = {}

local function get_class_for_name(name)
  local package = _G
  for part in name:gmatch('[^.]+') do
    package = package[part]
  end

  return package
end


local function create_class_for_name(name, class)
  local package = _G
  local last_package, last_part
  for part in name:gmatch('[^/]+') do
    local next_package = rawget(package, part)
    if not next_package then
      -- we had to create this part.  keep track tso we can us class
      rawset(package, part, {})
      last_package = package
      last_part = part
    else
      -- this part already existed
      last_package = nil
      last_part = nil
    end

    package = package[part]
  end

  if last_part then
    -- this is a ne class, use the existing fenv that luvit created
    last_package[last_part] = class
    class.__instance_mt__ = {__index = class}
    loaded_classes[name] = class

    local watcher = Watcher:new(class.__filename)
    log.info('watch', class.__filename)
    watcher:on("change", function (event, path)
      log.info('reload', name)
      require(class.__filename)
    end);

  else
    -- this is a reload, use the old class from the original load
    class = package
  end

  return class
end

function _G.Class(parent)
  local info = debug.getinfo(2)
  local filename = info.source:sub(2) -- remove the '@' prefix
  local name = filename:match('/?src/(.+).lua$')
  name = name or filename:match('/?(.+).lua$')
  local class = create_class_for_name(name, getfenv(2))

  if parent then
    setmetatable(class, {__index = parent})
  else
    -- use a function so globals don't tab complete
    setmetatable(class, {__index = function(self, name) return _G[name] end })
  end

  class.__class__ = class
  class.__classname__ = name
  class.__mod_time__ = fs.statSync(filename).mtime
  class.__super__ = parent

  function class.new(...)
    local inst = setmetatable({}, class.__instance_mt__)
    if inst.__init then inst:__init(...) end
    return inst
  end

  setfenv(2, class)

  return class
end


torch.custom_serializer = {}
function torch.custom_serializer.can_write(object)
  return object.__classname__ ~= nil
end

function torch.custom_serializer.write(object, file)
  local keys
  if object.__write_keys then 
    keys = object:__write_keys()
  else
    keys = {}
    for k,v in pairs(object) do table.insert(keys, k) end
  end

  file:writeInt(#keys + 1)
  for _,k in ipairs(keys) do
     file:writeObject(k)
     file:writeObject(object[k])
  end

  file:writeObject('__classname__')
  file:writeObject(object.__classname__)
end

function torch.custom_serializer.after_read(object)
  if object.__classname__ == nil then return end
  
  local class = get_class_for_name(object.__classname__)

  object.__classname__ = nil -- delete from the instance, which will get it back with the class next

  setmetatable(object, class.__instance_mt__)
  if object.__after_read then
    object:__after_read()
  end
end

local function reload_class(class)

end

local function reload() 
  for name, class in pairs(loaded_classes) do
    -- luvit sets __filename
    new_time = fs.statSync(class.__filename).mtime
    if new_time > class.__mod_time__ then
      -- file has changed
      name='../'..name
      log.trace('reload', name)
      require(name)
    end
  end
end


local repl = require 'repl'
local luvit_repl_evaluateLine = repl.evaluateLine
function repl.evaluateLine(line)
  reload()
  return luvit_repl_evaluateLine(line)
end



local function package_class_loader(self, name)
  local package_name = getmetatable(self).package_name
  require('../'..package_name..'/'..name)
  return rawget(rawget(_G, package_name), name)
end


for _, file_name in ipairs(fs.readdirSync(CLOUDLAB_SRC)) do
  local dir_name = path.join(CLOUDLAB_SRC, file_name)
  local stats = fs.statSync(dir_name)
  if stats.is_directory then
    _G[file_name] = {}
    setmetatable(_G[file_name], {
      __index = package_class_loader,
      package_name = file_name
    })

  end 
end

