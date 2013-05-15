local loaded_classes = {}

local function get_class_for_name(name)
  local package = _G
  for part in name:gmatch('[^.]+') do
    package = package[part]
  end

  return package
end

local function create_class_for_name(name)
  local package = _G
  for part in name:gmatch('[^.]+') do
    local next_package = rawget(package, part)
    if not next_package then rawset(package, part, {}) end
    package = package[part]
  end

  -- the last item in the package chain is the actual class
  local class = package
  if not class.__instance_mt__ then
    class.__instance_mt__ = {__index = class}
    loaded_classes[name] = class
  end

  return class
end

function _G.Class(parent)
  local info = debug.getinfo(2)
  local filename = info.source:sub(2) -- remove the '@' prefix
  local name = filename:match('/?src/(.+).lua$')
  name = name or filename:match('/?(.+).lua$')
  name = name:gsub('/','.')
  local class = create_class_for_name(name)

  if parent then
    setmetatable(class, {__index = parent})
  else
    -- use a function so globals don't tab complete
    setmetatable(class, {__index = function(self, name) return _G[name] end })
  end

  class.__class__ = class
  class.__classname__ = name
  class.__filename__ = filename
  class.__mod_time__ = sys.fstat(filename)
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




function _G.reload() 
  for name, class in pairs(loaded_classes) do
    new_time = sys.fstat(class.__filename__)
    if new_time > class.__mod_time__ then
      -- file has changed
      package.loaded[name] = nil
      require(name)
    end
  end
end


local function package_class_loader(self, name)
  local package_name = getmetatable(self).package_name
  require(package_name..'.'..name)
  return rawget(rawget(_G, package_name), name)
end


for file_name in paths.files(CLOUDLAB_SRC) do
  local dir_name = paths.concat(CLOUDLAB_SRC, file_name)
  if paths.dirp(dir_name) then
    _G[file_name] = {}
    setmetatable(_G[file_name], {
      __index = package_class_loader,
      package_name = file_name
    })

  end 
end

