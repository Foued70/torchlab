local loaded_classes = {}

function _G.Class(parent)
  local info = debug.getinfo(2)
  local filename = info.source:sub(2) -- remove the '@' prefix
  local name = filename:match('/src/(.+).lua$')

  local package = _G
  for part in name:gmatch('[^/]+') do
    if not package[part] then package[part] = {} end
    package = package[part]
  end

  -- the last item in the package chain is the actual class
  local class = package
  if not class.__instance_mt__ then
    class.__instance_mt__ = {__index = class}
    loaded_classes[name] = class
  end

  if parent then
    setmetatable(class, {__index = parent})
  else
    -- use a function so glbals don't tab complete
    setmetatable(class, {__index = function(self, name) return _G[name] end })
  end

  class.new = function (...)
    local inst = setmetatable({}, class.__instance_mt__)
    if inst.__init then inst:__init(...) end
    return inst
  end

  class.__filename__ = filename
  class.__mod_time__ = sys.fstat(filename)

  setfenv(2, class)

  return class
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
