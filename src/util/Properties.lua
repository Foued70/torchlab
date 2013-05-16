local Properties = Class()

local io = _G.io
local table = _G.table
local p = _G.p


function __newindex(self, name, value)
  table.insert(self.key_order, name)
  self.props[name] = value
end



function Properties:__init(filename)
  self.filename = filename
  self.key_order = {}
  self.props = {}

  local mt = getmetatable(self)
  setmetatable(self.props, mt)
  setmetatable(self, {
    __index = self.props,
    __newindex = __newindex
  })

  for line in io.lines(filename) do
    if line:match("^#") then
      table.insert(self.key_order, line)
    elseif line:match("^%s*$") then  
    else
      local name, value = line:match("([^%s:]+):%s*(.*)")
      table.insert(self.key_order, name)
      self.props[name] = value
    end
  end
end


function Properties:save()
  local file = io.open(self.filename, 'w+')
  for _, line in ipairs(self.key_order) do
    file:write(line)
    if self.props[line] then
      file:write(': ')
      file:write(self.props[line])
    end

    file:write('\n')
  end

  file:close()
end


