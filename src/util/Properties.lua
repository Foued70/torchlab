local Properties = Class()

local io = require'io'
local table = require'table'
local path = require'path'
local fs = require 'fs'

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

  if fs.statSync(filename).is_file then
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


Properties.cloudlab = util.Properties.new(path.join(HOME, '.cloudlab'))
