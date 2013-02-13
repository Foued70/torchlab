local http = require "socket.http"
local url = require "socket.url"
local ltn12 = require "ltn12"
local paths = require "paths"
local _ = require "util/underscore"
local qs = require "util/qs"
local c = require("util/cookie")
local cookie = c.cookie
local cookie_jar = c.jar:new()

-- fix ltn12.source.cat to work with lua 5.1
function ltn12.source.cat(...)
  local arg = {...}
  local src = table.remove(arg, 1)
  return function()
    while src do
      local chunk, err = src()
      if chunk then return chunk end
      if err then return nil, err end
      src = table.remove(arg, 1)
    end
  end
end

local Request = {}

function Request:new(options)
  local r = {}
  setmetatable(r, self)
  self.__index = self
  
  for k, v in pairs(options) do
    if not self[k] then
      r[k] = v
    else
      if type(v) == 'function' then options[k] = nil end
    end
  end  
  options = _.extend({}, options)
  
  return r:init(options)
end

function Request:set_header(name, value)
  if self.headers[name] then 
    self.headers[name] = self.headers[name]..','..value
  else 
    self.headers[name] = value
  end
  return self
end

local function file_size(file)
  local current = file:seek() 
  local size = file:seek("end")
  file:seek("set", current) 
  return size
end

local function gen_boundary()
  local t = {"BOUNDARY-"}
  for i=2,17 do t[i] = string.char(math.random(65,90)) end
  t[18] = "-BOUNDARY"
  return table.concat(t)
end

local function gen_source(boundary, files, data)
  local r = {}
  if data then
    local str_data = {}
    _.each(data, function(v, k)
      table.insert(str_data, "--"..boundary.."\r\n")
      table.insert(str_data, 'content-disposition: form-data; name="'..k..'"\r\n\r\n'..v..'\r\n')
    end)
    table.insert(r, ltn12.source.string(table.concat(str_data)))    
  end
  
  _.each(files, function(v, k)
    if type(v) == 'string' and paths.filep(v) then
      local file = ltn12.source.file(io.open(v, "r"))
      local str_data = {}
      
      table.insert(str_data, "--"..boundary.."\r\n")
      table.insert(str_data, 'content-disposition: form-data; name="'..k..'";')
      table.insert(str_data, 'filename="'..paths.basename(v)..'"\r\ncontent-type: application/octet-stream\r\n\r\n')
      table.insert(r, ltn12.source.string(table.concat(str_data)))     
      table.insert(r, file)
      table.insert(r, ltn12.source.string("\r\n"))
    end
  end)
  
  
  if #r > 0 then
    r[#r+1] = ltn12.source.string("--"..boundary.."--\r\n")
  else
    r[#r+1] = ltn12.source.empty()
  end
  
  return ltn12.source.cat(unpack(r))  
end

function Request:init(options)
  if not options then options = {} end
  
  if self.url then 
    self.uri = self.url
    self.url = nil
  end
  
  if not self.uri then
    error("options.uri is a required argument")
  else
    local parsed_uri = url.parse(self.uri)
    if not parsed_uri.host then
      error("Invalid URI "..self.uri)
    end
  end

  if self.headers then
    self.headers = _.extend({}, self.headers)
  else
    self.headers = {}
  end
  
  self:jar()
  
  self.method = self.method or 'GET'
  
  if self.files then
    --p('UPLOAD FILE', self.files)
    self.boundary = gen_boundary()    
    self:set_header('content-type', 'multipart/form-data; boundary='..self.boundary)      
    self.source = gen_source(self.boundary, self.files, self.data)
  elseif self.data then
    if type(self.data) == 'table' then
      self.data = qs.stringify(self.data)
    end
    self.source = ltn12.source.string(self.data)
    self:set_header('content-type', "application/x-www-form-urlencoded")
    self:set_header('content-length', #self.data)
  end
  
  local sink = false
  if not self.sink then
    sink = {}
    self.sink = ltn12.sink.table(sink)
  elseif type(self.sink) == 'string' then
    self.sink = ltn12.sink.file(io.open(self.sink, "w"))    
  end
    
  --p('URI:', self.uri, 'METHOD:', self.method, 'HEADERS:', self.headers, 'SINK:', self.sink, 'SOURCE:', self.source)
  
  local resp, code, headers, status =  http.request{
    url = self.uri, 
    method = self.method,
    headers = self.headers,
    sink = self.sink,
    source = self.source
  }
  
  if headers['set-cookie'] and not self._disableCookies then
    -- TODO: set-cookie = array?
    local new_cookie = cookie:new(headers['set-cookie'], self.uri)
    -- no local cookie jar
    cookie_jar:add(new_cookie)
  end
  
  if sink then   
    resp = sink
  end  
  
  return resp, code, headers, status
end

function Request:jar(jar)
  local cookies
  -- TODO: redirects?
  if self._redirectsFollowed == 0 then
    self.originalCookieHeader = self.headers.cookie
  end
  
  if type(jar) == 'boolean' and not jar then
    cookies = false
    self._disableCookies = true
  else
    -- fetch cookie from the global cookie jar
    cookies = cookie_jar:serialize(self.uri)
  end
    
  if cookies and #cookies > 0 then    
    if self.originalCookieHeader then
      -- Don't overwrite existing Cookie header
      self.headers.cookie = self.originalCookieHeader .. '; ' .. cookies
    else
      self.headers.cookie = cookies
    end
  end  

  return self
end

local function init_params(uri, options)
  if options and type(options) == 'table' then
    options.uri = uri
  elseif type(uri) == 'string' then
    options = {uri = uri}
  else 
    options = uri    
  end
  return options
end

local function jar()
  return cookie_jar
end

local function request(options)
  return Request:new(options)
end

local function get(uri, options)
  return request(init_params(uri, options))
end

local function post(uri, options)
  local params = init_params(uri, options)
  params.method = 'POST'
  return request(params)
end

local function put(uri, options)
  local params = init_params(uri, options)
  params.method = 'PUT'
  return request(params)
end


local exports = {}
exports.jar = jar
exports.get = get
exports.post = post
exports.put = put

return exports