local http = require "socket.http"
local url = require "socket.url"
local ltn12 = require "ltn12"
local _ = require "util/underscore"
local qs = require "util/qs"
local c = require("util/cookie")
local cookie = c.cookie
local cookie_jar = c.jar:new()

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
  
  if self.file and self.data then
    p('FILE AND DATA!')
  elseif self.file then
    local file = io.open(self.file, "r")
    self.source = ltn12.source.file(file)
    self:set_header('content-type', 'multipart/form-data')
    self:set_header(file_size(file))
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
    
  local resp, code, headers, status = http.request{
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


local exports = {}
exports.jar = jar
exports.get = get
exports.post = post

return exports