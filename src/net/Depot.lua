local json = require 'json'
local core = require 'core'
local fs = require 'fs'
local table = require 'table'

local qs = net.qs
local request = require'./request'


local Depot = Class()

local props = util.Properties.cloudlab
local host = props.depot_url or 'https://depot.floored.com'

local cookie

function login(email, password)
  if email == nil or password == nil then return end

  request.post(host..'/login.json', {email = email, password = password}, nil, nil, function(err, res, body)
    if res and res.status_code == 200 then
      cookie = res.headers['set-cookie']:match("[^;]+")

      if email ~= props.email or password ~= props.password then
        props.email = email
        props.password = password
        props:save()
      end
    else
      log.error(err or res)
    end
  end)
end

function is_logged_in()
  if not cookie then return false end
  local floored_cookie = json.parse(qs.unescape(cookie):match('{.*}'))
  if floored_cookie.auth and floored_cookie.auth.comrade then 
    return true 
  end
  return false
end

function get_arc(id, callback)
  request.get(host..'/arcs/'..id, {Cookie=cookie}, function(err, res, body)
    if res and res.status_code == 200 then
      local arc = json.parse(body)
      callback(nil, arc)
    else
      callback(err or res)
    end
  end)
end

local put_arc_file_emitter = core.Emitter:new()
local put_arc_file_queue = setmetatable({}, {__index=table})
local put_arc_file_count = 0

put_arc_file_emitter:on('process', function()
  if put_arc_file_count >= 3 or #put_arc_file_queue == 0 then return end

  put_arc_file_count = put_arc_file_count + 1

  local path, filename, callback = unpack(put_arc_file_queue:remove(1))
  request.put(host..'/arcs/'..path, nil, {file = filename}, {Cookie=cookie}, function(err, res, body)
    if res and res.status_code == 200 then
      callback(nil, tonumber(res.headers.last_modified))
    else
      callback(err or res)
    end

    put_arc_file_count = put_arc_file_count - 1
    put_arc_file_emitter:emit('process')
  end)
end)

function put_arc_file(path, filename, callback)
  put_arc_file_queue:insert({path, filename, callback})
  put_arc_file_emitter:emit('process')
end

function get_arc_file(path, filename, callback)
  local sink = fs.createWriteStream(filename) --fs.openSync(filename, 'w')
  --p(sink)
  request.get(host.."/arcs/"..path, {sink=sink, Cookie=cookie}, function(err, res, body)
        sink:close(nil,function(err) if(err) then print(err) end end)
        callback(err)
      end)
end


Depot.login(props.email, props.password)
