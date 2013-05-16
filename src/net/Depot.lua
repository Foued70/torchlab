local json = require "json"
local paths = require "paths"
local qs = require "net.qs"
local request = require "net.request"

local asset_dir = paths.concat(paths.dirname(paths.thisfile())..'/../assets')..'/'

local Depot = Class()

local props = util.Properties.new(paths.concat(HOME, '.cloudlab'))
local host = props.depot_url or "http://depot.floored.com"

function login(username, password)
  local resp, status_code = request.post(host.."/login.json", {data = {username = username, password = password}})
  if status_code == 200 then
    if username ~= props.username or password ~= props.password then
      props.username = username
      props.password = password
      props:save()
    end
    return true
  end

  return false
end

function is_logged_in()
  local floored_cookie = request.jar():get('floored_session')
  if not floored_cookie then return false end
  
  floored_cookie = json.decode(qs.unescape(floored_cookie.value):match('{.*}'))
  if floored_cookie.auth and floored_cookie.auth.comrade then 
    return true 
  end
  return false
end

local function scan_path(scan_name)
  return scan_name:gsub('_', '/') 
end

local function load_local(scan_name)
  local folder = asset_dir..scan_name
  local found = false
  
  if paths.dirp(folder) then
    for f in paths.files(folder) do
      if paths.basename(f):find('.obj') then
        found = asset_dir..f
        break
      end
    end   
  end 
  
  return found
end

local function job_url(scan_name)
  return host.."/comrade/jobs/"..scan_path(scan_name)
end

local function load_remote(scan_name)
  local folder = asset_dir..scan_name
  local filepath = folder..".zip"    

  local resp, stat_code = request.get(job_url(scan_name).."/download", {sink = filepath, redirect = false})
  if paths.filep(filepath) then
    if stat_code ~= 200 then
      p('ERROR GETTING MODEL FROM DEPOT', stat_code)
      os.execute("rm "..filepath)      
    else
      os.execute("unzip ".. filepath.. " -d "..folder)
      os.execute("rm "..filepath)      
      return load_local(scan_name)      
    end
  end
  
  return false
end

function get(scan_name)
  if not paths.dirp(asset_dir) then os.execute("mkdir -p "..asset_dir) end    
  return load_local(scan_name) or load_remote(scan_name)  
end

local function job_name(scan_name)
  return string.gsub(scan_name, "_%a_%d%d$", '')
end

function put(scan_name)
  local success = false
  local folder = asset_dir..scan_name  
  if paths.dirp(folder) then    
    local filepath = folder..".zip"        
    os.execute("zip -9 -j "..filepath.." "..folder.."/*")    
    local resp, stat_code, headers = request.put(job_url(job_name(scan_name))..'/edit.json', {files = {scan = filepath}})
    os.execute("rm "..filepath)    
    if stat_code == 200 then
      success = true 
    end
  end
  
  return success
end


function get_arc(id)
  local resp, code = request.get(host.."/arcs/"..id, {redirect = false})
  if code ~= 200 then
    log.error(resp[1])
    return
  end

  local arc = json.decode(table.concat(resp))
  return arc
end

function put_arc_file(path, filename)
  local resp, code = request.put(host.."/arcs/"..path, {files = {file = filename}, redirect = false})
  if code ~= 200 then
    log.error(code, resp[1])
    return false
  end

  return true
end

function get_arc_file(path, filename)
  local resp, code = request.get(host.."/arcs/"..path, {sink = filename, redirect = false})
  if code ~= 200 then
    log.error(code, resp[1])
    return false
  end

  return true
end

Depot.login(props.username, props.password)
