local json = require "json"
local paths = require "paths"
local qs = require "util/qs"
local request = require "util/request"
local host = "http://depot.floored.com/"
local asset_dir = paths.concat(paths.dirname(paths.thisfile())..'/../assets')..'/'

local function login(username, password)
  resp, status_code = request.post(host.."comrade/login.json", {data = {username = username, password = password}})
  return status_code == 200
end

local function is_logged_in()
  local floored_cookie = request.jar():get('floored_session')
  if not floored_cookie then return false end
  
  floored_cookie = json.decode(qs.unescape(floored_cookie.value):match('{.+}'))
  if floored_cookie.auth and floored_cookie.auth.comrade then 
    return true 
  else 
    return false
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
  return host.."comrade/jobs/"..scan_path(scan_name)
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

local function get(scan_name)
  if not paths.dirp(asset_dir) then os.execute("mkdir -p "..asset_dir) end    
  return load_local(scan_name) or load_remote(scan_name)  
end

local function job_name(scan_name)
  return string.gsub(scan_name, "_%a_%d%d$", '')
end

local function put(scan_name)
  local success = false
  local folder = asset_dir..scan_name  
  if paths.dirp(folder) then    
    local filepath = folder..".zip"        
    os.execute("zip -9 -j "..filepath.." "..folder.."/*")    
    resp, stat_code, headers = request.put(job_url(job_name(scan_name))..'/edit.json', {files = {scan = filepath}})
    os.execute("rm "..filepath)    
    if stat_code == 200 then
      success = true 
    end
  end
  
  return success
end


local exports = {}

exports.get = get
exports.put = put
exports.login = login
exports.is_logged_in = is_logged_in

return exports