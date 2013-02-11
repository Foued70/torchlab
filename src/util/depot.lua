local json = require "json"
local sys = require "sys"
local qs = require "util/qs"
local request = require "util/request"
local host = "http://depot.floored.com/"
local login_url = host.."comrade/login.json"
local asset_dir = '../assets/'

local function login(username, password)
  return request.post(login_url, {data = {username = username, password = password}})
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

local function scan_name(scan_path)
  return scan_path:gsub('/', '_') 
end

local function download_url(scan_path)
  return host.."comrade/jobs/"..scan_path.."/download"
end

local function load_local(scan_path)
  local folder = asset_dir..scan_path
  local found = false
  
  if sys.dirp(folder) then
    for f in sys.files(folder) do
      if sys.basename(f):find('.obj') then
        found = sys.concat(asset_dir..'/'..f)
        break
      end
    end   
  end 
  
  return found
end

local function load_remote(scan_path)
  local folder = asset_dir..scan_name(scan_path)
  local filepath = folder..".zip"    
  local resp, stat_code = request.get(download_url(scan_path), {sink = filepath})
  if stat_code == 200 and sys.filep(filepath) then  
    sys.execute("unzip ".. filepath.. " -d "..folder)
    sys.execute("rm "..filepath)
    return load_local(scan_path)
  end
  
  return false
end

local function get(scan_path)
  if not sys.dirp(asset_dir) then sys.execute("mkdir -p "..asset_dir) end    
  return load_local(scan_path) or load_remote(scan_path)  
end

local function put()
end


local exports = {}

exports.get = get
exports.put = put
exports.login = login
exports.is_logged_in = is_logged_in

return exports