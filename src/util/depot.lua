local json = require "json"
local qs = require "util/qs"
local request = require "util/request"
local host = "http://depot.floored.com/"
local login_url = host.."comrade/login.json"

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


local exports = {}

return exports