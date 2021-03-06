local fs = require'fs'
local path = require'path'

local ArcFile = Class()

function ArcFile:__init(name, arc_path)
  self.path = arc_path..'/'..name
  self.filename = path.join(data.Arc.CACHE_ROOT, arc_path, name)
  self.server_mod_time = 0 -- msecs, 0 means it doesn't exist on the server
end

-- true of the local file exits and local mod time > server mod time
function ArcFile:needs_upload()
  if fs.existsSync(self.filename) then
    local stats = fs.statSync(self.filename)
    return stats.mtime * 1000 > self.server_mod_time
  end

  return false
end

--true if the local file doesn't exist or the local mod time < server mod time
function ArcFile:needs_download() 
  if fs.existsSync(self.filename) then
    local stats = fs.statSync(self.filename)
    return stats.mtime * 1000 < self.server_mod_time
  end

  return true
end

function ArcFile:import(filename)
  os.execute("cp '"..filename.."' '"..self.filename.."'")
end

-- forced upload, no checks
function ArcFile:upload()
  net.Depot.put_arc_file(self.path, self.filename, function(err, result)
    if result then 
      self.server_mod_time = result
      fs.utimeSync(self.filename, self.server_mod_time/1000, self.server_mod_time/1000)
    else
      log.error(err)
    end
  end)
end

-- download the file if we don't have a local copy
function ArcFile:download(callback)
  if not self:needs_download() then callback(nil, self.filename) return end
  net.Depot.get_arc_file(self.path, self.filename, function(err)
    if not err then
      fs.utimeSync(self.filename, self.server_mod_time/1000, self.server_mod_time/1000)
    end
    callback(err, self.filename)
  end)
end

-- upload the file if it has changed
function ArcFile:save()
  if self:needs_upload() then self:upload() end
end


-- download the if it's stale and return the file handle
function ArcFile:open(mode, callback)
  self:download(function (err)
    p(err)
  end)
end

function ArcFile.parsed()

end

