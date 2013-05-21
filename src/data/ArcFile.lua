local ArcFile = Class()

function ArcFile:__init(name, path)
  self.path = path..'/'..name
  self.abs_filename = paths.concat(data.Arc.CACHE_ROOT, path, name)
  self.server_mod_time = 0 -- msecs, 0 means it doesn't exist on the server
end

-- true of the local file exits and local mod time > server mod time
function ArcFile:needs_upload() 
  local local_mod_time = sys.fstat(self.abs_filename)
  return local_mod_time ~= nil and local_mod_time * 1000 > self.server_mod_time
end

--true if the local file doesn't exist or the local mod time < server mod time
function ArcFile:needs_download() 
  local local_mod_time = sys.fstat(self.abs_filename)
  return local_mod_time == nil or local_mod_time * 1000 < self.server_mod_time
end

function ArcFile:import(filename)
  os.execute("cp '"..filename.."' '"..self.abs_filename.."'")
end

-- forced upload, no checks
function ArcFile:upload()
  local result = net.Depot.put_arc_file(self.path, self.abs_filename)
  if result then 
    self.server_mod_time = result
    util.utime.set_mod_time(self.abs_filename, self.server_mod_time)
  end
end

-- download the file if we don't have a local copy
function ArcFile:download()
  if not self:needs_download() then return end
  local result = net.Depot.get_arc_file(self.path, self.abs_filename)
  util.utime.set_mod_time(self.abs_filename, self.server_mod_time)
  return result
end

-- upload the file if it has changed
function ArcFile:save()
  if self:needs_upload() then self:upload() end
end


-- download the if it's stale and return the file handle
function ArcFile:open(mode)
  self:download()
  return io.open(self.abs_filename, mode)
end

function ArcFile.parsed()

end

