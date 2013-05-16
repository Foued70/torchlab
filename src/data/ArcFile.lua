local ArcFile = Class()

function ArcFile:__init(name, path)
  self.path = path..name
  self.abs_filename = paths.concat(data.Arc.CACHE_ROOT, path, name)
end

function ArcFile:snap_time()
  self.last_sync_time = sys.fstat(self.abs_filename)
end

function ArcFile:needs_sync() 
  local current_mod_time = sys.fstat(self.abs_filename)
  return current_mod_time ~= nil and current_mod_time > self.last_sync_time
end

function ArcFile:import(filename)
  os.execute("cp '"..filename.."' '"..self.abs_filename.."'")
  -- set last_sync_time to 0 so it will always be > current_mod_time so it will get up synced
  self.last_sync_time = 0
end

function ArcFile:upload()
  net.Depot.put_arc_file(self.path, self.abs_filename)
end

-- download the file if we don't have a local copy
function ArcFile:download()
  if paths.filep(self.abs_filename) then return end
  local result = net.Depot.get_arc_file(self.path, self.abs_filename)
  self:snap_time()
  return result
end

-- upload the file if it has changed
function ArcFile:save()
  if self:needs_sync() then self:upload() end
end

function ArcFile:open(mode)
  return io.open(self.abs_filename, mode)
end

function ArcFile.parsed()

end

