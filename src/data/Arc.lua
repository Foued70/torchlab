local Arc = Class()

CACHE_ROOT = paths.concat(os.getenv('CLOUDLAB_TMP') or os.getenv('TMPDIR'), 'arcs') 

function get(id)
  local arc = Arc.new(id)
  arc:refresh()
  return arc
end

function Arc:__init(path)
  self.path = path
  os.execute("mkdir -p '"..paths.concat(CACHE_ROOT, path).."'")
end


-- Get the latest tree from the server.
-- This won't change any files, but will mark them as stale if a new version exists on the server.
function Arc:refresh()
  local tree = net.Depot.get_arc(self.path)
  self:populate(tree)
end

function Arc:populate(tree)
  for name, value in pairs(tree) do
    if value.last_modified then -- indicates a file and not a directory
      self:file(name, value.last_modified)
    else
      self:dir(name):populate(value)
    end
  end
end

function Arc:file(name, server_mod_time)
  self[name] = rawget(self, name) or data.ArcFile.new(name, self.path)
  self[name].server_mod_time = server_mod_time or 0
  return self[name]
end

function Arc:dir(name)
  self[name] = rawget(self, name) or data.Arc.new(self.path..'/'..name)
  return self[name]
end


-- import files and directories into this Arc.  If your directory looks like this:
-- a
-- |_b
--   |_c
--   |_d
--
-- arc:import('/a/b') -> arc.b.c, arc.b.d
-- arc:import('/a/b/*') -> arc.c, arc.d
-- arc:import('/a/b/c') -> arc.c
--
-- dot files are ignored
function Arc:import(dir_file_glob)
  local dir = paths.dirname(dir_file_glob)
  local files
  if dir_file_glob:sub(-1) == '*' then
    files = paths.dir(dir)
  else
    files = {paths.basename(dir_file_glob)}
  end

  for i, filename in ipairs(files) do
    if filename:sub(1, 1) ~= '.' then -- no dot files
      local abs_name = paths.concat(dir, filename)
      self:import_single(filename, abs_name)
    end
  end
end

-- import the given file or dir to arc.<name>
function Arc:import_single(name, file_or_dir)
  if paths.dirp(file_or_dir) then
    self:dir(name):import(file_or_dir..'/*')
  else
    self:file(name):import(file_or_dir)
  end
end


function Arc:save()
  for name, value in pairs(self) do
    if (value.__class__ == data.Arc or value.__class__ == data.ArcFile) then
      value:save()
    end
  end
end

