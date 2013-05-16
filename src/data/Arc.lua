local Arc = Class()

CACHE_ROOT = paths.concat(os.getenv('CLOUDLAB_TMP') or os.getenv('TMPDIR'), 'arcs') 

function get(id)
  local tree = net.Depot.get_arc(id)
  return Arc.new(tree, id..'/')
end

function Arc:__init(tree, path)
  self.path = path
  os.execute("mkdir -p '"..paths.concat(CACHE_ROOT, path).."'")
  for name, value in pairs(tree) do
    if value == 0 then
      self:add_file(name)
    else
      self[name] = data.Arc.new(value, path..name..'/')
    end
  end
end


function Arc:add_file(name)
  self[name] = data.ArcFile.new(name, self.path)
  return self[name]
end

function Arc:add_dir(name)
  self[name] = data.Arc.new({}, self.path..name..'/')
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
    local sub_arc = rawget(self, name) or self:add_dir(name)
    sub_arc:import(file_or_dir..'/*')
  else
    local arc_file = rawget(self, name) or self:add_file(name)
    arc_file:import(file_or_dir)
  end
end


function Arc:save()
  for name, value in pairs(self) do
    if (value.__class__ == data.Arc or value.__class__ == data.ArcFile) then
      value:save()
    end
  end
end