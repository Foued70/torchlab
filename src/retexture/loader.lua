require "torch"
require "paths"
require "sys"

local cache_dir = paths.concat(paths.dirname(paths.thisfile()), 'cache')
sys.execute("mkdir -p "..cache_dir)

local function loader(file, init_fn)
  local object = nil
  local cached_file = paths.concat(cache_dir, file:gsub("/","_")..".t7")
  
  if paths.filep(cached_file) then
    sys.tic()
    object = torch.load(cached_file)
    log.trace('Loaded cached file', cached_file, sys.toc())
  else
    object = init_fn(file)
    torch.save(cached_file, object)
    log.trace('Caching file', file)
  end
  return object
end




return loader;