require "torch"
require "paths"
require "sys"

local cache_dir = paths.concat(paths.dirname(paths.thisfile()), 'cache')
sys.execute("mkdir -p "..cache_dir)

local function loader(file, init_fn, ...)
  local object = nil

  -- TODO: Create caching that works on our new Class() system. 
  -- The following cache scheme is broken, and disabled, as it only works for torch classes.
 
  local cached_file = file  
  local torch_file = cached_file:find(".t7$")
   --[[
  cached_file = cached_file:gsub("/", "_")
  if not torch_file then cached_file = cached_file..".t7" end  
  cached_file = paths.concat(cache_dir, cached_file)

  if paths.filep(cached_file) then
    sys.tic()
    object = torch.load(cached_file)
    log.trace('Loaded cached file', cached_file, sys.toc())
  else
    if torch_file then
      object = init_fn(...)
    else
      object = init_fn(file, ...)
    end
    torch.save(cached_file, object)
    log.trace('Caching file', file)
  end
  ]]--

  if torch_file then
      object = init_fn(...)
    else
      object = init_fn(file, ...)
  end

  return object
end

return loader