local path = require "path"
local fs = require "fs"

local cache_dir = path.join(__dirname, 'cache')
if not fs.existsSync(cache_dir) then fs.mkdirSync(cache_dir)

local function loader(file, init_fn, ...)
  local object = nil
 
  local cached_file = file  
  local torch_file = cached_file:find(".t7$")
  
  cached_file = cached_file:gsub("/", "_")
  if not torch_file then cached_file = cached_file..".t7" end  
  cached_file = path.join(cache_dir, cached_file)

  if util.fs.is_file(cached_file) then
    object = torch.load(cached_file)
    log.trace('Loaded cached file', cached_file)
  else
    if torch_file then
      object = init_fn(...)
    else
      object = init_fn(file, ...)
    end
    torch.save(cached_file, object)
    log.trace('Caching file', file)
  end
  
  return object
end

return loader
